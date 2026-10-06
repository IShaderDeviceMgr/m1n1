/* SPDX-License-Identifier: MIT */

/*
 * Suspend-to-RAM experiment for T6000 (J314s), run from m1n1 stage 2 before
 * anything else is booted. See j314s-notes SLEEP.md §3.2.
 *
 * Replays what macOS 13.5 does on the way into S2R, as far as it applies to a
 * system with no drivers running:
 *   AppleSMC:        MBSE = 'slpw', MBSE = 'slpa', RTKit AP power state sleep
 *                    (0x201)
 *   AppleT6000PMGR:  quiesceHW(): AFR PS off, SIO / SIO_CPU auto-PM off, SEP
 *                    PG wait off, SOC clock dividers parked, PLL off mode,
 *                    media PS check, startS2RTimer()
 *   XNU:             arm64_prepare_for_sleep(deep) = cpu_sleep(true)
 * The boot CPU's RVBAR is locked to m1n1's _vectors_start, and m1n1 is
 * reserved memory, so a wake re-enters m1n1 at cpu_reset; _cpu_reset_c sends
 * it to s2r_resume() when the record below says we went to sleep.
 *
 * Only runs with `s2rtest=<mode>` in the m1n1 config and the charger
 * connected (SMC CHIS != 0); unplugging the charger always boots normally.
 * Modes stop at different points to find what resets the machine:
 *   1  full sequence, deep WFI
 *   2  SMC messages only, then count seconds (no PMGR writes, no WFI)
 *   3  SMC + PMGR quiesce + S2R timer, then count seconds (no WFI)
 *   4  SMC + PMGR quiesce, then count seconds (no S2R timer, no WFI)
 *   5  as 1, but first start the secondary CPUs and park them in deep sleep
 *      (smp_stop_secondaries(true), like XNU's per-CPU ml_arm_sleep), and
 *      print the cluster power-state registers before and after
 *   6  as 5, then power the non-boot clusters off like quiesceHW's
 *      enableCluster(c, false): ClusterCtl[1], [2] (the P-clusters; run 6
 *      showed only those react to parking the secondaries) target := 0
 *   7  as 6, plus: list every PMGR power state still on (before the
 *      countdown), and force the display power states off last (child
 *      first, like quiesceHW does for die 1 on t6002); the screen goes dark
 *   8  as 6 (screen stays lit), but the resume path does nothing except draw
 *      row 1 and count seconds on row 2: tells a real reset apart from an
 *      RVBAR re-entry whose re-init crashes
 *   9  as 6, plus power off every PS that iBoot left on and Linux does not
 *      mark apple,always-on (s2r_psoff.h, children first, display last),
 *      the way Linux's pmgr-pwrstate does; one bar per PS from row 3 on
 *      (short bar = did not reach off), the screen goes dark at entry 60
 *  10  as 9, plus mask every AIC interrupt last, like AppleInterruptControllerV2
 *      does on gIOPlatformQuiesceActionKey (MASK_SET 0x6400.., 0xffffffff)
 *
 * Progress is drawn straight into the framebuffer (aligned 32-bit stores,
 * safe with the MMU off), since nothing can be printed after mmu_shutdown():
 *   row 0  one white square per step reached (short bar = step failed):
 *          slpw, slpa, AP sleep, MMU off, AFR, SIO, clocks, PLL, S2R timer,
 *          WFI
 *   row 1  s2r_resume(): entered, watchdog armed, heap, MCC, MMU, console
 *   row 2  modes 2/3: one bar per second since the last step
 *
 * Not done (unknown or not applicable here): the WKTP write (gated by an
 * AppleSMC flag of unknown origin), powering off the non-boot CPU clusters
 * (enableCluster; the secondaries were never started), the AOP AP wake
 * masks (left as iBoot set them; only read).
 */

#include "s2r.h"
#include "fb.h"
#include "heapblock.h"
#include "mcc.h"
#include "memory.h"
#include "smc.h"
#include "smp.h"
#include "string.h"
#include "uart.h"
#include "utils.h"
#include "wdt.h"
#include "xnuboot.h"

#include "s2r_psoff.h"

#define S2R_MAGIC 0x5332522d74657374UL /* "S2R-test" */

/* PMGR (pmgr reg 0 = 0x28e080000, reg 2 = 0x28e580000, ClkGen = reg 4) */
#define PS_AFR          0x28e0801e8 /* regmap 0 + 0x1e8 */
#define PS_SIO          0x28e580180 /* SOC-East (regmap 2) + 0x180 */
#define PS_SIO_CPU      0x28e580188 /* SOC-East + 0x188 */
#define PMGR_SEP_PG     0x28e0cc200 /* regmap 0 + 0x4c200, bit 2 */
#define CLKGEN_BASE     0x28e000000 /* regmap 0x3f */
#define PLL_OFF_MODE    0x28e224000 /* regmap 0x37 */
#define MINIPMGR_S2R    0x2925c4000 /* master mini-PMGR + 0x4000 */
#define AOP_WAKE_MASK   0x292004008 /* AOP global AP controls 0x4008..0x4020 */
#define AOP_WAKE_STATUS 0x292004024 /* 0x4024..0x403c */
/* Reg group 2 (ClusterCtl) = regmap 0 + 0x18000; enableCluster() writes the
 * PS target of physical cluster n at + 4 * n */
#define CLUSTER_CTL     0x28e098000
/* AIC2 (0x28e100000): IRQ_CFG 0x2000 (4096 x u32), SW_SET 0x6000, SW_CLR
 * 0x6200, MASK_SET 0x6400, MASK_CLR 0x6600 (128 words each); reads of
 * MASK_SET return the mask, 1 = masked */
#define AIC_MASK_SET    0x28e106400
#define AIC_INFO1       0x28e100004

#define PS_AUTO_ENABLE BIT(28)
#define PS_TARGET      GENMASK(3, 0)
#define PS_ACTUAL      GENMASK(7, 4)

#define TIMEOUT_US 192000 /* quiesceHW's 0x2ee00 */

static const u32 clkgen_regs[] = {0x38164, 0x3816c, 0x38168, 0x3815c, 0x38170};

/* AppleT6000PMGR::quiesceHW()::mediaPsRegs, the entries used on T6000 */
static const u64 media_ps[] = {
    0x28e080230, 0x28e080268, 0x28e080270, 0x28e0802c8, 0x28e08c000, 0x28e08c008, 0x28e08c010,
    0x28e08c018, 0x28e08c020, 0x28e08c028, 0x28e580390, 0x28e580398, 0x28e5803a0, 0x28e5803b0,
    0x28e5803c0, 0x28e584018, 0x28e584020, 0x28e584028, 0x28e584030, 0x28e584038, 0x28e584040,
    0x28e588000, 0x28e588008, 0x28e588010, 0x28e588018, 0x28e588020,
};

/* Failed steps, for the record */
#define FAIL_AFR    BIT(0)
#define FAIL_SIO    BIT(1)
#define FAIL_CLKGEN BIT(2)
#define FAIL_PLL    BIT(3)
#define FAIL_SLPW   BIT(4)
#define FAIL_SLPA   BIT(5)
#define FAIL_APPWR  BIT(6)
#define FAIL_DISP   BIT(7)

/* Display power states, child before parent: disp0_cpu0, disp0_fe,
 * dispext0_cpu0, dispext0_fe */
static const u64 disp_ps[] = {0x28e580350, 0x28e580328, 0x28e080280, 0x28e080258};

/* PS register windows (ADT pmgr ps-regs), 8-byte stride */
static const struct {
    u64 base;
    u32 len;
} ps_windows[] = {
    {0x28e080100, 0x300}, {0x28e08c000, 0x100}, {0x28e580100, 0x300},
    {0x28e584000, 0x100}, {0x28e588000, 0x100}, {0x292280000, 0x100},
};

/*
 * Kept in .data: m1n1 is reserved memory and an RVBAR entry does not clear
 * BSS, but .data also survives a debugger reading the image.
 */
struct s2r_record {
    u64 magic;
    u64 mode;
    u64 armed;
    u64 wakes;
    u64 cntfrq;
    u64 cntpct_sleep;
    u64 cntpct_wake;
    u32 failed;
    u32 media_on;
    u32 wake_mask[7];
    u32 wake_status_before[7];
    u32 wake_status_after[7];
};

static struct s2r_record rec __attribute__((section(".data"))) = {0};
static int mode;

void s2r_configure(const char *val)
{
    mode = 0;
    for (; *val >= '0' && *val <= '9'; val++)
        mode = mode * 10 + (*val - '0');
    if (mode < 1 || mode > 10)
        mode = 0;
}

/* Breadcrumbs, written to the framebuffer directly (works with the MMU off) */
#define CRUMB_SIZE 48
#define CRUMB_STEP 72

static void crumb_box(int row, int idx, int h, u32 color)
{
    u64 base = cur_boot_args.video.base;
    u64 stride = cur_boot_args.video.stride;
    u64 x0 = 96 + idx * CRUMB_STEP, y0 = 160 + row * (CRUMB_SIZE + 32);

    if (!base || x0 + CRUMB_SIZE > cur_boot_args.video.width ||
        y0 + CRUMB_SIZE > cur_boot_args.video.height)
        return;

    for (u64 y = y0 + CRUMB_SIZE - h; y < y0 + CRUMB_SIZE; y++)
        for (u64 x = x0; x < x0 + CRUMB_SIZE; x++)
            write32(base + y * stride + x * 4, color);
}

static void crumb(int row, int idx, bool ok)
{
    crumb_box(row, idx, ok ? CRUMB_SIZE : CRUMB_SIZE / 4, 0xffffffff);
}

/* Modes 2/3: show seconds until whatever happens next (MMU off) */
static void count_seconds(void) __attribute__((noreturn));
static void count_seconds(void)
{
    for (int i = 0;; i++) {
        crumb_box(2 + i / 30, i % 30, CRUMB_SIZE, 0xffffffff);
        mdelay(1000);
    }
}

/*
 * Published in FDT /chosen as asahi,s2r-record so an S2R attempt from Linux
 * can arm the record (magic @0, mode @8, armed @16) and a wake through RVBAR
 * lands in s2r_resume().
 */
u64 s2r_record_addr(void)
{
    return (u64)&rec;
}

bool s2r_resume_pending(void)
{
    return rec.magic == S2R_MAGIC && rec.armed;
}

static void read_words(u64 base, u32 *out)
{
    for (int i = 0; i < 7; i++)
        out[i] = read32(base + 4 * i);
}

static void print_words(const char *name, const u32 *w)
{
    printf("  %-18s %08x %08x %08x %08x %08x %08x %08x\n", name, w[0], w[1], w[2], w[3], w[4],
           w[5], w[6]);
}

static u32 count_media_on(bool print)
{
    u32 on = 0;

    for (size_t i = 0; i < ARRAY_SIZE(media_ps); i++) {
        u32 v = read32(media_ps[i]);
        if (v & PS_TARGET) {
            on++;
            if (print)
                printf("  media PS 0x%lx still on (0x%08x)\n", media_ps[i], v);
        }
    }
    return on;
}

static void print_clusters(const char *when)
{
    printf("  cluster PS %-7s", when);
    for (int i = 0; i < 4; i++)
        printf(" %08x", read32(CLUSTER_CTL + 4 * i));
    printf("\n");
}

static void print_ps_on(void)
{
    int n = 0;

    printf("  PS still on (addr:value):\n");
    for (size_t w = 0; w < ARRAY_SIZE(ps_windows); w++) {
        for (u32 off = 0; off < ps_windows[w].len; off += 8) {
            u64 addr = ps_windows[w].base + off;
            u32 v = read32(addr);
            if (v == 0xffffffff || !(v & PS_TARGET))
                continue;
            printf("%s%lx:%x", n % 6 ? "  " : "\n   ", addr & 0xfffffff, v);
            n++;
        }
    }
    printf("\n  %d on\n", n);
}

/* pmgr-pwrstate apple_pmgr_ps_set(0) */
static bool ps_off(u64 addr)
{
    u32 v = read32(addr);
    v &= ~(BIT(12) | BIT(10) | PS_AUTO_ENABLE | BIT(9) | BIT(8) | PS_TARGET);
    write32(addr, v);
    return !poll32(addr, PS_ACTUAL, 0, 10000);
}

static void countdown(const char *what, int seconds)
{
    for (int i = seconds; i > 0; i--) {
        printf("\r  %s in %2d s ", what, i);
        mdelay(1000);
    }
    printf("\n");
}

/* AppleT6000PMGR::quiesceHW() steps for one die, MMU off, no printing. */
static void quiesce_pmgr(void)
{
    /* PS 0x1e8 (AFR) := 0, wait actual == 0 */
    write32(PS_AFR, 0);
    if (poll32(PS_AFR, PS_ACTUAL, 0, TIMEOUT_US))
        rec.failed |= FAIL_AFR;

    /* SIO, SIO_CPU: drop auto-enable, wait actual == target */
    u64 sio[] = {PS_SIO, PS_SIO_CPU};
    for (int i = 0; i < 2; i++) {
        u32 v = read32(sio[i]);
        write32(sio[i], v & ~PS_AUTO_ENABLE);
        if (poll32(sio[i], PS_ACTUAL, FIELD_PREP(PS_ACTUAL, v & PS_TARGET), TIMEOUT_US))
            rec.failed |= FAIL_SIO;
    }

    /* _enableWaitSEPPG(false) */
    clear32(PMGR_SEP_PG, BIT(2));

    /* Park the SOC clock dividers: bits 24-27 off, bit 31 on, wait bit 30 clear */
    for (size_t i = 0; i < ARRAY_SIZE(clkgen_regs); i++) {
        u64 reg = CLKGEN_BASE + clkgen_regs[i];
        write32(reg, (read32(reg) & 0x70ffffff) | 0x80000000);
        if (poll32(reg, BIT(30), 0, TIMEOUT_US))
            rec.failed |= FAIL_CLKGEN;
    }
}

/* AppleT6000PMGR::setPLLOffMode(0x37, 0, 3) */
static void pll_off_mode(void)
{
    mask32(PLL_OFF_MODE, GENMASK(5, 3), 3 << 3);
    set32(PLL_OFF_MODE, BIT(1));
    clear32(PLL_OFF_MODE, BIT(0));
    if (poll32(PLL_OFF_MODE, BIT(31), 0, TIMEOUT_US))
        rec.failed |= FAIL_PLL;
}

void s2r_test(void)
{
    if (!mode)
        return;

    fb_set_active(true);
    printf("\n*** S2R test (SLEEP.md 3.2), mode %d ***\n", mode);

    smc_dev_t *smc = smc_init();
    if (!smc) {
        printf("S2R: SMC init failed, skipping\n");
        return;
    }

    u32 chis = 0;
    if (smc_read_u32(smc, 'CHIS', &chis) || !chis) {
        printf("S2R: charger not connected (CHIS=0x%x), skipping; booting normally\n", chis);
        smc_shutdown(smc);
        return;
    }

    memset(&rec, 0, sizeof(rec));
    read_words(AOP_WAKE_MASK, rec.wake_mask);
    read_words(AOP_WAKE_STATUS, rec.wake_status_before);
    rec.cntfrq = mrs(CNTFRQ_EL0);

    printf("S2R: CHIS=0x%x\n", chis);
    print_words("AOP wake masks", rec.wake_mask);
    print_words("AOP wake status", rec.wake_status_before);
    printf("  PS afr %08x sio %08x sio_cpu %08x sep-pg %08x pll %08x\n", read32(PS_AFR),
           read32(PS_SIO), read32(PS_SIO_CPU), read32(PMGR_SEP_PG), read32(PLL_OFF_MODE));
    rec.media_on = count_media_on(true);
    printf("  %u media PS on (macOS panics on any)\n", rec.media_on);

    print_clusters("now");
    if (mode >= 5) {
        smp_start_secondaries();
        smp_stop_secondaries(true);
        print_clusters("parked");
        mdelay(100);
        print_clusters("+100ms");
    }
    if (mode == 6 || mode == 8 || mode >= 9) {
        for (int i = 1; i <= 2; i++)
            mask32(CLUSTER_CTL + 4 * i, PS_TARGET, 0);
        mdelay(10);
        print_clusters("off");
        mdelay(100);
        print_clusters("+100ms");
    }

    if (mode == 7)
        print_ps_on();
    if (mode >= 9)
        printf("  will power off %lu PS (display from #60, screen goes dark)\n",
               ARRAY_SIZE(psoff));
    if (mode == 10) {
        u32 nr = read32(AIC_INFO1) & 0xffff, unmasked = 0;
        for (u32 i = 0; i < nr; i++)
            if (!(read32(AIC_MASK_SET + 4 * (i / 32)) & BIT(i % 32)))
                unmasked++;
        printf("  AIC: %u IRQs, %u unmasked (will mask all)\n", nr, unmasked);
    }

    printf("\nAfter the screen stops updating, wait 30 s, then press the power button once.\n");
    printf("Hold the power button to abort.\n");
    countdown("sleeping", 15);

    if (smc_write_u32(smc, 'MBSE', 'slpw'))
        rec.failed |= FAIL_SLPW;
    if (smc_write_u32(smc, 'MBSE', 'slpa'))
        rec.failed |= FAIL_SLPA;
    if (!smc_set_ap_power(smc, 0x201))
        rec.failed |= FAIL_APPWR;
    printf("S2R: SMC slpw/slpa/AP sleep done (failed=0x%x); going down\n", rec.failed);

    rec.magic = S2R_MAGIC;
    rec.mode = mode;

    /* From here on caches and MMU are off: no printing, writes go to DRAM */
    fb_set_active(false);
    mmu_shutdown();
    sysop("msr daifset, #0xf");

    if (mode == 7) {
        for (size_t i = 0; i < ARRAY_SIZE(disp_ps); i++) {
            write32(disp_ps[i], 0);
            if (poll32(disp_ps[i], PS_ACTUAL, 0, TIMEOUT_US))
                rec.failed |= FAIL_DISP;
        }
    }

    crumb(0, 0, !(rec.failed & FAIL_SLPW));
    crumb(0, 1, !(rec.failed & FAIL_SLPA));
    crumb(0, 2, !(rec.failed & FAIL_APPWR));
    crumb(0, 3, true);
    if (mode == 2)
        count_seconds();

    if (mode >= 9) {
        for (size_t i = 0; i < ARRAY_SIZE(psoff); i++)
            crumb_box(3 + i / 30, i % 30, ps_off(psoff[i].addr) ? CRUMB_SIZE : CRUMB_SIZE / 4,
                      0xffffffff);
    }

    quiesce_pmgr();
    crumb(0, 4, !(rec.failed & FAIL_AFR));
    crumb(0, 5, !(rec.failed & FAIL_SIO));
    crumb(0, 6, !(rec.failed & FAIL_CLKGEN));
    rec.media_on = count_media_on(false);
    pll_off_mode();
    pll_off_mode();
    crumb(0, 7, !(rec.failed & FAIL_PLL));

    if (mode == 4)
        count_seconds();

    rec.cntpct_sleep = mrs(CNTPCT_EL0);
    rec.armed = mode == 1 || mode >= 5;
    sysop("dsb sy");

    if (mode == 10)
        for (int i = 0; i < 128; i++)
            write32(AIC_MASK_SET + 4 * i, 0xffffffff);

    write32(MINIPMGR_S2R, 1); /* startS2RTimer() */
    crumb(0, 8, true);
    if (mode == 3)
        count_seconds();

    crumb(0, 9, true);
    cpu_sleep(true);
}

void s2r_resume(void)
{
    crumb(1, 0, true);
    /* Whatever happens below, come back through iBoot in 90 s */
    wdt_arm(90);
    uart_detach();
    rec.armed = 0;
    if (rec.mode == 8) {
        crumb(1, 1, true);
        crumb(1, 2, true);
        count_seconds();
    }
    rec.wakes++;
    rec.cntpct_wake = mrs(CNTPCT_EL0);
    read_words(AOP_WAKE_STATUS, rec.wake_status_after);
    sysop("dsb sy");

    crumb(1, 1, true);

    heapblock_init();
    crumb(1, 2, true);
    mcc_init();
    crumb(1, 3, true);
    mmu_init();
    crumb(1, 4, true);
    fb_init(false);
    fb_set_active(true);
    crumb(1, 5, true);

    printf("\n*** S2R test: woke up through RVBAR (wake #%lu) ***\n", rec.wakes);
    printf("  failed steps 0x%x, media PS on at sleep: %u\n", rec.failed, rec.media_on);
    printf("  CNTPCT sleep 0x%lx wake 0x%lx (%lu s at %lu Hz)\n", rec.cntpct_sleep,
           rec.cntpct_wake, (rec.cntpct_wake - rec.cntpct_sleep) / (rec.cntfrq ?: 1), rec.cntfrq);
    print_words("AOP wake masks", rec.wake_mask);
    print_words("wake status before", rec.wake_status_before);
    print_words("wake status after", rec.wake_status_after);

    smc_dev_t *smc = smc_init();
    if (smc) {
        u32 mbse = 0;
        u64 mbsw = 0;
        int r1 = smc_read(smc, 'MBSE', &mbse, sizeof(mbse));
        int r2 = smc_read(smc, 'MBSW', &mbsw, sizeof(mbsw));
        printf("  SMC MBSE 0x%08x (%d) MBSW 0x%016lx (%d)\n", mbse, r1, mbsw, r2);
        if (smc_write_u32(smc, 'MBSE', 'waka'))
            printf("  SMC MBSE=waka failed\n");
    } else {
        printf("  SMC init failed after wake\n");
    }

    printf("\nTake a photo; rebooting through the watchdog.\n");
    while (1)
        sysop("wfe");
}
