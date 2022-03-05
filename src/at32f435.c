/*
 * stm32f10x.c
 * 
 * Core and peripheral registers.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

bool_t is_artery_mcu;
unsigned int flash_page_size = FLASH_PAGE_SIZE;
unsigned int ram_kb = 64;

struct extra_exception_frame {
    uint32_t r4, r5, r6, r7, r8, r9, r10, r11, lr;
};

static inline bool_t in_text(uint32_t p)
{
    return (p >= (uint32_t)_smaintext) && (p < (uint32_t)_emaintext);
}

static void show_stack(uint32_t sp, uint32_t top)
{
    unsigned int i = 0;
    sp &= ~3;
    while (sp < top) {
        uint32_t v = *(uint32_t *)sp & ~1;
        sp += 4;
        if (!in_text(v))
            continue;
        if ((i++&7) == 0)
            printk("\n ", sp);
        printk("%x ", v);
    }
    printk("\n");
}

void EXC_unexpected(struct extra_exception_frame *extra)
{
    struct exception_frame *frame;
    uint8_t exc = (uint8_t)read_special(psr);
    uint32_t msp, psp;

    if (extra->lr & 4) {
        frame = (struct exception_frame *)read_special(psp);
        psp = (uint32_t)(frame + 1);
        msp = (uint32_t)(extra + 1);
    } else {
        frame = (struct exception_frame *)(extra + 1);
        psp = read_special(psp);
        msp = (uint32_t)(frame + 1);
    }

    printk("Unexpected %s #%u at PC=%08x (%s):\n",
           (exc < 16) ? "Exception" : "IRQ",
           (exc < 16) ? exc : exc - 16,
           frame->pc, (extra->lr & 8) ? "Thread" : "Handler");
    printk(" r0:  %08x   r1:  %08x   r2:  %08x   r3:  %08x\n",
           frame->r0, frame->r1, frame->r2, frame->r3);
    printk(" r4:  %08x   r5:  %08x   r6:  %08x   r7:  %08x\n",
           extra->r4, extra->r5, extra->r6, extra->r7);
    printk(" r8:  %08x   r9:  %08x   r10: %08x   r11: %08x\n",
           extra->r8, extra->r9, extra->r10, extra->r11);
    printk(" r12: %08x   sp:  %08x   lr:  %08x   pc:  %08x\n",
           frame->r12, (extra->lr & 4) ? psp : msp, frame->lr, frame->pc);
    printk(" msp: %08x   psp: %08x   psr: %08x\n",
           msp, psp, frame->psr);
    
    if ((msp >= (uint32_t)_irq_stackbottom)
        && (msp < (uint32_t)_irq_stacktop)) {
        printk("IRQ Call Trace:");
        show_stack(msp, (uint32_t)_irq_stacktop);
    }

    if ((psp >= (uint32_t)_thread_stackbottom)
        && (psp < (uint32_t)_thread_stacktop)) {
        printk("Process Call Trace (Thread 0):", psp);
        show_stack(psp, (uint32_t)_thread_stacktop);
    } else if ((psp >= (uint32_t)_thread1_stackbottom)
        && (psp < (uint32_t)_thread1_stacktop)) {
        printk("Process Call Trace (Thread 1):", psp);
        show_stack(psp, (uint32_t)_thread1_stacktop);
    }

    system_reset();
}

static void exception_init(void)
{
    /* Initialise and switch to Process SP. Explicit asm as must be
     * atomic wrt updates to SP. We can't guarantee that in C. */
    asm volatile (
        "    mrs  r1,msp     \n"
        "    msr  psp,r1     \n" /* Set up Process SP    */
        "    movs r1,%0      \n"
        "    msr  control,r1 \n" /* Switch to Process SP */
        "    isb             \n" /* Flush the pipeline   */
        :: "i" (CONTROL_SPSEL) : "r1" );

    /* Set up Main SP for IRQ/Exception context. */
    write_special(msp, _irq_stacktop);

    /* Initialise interrupts and exceptions. */
    scb->vtor = (uint32_t)(unsigned long)vector_table;
    scb->ccr |= SCB_CCR_STKALIGN | SCB_CCR_DIV_0_TRP;
    /* GCC inlines memcpy() using full-word load/store regardless of buffer
     * alignment. Hence it is unsafe to trap on unaligned accesses. */
    /*scb->ccr |= SCB_CCR_UNALIGN_TRP;*/
    scb->shcsr |= (SCB_SHCSR_USGFAULTENA |
                   SCB_SHCSR_BUSFAULTENA |
                   SCB_SHCSR_MEMFAULTENA);

    /* SVCall/PendSV exceptions have lowest priority. */
    scb->shpr2 = 0xff<<24;
    scb->shpr3 = 0xff<<16;
}

static void identify_mcu(void)
{
    /* DBGMCU_IDCODE (E0042000): 
     *  STM32F105RB:  10016418 (device id: 418) 
     *  AT32F415CBT7: 700301c5 (device id: 1c5)
     *  AT32F415RCT7: 70030240 (device id: 240) 
     * However the AT32 IDCODE values are undocumented so we cannot rely 
     * on them (for example, what will be the ID for chips with differing 
     * amounts of Flash, or numbers of pins?) */

    /* We detect an Artery MCU by presence of Cortex-M4 CPUID. 
     * Cortex-M4: 41xfc24x ; Cortex-M3: 41xfc23x */
    is_artery_mcu = ((scb->cpuid >> 4) & 0xf) == 4;

    if (is_artery_mcu) {
        unsigned int flash_kb = *(uint16_t *)0x1ffff7e0;
        ram_kb = 32;
        if (flash_kb == 128)
            flash_page_size = 1024;
    }
}

static void clock_init(void)
{
    flash->divr = FLASH_DIVR_DIV_3;

    /* Start up the external oscillator. */
    rcc->cr |= RCC_CR_HSEON;
    while (!(rcc->cr & RCC_CR_HSERDY))
        cpu_relax();

    /* Configure PLL for 8MHz input, 288MHz output. */
    rcc->pllcfgr = (RCC_PLLCFGR_PLLSRC_HSE | /* PLLSrc = HSE = 8MHz */
                    RCC_PLLCFGR_PLL_MS(1) |  /* PLL In = HSE/1 = 8MHz */
                    RCC_PLLCFGR_PLL_NS(72) | /* PLLVCO = 8MHz*72 = 576MHz */
                    RCC_PLLCFGR_PLL_FR(PLL_FR_2)); /* PLL Out = 576MHz/2 */

    /* Bus divisors. */
    rcc->cfgr = (RCC_CFGR_PPRE2(4) | /* APB2 = 288MHz/2 = 144MHz  */
                 RCC_CFGR_PPRE1(4) | /* APB1 = 288MHz/2 = 144MHz */
                 RCC_CFGR_HPRE(0));  /* AHB  = 288MHz/1 = 288MHz */

    /* Enable and stabilise the PLL. */
    rcc->cr |= RCC_CR_PLLON;
    while (!(rcc->cr & RCC_CR_PLLRDY))
        cpu_relax();

    /* Switch to the externally-driven PLL for system clock. */
    rcc->cfgr |= RCC_CFGR_SW(2);
    while ((rcc->cfgr & RCC_CFGR_SWS(3)) != RCC_CFGR_SWS(2))
        cpu_relax();

    /* Internal oscillator no longer needed. */
    rcc->cr &= ~RCC_CR_HSION;

    /* Enable SysTick counter at 72/8=9MHz. */
    stk->load = STK_MASK;
    stk->ctrl = STK_CTRL_ENABLE;
}

static void peripheral_init(void)
{
    /* Enable basic GPIO clocks, DTCM RAM, DMA, and EXTICR. */
    rcc->ahb1enr |= (RCC_AHB1ENR_DMA2EN |
                     RCC_AHB1ENR_DMA1EN |
                     RCC_AHB1ENR_GPIOCEN |
                     RCC_AHB1ENR_GPIOBEN | 
                     RCC_AHB1ENR_GPIOAEN);
    rcc->apb2enr |= (RCC_APB2ENR_SYSCFGEN);

    /* Release JTAG pins. */
    gpio_configure_pin(gpioa, 15, GPI_floating);
    gpio_configure_pin(gpiob,  3, GPI_floating);
    gpio_configure_pin(gpiob,  4, GPI_floating);
}

void stm32_init(void)
{
    exception_init();
    identify_mcu();
    clock_init();
    peripheral_init();
    cpu_sync();
}

void delay_ticks(unsigned int ticks)
{
    unsigned int diff, cur, prev = stk->val;

    for (;;) {
        cur = stk->val;
        diff = (prev - cur) & STK_MASK;
        if (ticks <= diff)
            break;
        ticks -= diff;
        prev = cur;
    }
}

void delay_ns(unsigned int ns)
{
    delay_ticks((ns * STK_MHZ) / 1000u);
}

void delay_us(unsigned int us)
{
    delay_ticks(us * STK_MHZ);
}

void delay_ms(unsigned int ms)
{
    delay_ticks(ms * 1000u * STK_MHZ);
}

void gpio_configure_pin(GPIO gpio, unsigned int pin, unsigned int mode)
{
    gpio_write_pin(gpio, pin, mode >> 7);
    gpio->moder = (gpio->moder & ~(3<<(pin<<1))) | ((mode&3)<<(pin<<1));
    mode >>= 2;
    gpio->otyper = (gpio->otyper & ~(1<<pin)) | ((mode&1)<<pin);
    mode >>= 1;
    gpio->odrvr = (gpio->odrvr & ~(3<<(pin<<1))) | ((mode&3)<<(pin<<1));
    mode >>= 2;
    gpio->pupdr = (gpio->pupdr & ~(3<<(pin<<1))) | ((mode&3)<<(pin<<1));
}

void _exti_route(unsigned int px, unsigned int pin)
{
    unsigned int n = pin >> 2;
    unsigned int s = (pin & 3) << 2;
    uint32_t exticr = syscfg->exticr[n];
    ASSERT(!in_exception()); /* no races please */
    exticr &= ~(0xf << s);
    exticr |= px << s;
    syscfg->exticr[n] = exticr;
}

void system_reset(void)
{
    console_sync();
    printk("Resetting...\n");
    scb->aircr = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    for (;;) ;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
