/*
 * stm32f10x_regs.h
 * 
 * Core and peripheral register definitions.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* SysTick timer */
struct stk {
    uint32_t ctrl;     /* 00: Control and status */
    uint32_t load;     /* 04: Reload value */
    uint32_t val;      /* 08: Current value */
    uint32_t calib;    /* 0C: Calibration value */
};

#define STK_CTRL_COUNTFLAG  (1u<<16)
#define STK_CTRL_CLKSOURCE  (1u<< 2)
#define STK_CTRL_TICKINT    (1u<< 1)
#define STK_CTRL_ENABLE     (1u<< 0)

#define STK_MASK            ((1u<<24)-1)

#define STK_BASE 0xe000e010

/* System control block */
struct scb {
    uint32_t cpuid;    /* 00: CPUID base */
    uint32_t icsr;     /* 04: Interrupt control and state */
    uint32_t vtor;     /* 08: Vector table offset */
    uint32_t aircr;    /* 0C: Application interrupt and reset control */
    uint32_t scr;      /* 10: System control */
    uint32_t ccr;      /* 14: Configuration and control */
    uint32_t shpr1;    /* 18: System handler priority reg #1 */
    uint32_t shpr2;    /* 1C: system handler priority reg #2 */
    uint32_t shpr3;    /* 20: System handler priority reg #3 */
    uint32_t shcsr;    /* 24: System handler control and state */
    uint32_t cfsr;     /* 28: Configurable fault status */
    uint32_t hfsr;     /* 2C: Hard fault status */
    uint32_t _unused;  /* 30: - */
    uint32_t mmar;     /* 34: Memory management fault address */
    uint32_t bfar;     /* 38: Bus fault address */
};

#define SCB_CCR_STKALIGN       (1u<<9)
#define SCB_CCR_BFHFNMIGN      (1u<<8)
#define SCB_CCR_DIV_0_TRP      (1u<<4)
#define SCB_CCR_UNALIGN_TRP    (1u<<3)
#define SCB_CCR_USERSETMPEND   (1u<<1)
#define SCB_CCR_NONBASETHRDENA (1u<<0)

#define SCB_SHCSR_USGFAULTENA    (1u<<18)
#define SCB_SHCSR_BUSFAULTENA    (1u<<17)
#define SCB_SHCSR_MEMFAULTENA    (1u<<16)
#define SCB_SHCSR_SVCALLPENDED   (1u<<15)
#define SCB_SHCSR_BUSFAULTPENDED (1u<<14)
#define SCB_SHCSR_MEMFAULTPENDED (1u<<13)
#define SCB_SHCSR_USGFAULTPENDED (1u<<12)
#define SCB_SHCSR_SYSTICKACT     (1u<<11)
#define SCB_SHCSR_PENDSVACT      (1u<<10)
#define SCB_SHCSR_MONITORACT     (1u<< 8)
#define SCB_SHCSR_SVCALLACT      (1u<< 7)
#define SCB_SHCSR_USGFAULTACT    (1u<< 3)
#define SCB_SHCSR_BUSFAULTACT    (1u<< 1)
#define SCB_SHCSR_MEMFAULTACT    (1u<< 0)

#define SCB_CFSR_DIVBYZERO     (1u<<25)
#define SCB_CFSR_UNALIGNED     (1u<<24)
#define SCB_CFSR_NOCP          (1u<<19)
#define SCB_CFSR_INVPC         (1u<<18)
#define SCB_CFSR_INVSTATE      (1u<<17)
#define SCB_CFSR_UNDEFINSTR    (1u<<16)
#define SCB_CFSR_BFARVALID     (1u<<15)
#define SCB_CFSR_STKERR        (1u<<12)
#define SCB_CFSR_UNSTKERR      (1u<<11)
#define SCB_CFSR_IMPRECISERR   (1u<<10)
#define SCB_CFSR_PRECISERR     (1u<< 9)
#define SCB_CFSR_IBUSERR       (1u<< 8)
#define SCB_CFSR_MMARVALID     (1u<< 7)
#define SCB_CFSR_MSTKERR       (1u<< 4)
#define SCB_CFSR_MUNSTKERR     (1u<< 3)
#define SCB_CFSR_DACCVIOL      (1u<< 1)
#define SCB_CFSR_IACCVIOL      (1u<< 0)

#define SCB_AIRCR_VECTKEY     (0x05fau<<16)
#define SCB_AIRCR_SYSRESETREQ (1u<<2)

#define SCB_BASE 0xe000ed00

/* Nested vectored interrupt controller */
struct nvic {
    uint32_t iser[32]; /*  00: Interrupt set-enable */
    uint32_t icer[32]; /*  80: Interrupt clear-enable */
    uint32_t ispr[32]; /* 100: Interrupt set-pending */
    uint32_t icpr[32]; /* 180: Interrupt clear-pending */
    uint32_t iabr[64]; /* 200: Interrupt active */
    uint8_t ipr[80];   /* 300: Interrupt priority */
};

#define NVIC_BASE 0xe000e100

struct dbg {
    uint32_t mcu_idcode; /* 00: MCU ID code */
    uint32_t mcu_cr;     /* 04: Debug MCU configuration */
};

#define DBG_BASE 0xe0042000

/* Flash memory interface */
struct flash_bank {
    uint32_t sr;       /* +00: Flash status */
    uint32_t cr;       /* +04: Flash control */
    uint32_t ar;       /* +08: Flash address */
};
struct flash {
    uint32_t psr;      /* 00: Performance select */
    uint32_t unlock1;  /* 04: Bank 1 unlock */
    uint32_t opt_unlock;/* 08: Option bytes unlock */
    struct flash_bank bank1;
    uint32_t rsvd;     /* 18: - */
    uint32_t obr;      /* 1C: Option byte */
    uint32_t epps0;    /* 20: Erase/program protection */
    uint32_t rsvd2[2]; /* 24-28: - */
    uint32_t epps1;    /* 2C: Erase/program protection */
    uint32_t rsvd3[5]; /* 30-40: - */
    uint32_t unlock2;  /* 44: Bank 2 unlock */
    uint32_t rsvd4;    /* 48: - */
    struct flash_bank bank2;
    uint32_t contr;    /* 58: Continue read */
    uint32_t divr;     /* 5C: Divider */
};

#define FLASH_SR_EOP         (1u<< 5)
#define FLASH_SR_WRPRTERR    (1u<< 4)
#define FLASH_SR_PGERR       (1u<< 2)
#define FLASH_SR_BSY         (1u<< 0)

#define FLASH_CR_EOPIE       (1u<<12)
#define FLASH_CR_ERRIE       (1u<<10)
#define FLASH_CR_OPTWRE      (1u<< 9)
#define FLASH_CR_LOCK        (1u<< 7)
#define FLASH_CR_ERASE_STRT  (1u<< 6)
#define FLASH_CR_OPTER       (1u<< 5)
#define FLASH_CR_OPTPG       (1u<< 4)
#define FLASH_CR_BLK_ER      (1u<< 3)
#define FLASH_CR_BANK_ER     (1u<< 2)
#define FLASH_CR_SEC_ER      (1u<< 1)
#define FLASH_CR_PG          (1u<< 0)

#define FLASH_DIVR_DIV_2     (0u<< 0)
#define FLASH_DIVR_DIV_3     (1u<< 0)
#define FLASH_DIVR_DIV_4     (2u<< 0)

#define FLASH_BASE 0x40023c00

/* Power control */
struct pwr {
    uint32_t cr;       /* 00: Power control */
    uint32_t csr;      /* 04: Power control/status */
};

#define PWR_CR_DBP           (1u<< 8)

#define PWR_BASE 0x40007000

/* Backup */
struct bkp {
    uint32_t _0[1];    /* 00: - */
    uint32_t dr1[10];  /* 04-28: Data block #1 */
    uint32_t rtccr;    /* 2C: RTC clock calibration */
    uint32_t cr;       /* 30: Control */
    uint32_t csr;      /* 34: Control/status */
    uint32_t _1[2];    /* 38-3C: - */
    uint32_t dr2[32];  /* 40-BC: Data block #2 */
};

#define BKP_BASE 0x40006c00

/* Reset and clock control */
struct rcc {
    uint32_t cr;       /* 00: Clock control */
    uint32_t pllcfgr;  /* 04: PLL configuration */
    uint32_t cfgr;     /* 08: Clock configuration */
    uint32_t cir;      /* 0C: Clock interrupt */
    uint32_t ahb1rstr; /* 10: AHB1 peripheral reset */
    uint32_t ahb2rstr; /* 14: AHB2 peripheral reset */
    uint32_t ahb3rstr; /* 18: AHB3 peripheral reset */
    uint32_t _unused0; /* 1C: - */
    uint32_t apb1rstr; /* 20: APB1 peripheral reset */
    uint32_t apb2rstr; /* 24: APB2 peripheral reset */
    uint32_t _unused1; /* 28: - */
    uint32_t _unused2; /* 2C: - */
    uint32_t ahb1enr;  /* 30: AHB1 peripheral clock enable */
    uint32_t ahb2enr;  /* 34: AHB2 peripheral clock enable */
    uint32_t ahb3enr;  /* 38: AHB3 peripheral clock enable */
    uint32_t _unused3; /* 3C: - */
    uint32_t apb1enr;  /* 40: APB1 peripheral clock enable */
    uint32_t apb2enr;  /* 44: APB2 peripheral clock enable */
    uint32_t _unused4; /* 48: - */
    uint32_t _unused5; /* 4C: - */
    uint32_t ahb1lpenr;/* 50: AHB1 peripheral clock enable (low-power mode) */
    uint32_t ahb2lpenr;/* 54: AHB2 peripheral clock enable (low-power mode) */
    uint32_t ahb3lpenr;/* 58: AHB3 peripheral clock enable (low-power mode) */
    uint32_t _unused6; /* 5C: - */
    uint32_t apb1lpenr;/* 60: APB1 peripheral clock enable (low-power mode) */
    uint32_t apb2lpenr;/* 64: APB2 peripheral clock enable (low-power mode) */
    uint32_t _unused7; /* 68: - */
    uint32_t _unused8; /* 6C: - */
    uint32_t bdcr;     /* 70: Backup domain control */
    uint32_t csr;      /* 74: Clock control & status */
    uint32_t _unused9[10]; /* 78-9C: - */
    uint32_t misc1;    /* A0: Misc 1 */
    uint32_t misc2;    /* A4: Misc 2 */
};

#define RCC_CR_PLLRDY        (1u<<25)
#define RCC_CR_PLLON         (1u<<24)
#define RCC_CR_CFDEN         (1u<<19)
#define RCC_CR_HSEBYP        (1u<<18)
#define RCC_CR_HSERDY        (1u<<17)
#define RCC_CR_HSEON         (1u<<16)
#define RCC_CR_HSIRDY        (1u<<1)
#define RCC_CR_HSION         (1u<<0)

#define RCC_PLLCFGR_PLLSRC_HSE (1<<22)
#define PLL_FR_2 1
#define RCC_PLLCFGR_PLL_FR(x) ((x)<<16)
#define RCC_PLLCFGR_PLL_NS(x) ((x)<< 6)
#define RCC_PLLCFGR_PLL_MS(x) ((x)<< 0)

#define RCC_CFGR_MCO2(x)     ((x)<<30)
#define RCC_CFGR_MCO2PRE(x)  ((x)<<27)
#define RCC_CFGR_USBPRE(x)   ((x)<<24)
#define RCC_CFGR_MCO1(x)     ((x)<<21)
#define RCC_CFGR_RTCPRE(x)   ((x)<<16)
#define RCC_CFGR_PPRE2(x)    ((x)<<13)
#define RCC_CFGR_PPRE1(x)    ((x)<<10)
#define RCC_CFGR_HPRE(x)     ((x)<< 4)
#define RCC_CFGR_SWS(x)      ((x)<< 2)
#define RCC_CFGR_SW(x)       ((x)<< 0)

#define RCC_AHB1ENR_OTGFS2EN (1u<<29)
#define RCC_AHB1ENR_DMA2EN   (1u<<24)
#define RCC_AHB1ENR_DMA1EN   (1u<<22)
#define RCC_AHB1ENR_EDMAEN   (1u<<21)
#define RCC_AHB1ENR_CRCEN    (1u<<12)
#define RCC_AHB1ENR_GPIOHEN  (1u<< 7)
#define RCC_AHB1ENR_GPIOGEN  (1u<< 6)
#define RCC_AHB1ENR_GPIOFEN  (1u<< 5)
#define RCC_AHB1ENR_GPIOEEN  (1u<< 4)
#define RCC_AHB1ENR_GPIODEN  (1u<< 3)
#define RCC_AHB1ENR_GPIOCEN  (1u<< 2)
#define RCC_AHB1ENR_GPIOBEN  (1u<< 1)
#define RCC_AHB1ENR_GPIOAEN  (1u<< 0)

#define RCC_AHB2ENR_OTGFS1EN (1u<< 7)
#define RCC_AHB2ENR_DVPEN    (1u<< 0)

#define RCC_AHB3ENR_QSPI2EN  (1u<<14)
#define RCC_AHB3ENR_QSPI1EN  (1u<< 1)
#define RCC_AHB3ENR_XMCEN    (1u<< 0)

#define RCC_APB1ENR_USART8EN (1u<<31)
#define RCC_APB1ENR_USART7EN (1u<<30)
#define RCC_APB1ENR_DACEN    (1u<<29)
#define RCC_APB1ENR_PWREN    (1u<<28)
#define RCC_APB1ENR_CAN1EN   (1u<<25)
#define RCC_APB1ENR_I2C3EN   (1u<<23)
#define RCC_APB1ENR_I2C2EN   (1u<<22)
#define RCC_APB1ENR_I2C1EN   (1u<<21)
#define RCC_APB1ENR_USART5EN (1u<<20)
#define RCC_APB1ENR_USART4EN (1u<<19)
#define RCC_APB1ENR_USART3EN (1u<<18)
#define RCC_APB1ENR_USART2EN (1u<<17)
#define RCC_APB1ENR_SPI3EN   (1u<<15)
#define RCC_APB1ENR_SPI2EN   (1u<<14)
#define RCC_APB1ENR_WWDGEN   (1u<<11)
#define RCC_APB1ENR_TIM14EN  (1u<< 8)
#define RCC_APB1ENR_TIM13EN  (1u<< 7)
#define RCC_APB1ENR_TIM12EN  (1u<< 6)
#define RCC_APB1ENR_TIM7EN   (1u<< 5)
#define RCC_APB1ENR_TIM6EN   (1u<< 4)
#define RCC_APB1ENR_TIM5EN   (1u<< 3)
#define RCC_APB1ENR_TIM4EN   (1u<< 2)
#define RCC_APB1ENR_TIM3EN   (1u<< 1)
#define RCC_APB1ENR_TIM2EN   (1u<< 0)

#define RCC_APB2ENR_ACCEN    (1u<<29)
#define RCC_APB2ENR_TIM20EN  (1u<<20)
#define RCC_APB2ENR_TIM11EN  (1u<<18)
#define RCC_APB2ENR_TIM10EN  (1u<<17)
#define RCC_APB2ENR_TIM9EN   (1u<<16)
#define RCC_APB2ENR_SYSCFGEN (1u<<14)
#define RCC_APB2ENR_SPI4EN   (1u<<13)
#define RCC_APB2ENR_SPI1EN   (1u<<12)
#define RCC_APB2ENR_ADC3EN   (1u<<10)
#define RCC_APB2ENR_ADC2EN   (1u<< 9)
#define RCC_APB2ENR_ADC1EN   (1u<< 8)
#define RCC_APB2ENR_USART6EN (1u<< 5)
#define RCC_APB2ENR_USART1EN (1u<< 4)
#define RCC_APB2ENR_TIM8EN   (1u<< 1)
#define RCC_APB2ENR_TIM1EN   (1u<< 0)

#define RCC_BDCR_BDRST       (1u<<16)
#define RCC_BDCR_RTCEN       (1u<<15)
#define RCC_BDCR_RTCSEL(x)   ((x)<<8)
#define RCC_BDCR_LSEDRV(x)   ((x)<<3)
#define RCC_BDCR_LSEBYP      (1u<< 2)
#define RCC_BDCR_LSERDY      (1u<< 1)
#define RCC_BDCR_LSEON       (1u<< 0)

#define RCC_CSR_LPWRRSTF     (1u<<31)
#define RCC_CSR_WWDGRSTF     (1u<<30)
#define RCC_CSR_IWDGRSTF     (1u<<29)
#define RCC_CSR_SFTRSTF      (1u<<28)
#define RCC_CSR_PORRSTF      (1u<<27)
#define RCC_CSR_PINRSTF      (1u<<26)
#define RCC_CSR_BORRSTF      (1u<<25)
#define RCC_CSR_RMVF         (1u<<24)
#define RCC_CSR_LSIRDY       (1u<< 1)
#define RCC_CSR_LSION        (1u<< 0)

#define RCC_BASE 0x40023800

/* General-purpose I/O */
struct gpio {
    uint32_t moder;   /* 00: Port mode */
    uint32_t otyper;  /* 04: Port output type */
    uint32_t odrvr;   /* 08: Drive capability */
    uint32_t pupdr;   /* 0C: Port pull-up/pull-down */
    uint32_t idr;     /* 10: Port input data */
    uint32_t odr;     /* 14: Port output data */
    uint32_t bsrr;    /* 18: Port bit set/reset */
    uint32_t lckr;    /* 1C: Port configuration lock */
    uint32_t afrl;    /* 20: Alternate function low */
    uint32_t afrh;    /* 24: Alternate function high */
    uint32_t clr;     /* 28: Port bit clear */
    uint32_t rsvd[4]; /* 2C-38 */
    uint32_t hdrv;    /* 3C: Huge current control */
};

/* 0-1: MODE, 2: OTYPE, 3-4:ODRV, 5-6:PUPD, 7:OUTPUT_LEVEL */
#define GPI_analog    0x3u
#define GPI(pupd)     (0x0u|((pupd)<<5))
#define PUPD_none     0
#define PUPD_up       1
#define PUPD_down     2
#define GPI_floating  GPI(PUPD_none)
#define GPI_pull_down GPI(PUPD_down)
#define GPI_pull_up   GPI(PUPD_up)

#define GPO_pushpull(speed,level)  (0x1u|((speed)<<3)|((level)<<7))
#define GPO_opendrain(speed,level) (0x5u|((speed)<<3)|((level)<<7))
#define AFI(pupd)                  (0x2u|((pupd)<<5))
#define AFO_pushpull(speed)        (0x2u|((speed)<<3))
#define AFO_opendrain(speed)       (0x6u|((speed)<<3))
#define _2MHz 0
#define _10MHz 0
#define _50MHz 0
#define LOW  0
#define HIGH 1

#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define GPIOC_BASE 0x40020800
#define GPIOD_BASE 0x40020C00
#define GPIOE_BASE 0x40021000
#define GPIOF_BASE 0x40021400
#define GPIOG_BASE 0x40021800
#define GPIOH_BASE 0x40021C00

/* System configuration controller */
struct syscfg {
    uint32_t cfg1;       /* 00: Configuration 1 */
    uint32_t cfg2;       /* 04: Configuration 2 */
    uint32_t exticr[4];  /* 08-14: External interrupt configuration #1-4 */
    uint32_t _pad[5];
    uint32_t uhdrv;      /* 2C: Ultra high source/sink strength */
};

#define SYSCFG_BASE 0x40013800

struct exti {
    uint32_t imr;        /* 00: Interrupt mask */
    uint32_t emr;        /* 04: Event mask */
    uint32_t rtsr;       /* 08: Rising trigger selection */
    uint32_t ftsr;       /* 0C: Falling trigger selection */
    uint32_t swier;      /* 10: Software interrupt event */
    uint32_t pr;         /* 14: Pending */
};

#define EXTI_BASE 0x40013c00

/* DMA */
struct dma_chn {
    uint32_t ccr;        /* +00: Configuration */
    uint32_t cndtr;      /* +04: Number of data */
    uint32_t cpar;       /* +08: Peripheral address */
    uint32_t cmar;       /* +0C: Memory address */
    uint32_t rsvd;       /* +10: - */
};
struct dma {
    uint32_t isr;        /* 00: Interrupt status */
    uint32_t ifcr;       /* 04: Interrupt flag clear */
    struct dma_chn ch[7];
};

/* n=1..7 */
#define DMA_ISR_TEIF(n)      (8u<<(((n)-1)*4))
#define DMA_ISR_HTIF(n)      (4u<<(((n)-1)*4))
#define DMA_ISR_TCIF(n)      (2u<<(((n)-1)*4))
#define DMA_ISR_GIF(n)       (1u<<(((n)-1)*4))

/* n=1..7 */
#define DMA_IFCR_CTEIF(n)    (8u<<(((n)-1)*4))
#define DMA_IFCR_CHTIF(n)    (4u<<(((n)-1)*4))
#define DMA_IFCR_CTCIF(n)    (2u<<(((n)-1)*4))
#define DMA_IFCR_CGIF(n)     (1u<<(((n)-1)*4))

#define DMA_CCR_MEM2MEM      (1u<<14)
#define DMA_CCR_PL_LOW       (0u<<12)
#define DMA_CCR_PL_MEDIUM    (1u<<12)
#define DMA_CCR_PL_HIGH      (2u<<12)
#define DMA_CCR_PL_V_HIGH    (3u<<12)
#define DMA_CCR_MSIZE_8BIT   (0u<<10)
#define DMA_CCR_MSIZE_16BIT  (1u<<10)
#define DMA_CCR_MSIZE_32BIT  (2u<<10)
#define DMA_CCR_PSIZE_8BIT   (0u<< 8)
#define DMA_CCR_PSIZE_16BIT  (1u<< 8)
#define DMA_CCR_PSIZE_32BIT  (2u<< 8)
#define DMA_CCR_MINC         (1u<< 7)
#define DMA_CCR_PINC         (1u<< 6)
#define DMA_CCR_CIRC         (1u<< 5)
#define DMA_CCR_DIR_P2M      (0u<< 4)
#define DMA_CCR_DIR_M2P      (1u<< 4)
#define DMA_CCR_TEIE         (1u<< 3)
#define DMA_CCR_HTIE         (1u<< 2)
#define DMA_CCR_TCIE         (1u<< 1)
#define DMA_CCR_EN           (1u<< 0)

#define DMA1_BASE 0x40020000
#define DMA2_BASE 0x40020400

/* Timer */
struct tim {
    uint32_t cr1;   /* 00: Control 1 */
    uint32_t cr2;   /* 04: Control 2 */
    uint32_t smcr;  /* 08: Slave mode control */
    uint32_t dier;  /* 0C: DMA/interrupt enable */
    uint32_t sr;    /* 10: Status */
    uint32_t egr;   /* 14: Event generation */
    uint32_t ccmr1; /* 18: Capture/compare mode 1 */
    uint32_t ccmr2; /* 1C: Capture/compare mode 2 */
    uint32_t ccer;  /* 20: Capture/compare enable */
    uint32_t cnt;   /* 24: Counter */
    uint32_t psc;   /* 28: Prescaler */
    uint32_t arr;   /* 2C: Auto-reload */
    uint32_t rcr;   /* 30: Repetition counter */
    uint32_t ccr1;  /* 34: Capture/compare 1 */
    uint32_t ccr2;  /* 38: Capture/compare 2 */
    uint32_t ccr3;  /* 3C: Capture/compare 3 */
    uint32_t ccr4;  /* 40: Capture/compare 4 */
    uint32_t bdtr;  /* 44: Break and dead-time */
    uint32_t dcr;   /* 48: DMA control */
    uint32_t dmar;  /* 4C: DMA address for full transfer */
};

#define TIM_CR1_ARPE         (1u<<7)
#define TIM_CR1_DIR          (1u<<4)
#define TIM_CR1_OPM          (1u<<3)
#define TIM_CR1_URS          (1u<<2)
#define TIM_CR1_UDIS         (1u<<1)
#define TIM_CR1_CEN          (1u<<0)

#define TIM_CR2_TI1S         (1u<<7)
#define TIM_CR2_CCDS         (1u<<3)

#define TIM_DIER_TDE         (1u<<14)
#define TIM_DIER_CC4DE       (1u<<12)
#define TIM_DIER_CC3DE       (1u<<11)
#define TIM_DIER_CC2DE       (1u<<10)
#define TIM_DIER_CC1DE       (1u<<9)
#define TIM_DIER_UDE         (1u<<8)
#define TIM_DIER_TIE         (1u<<6)
#define TIM_DIER_CC4IE       (1u<<4)
#define TIM_DIER_CC3IE       (1u<<3)
#define TIM_DIER_CC2IE       (1u<<2)
#define TIM_DIER_CC1IE       (1u<<1)
#define TIM_DIER_UIE         (1u<<0)

#define TIM_SR_CC4OF         (1u<<12)
#define TIM_SR_CC3OF         (1u<<11)
#define TIM_SR_CC2OF         (1u<<10)
#define TIM_SR_CC1OF         (1u<<9)
#define TIM_SR_TIF           (1u<<6)
#define TIM_SR_CC4IF         (1u<<4)
#define TIM_SR_CC3IF         (1u<<3)
#define TIM_SR_CC2IF         (1u<<2)
#define TIM_SR_CC1IF         (1u<<1)
#define TIM_SR_UIF           (1u<<0)

#define TIM_EGR_TG           (1u<<6)
#define TIM_EGR_CC4G         (1u<<4)
#define TIM_EGR_CC3G         (1u<<3)
#define TIM_EGR_CC2G         (1u<<2)
#define TIM_EGR_CC1G         (1u<<1)
#define TIM_EGR_UG           (1u<<0)

#define TIM_CCMR1_OC2CE      (1u <<15)
#define TIM_CCMR1_OC2M(x)    ((x)<<12)
#define TIM_CCMR1_OC2PE      (1u <<11)
#define TIM_CCMR1_OC2FE      (1u <<10)
#define TIM_CCMR1_CC2S(x)    ((x)<< 8)
#define TIM_CCMR1_OC1CE      (1u << 7)
#define TIM_CCMR1_OC1M(x)    ((x)<< 4)
#define TIM_CCMR1_OC1PE      (1u << 3)
#define TIM_CCMR1_OC1FE      (1u << 2)
#define TIM_CCMR1_CC1S(x)    ((x)<< 0)

#define TIM_CCMR1_IC2F(x)    ((x)<<12)
#define TIM_CCMR1_IC2PSC(x)  ((x)<<10)
#define TIM_CCMR1_IC1F(x)    ((x)<< 4)
#define TIM_CCMR1_IC1PSC(x)  ((x)<< 2)

#define TIM_CCMR2_OC4CE      (1u <<15)
#define TIM_CCMR2_OC4M(x)    ((x)<<12)
#define TIM_CCMR2_OC4PE      (1u <<11)
#define TIM_CCMR2_OC4FE      (1u <<10)
#define TIM_CCMR2_CC4S(x)    ((x)<< 8)
#define TIM_CCMR2_OC3CE      (1u << 7)
#define TIM_CCMR2_OC3M(x)    ((x)<< 4)
#define TIM_CCMR2_OC3PE      (1u << 3)
#define TIM_CCMR2_OC3FE      (1u << 2)
#define TIM_CCMR2_CC3S(x)    ((x)<< 0)

#define TIM_CCMR2_IC4F(x)    ((x)<<12)
#define TIM_CCMR2_IC4PSC(x)  ((x)<<10)
#define TIM_CCMR2_IC3F(x)    ((x)<< 4)
#define TIM_CCMR2_IC3PSC(x)  ((x)<< 2)

#define TIM_OCM_FROZEN       (0u)
#define TIM_OCM_SET_HIGH     (1u)
#define TIM_OCM_SET_LOW      (2u)
#define TIM_OCM_TOGGLE       (3u)
#define TIM_OCM_FORCE_LOW    (4u)
#define TIM_OCM_FORCE_HIGH   (5u)
#define TIM_OCM_PWM1         (6u)
#define TIM_OCM_PWM2         (7u)
#define TIM_OCM_MASK         (7u)

#define TIM_CCS_OUTPUT       (0u)
#define TIM_CCS_INPUT_TI1    (1u)
#define TIM_CCS_INPUT_TI2    (2u)
#define TIM_CCS_INPUT_TRC    (3u)
#define TIM_CCS_MASK         (3u)

#define TIM_CCER_CC4P        (1u<<13)
#define TIM_CCER_CC4E        (1u<<12)
#define TIM_CCER_CC3P        (1u<< 9)
#define TIM_CCER_CC3E        (1u<< 8)
#define TIM_CCER_CC2P        (1u<< 5)
#define TIM_CCER_CC2E        (1u<< 4)
#define TIM_CCER_CC1P        (1u<< 1)
#define TIM_CCER_CC1E        (1u<< 0)

#define TIM_BDTR_MOE         (1u<<15)
#define TIM_BDTR_AOE         (1u<<14)
#define TIM_BDTR_BKP         (1u<<13)
#define TIM_BDTR_BKE         (1u<<12)
#define TIM_BDTR_OSSR        (1u<<11)
#define TIM_BDTR_OSSI        (1u<<10)
#define TIM_BDTR_LOCK(x)     ((x)<<8)
#define TIM_BDTR_DTG(x)      ((x)<<0)

#define TIM1_BASE 0x40012c00
#define TIM2_BASE 0x40000000
#define TIM3_BASE 0x40000400
#define TIM4_BASE 0x40000800
#define TIM5_BASE 0x40000c00
#define TIM6_BASE 0x40001000
#define TIM7_BASE 0x40001400

/* SPI/I2S */
struct spi {
    uint32_t cr1;     /* 00: Control 1 */
    uint32_t cr2;     /* 04: Control 2 */
    uint32_t sr;      /* 08: Status */
    uint32_t dr;      /* 0C: Data */
    uint32_t crcpr;   /* 10: CRC polynomial */
    uint32_t rxcrcr;  /* 14: RX CRC */
    uint32_t txcrcr;  /* 18: TX CRC */
    uint32_t i2scfgr; /* 1C: I2S configuration */
    uint32_t i2spr;   /* 20: I2S prescaler */
};

#define SPI_CR1_BIDIMODE  (1u<<15)
#define SPI_CR1_BIDIOE    (1u<<14)
#define SPI_CR1_CRCEN     (1u<<13)
#define SPI_CR1_CRCNEXT   (1u<<12)
#define SPI_CR1_DFF       (1u<<11)
#define SPI_CR1_RXONLY    (1u<<10)
#define SPI_CR1_SSM       (1u<< 9)
#define SPI_CR1_SSI       (1u<< 8)
#define SPI_CR1_LSBFIRST  (1u<< 7)
#define SPI_CR1_SPE       (1u<< 6)
#define SPI_CR1_BR_DIV2   (0u<< 3)
#define SPI_CR1_BR_DIV4   (1u<< 3)
#define SPI_CR1_BR_DIV8   (2u<< 3)
#define SPI_CR1_BR_DIV16  (3u<< 3)
#define SPI_CR1_BR_DIV32  (4u<< 3)
#define SPI_CR1_BR_DIV64  (5u<< 3)
#define SPI_CR1_BR_DIV128 (6u<< 3)
#define SPI_CR1_BR_DIV256 (7u<< 3)
#define SPI_CR1_BR_MASK   (7u<< 3)
#define SPI_CR1_MSTR      (1u<< 2)
#define SPI_CR1_CPOL      (1u<< 1)
#define SPI_CR1_CPHA      (1u<< 0)

#define SPI_CR2_TXEIE     (1u<< 7)
#define SPI_CR2_RXNEIE    (1u<< 6)
#define SPI_CR2_ERRIE     (1u<< 5)
#define SPI_CR2_SSOE      (1u<< 2)
#define SPI_CR2_TXDMAEN   (1u<< 1)
#define SPI_CR2_RXDMAEN   (1u<< 0)

#define SPI_SR_BSY        (1u<< 7)
#define SPI_SR_OVR        (1u<< 6)
#define SPI_SR_MODF       (1u<< 5)
#define SPI_SR_CRCERR     (1u<< 4)
#define SPI_SR_USR        (1u<< 3)
#define SPI_SR_CHSIDE     (1u<< 2)
#define SPI_SR_TXE        (1u<< 1)
#define SPI_SR_RXNE       (1u<< 0)

#define SPI1_BASE 0x40013000
#define SPI2_BASE 0x40003800
#define SPI3_BASE 0x40003C00

/* I2C */
struct i2c {
    uint32_t cr1;     /* 00: Control 1 */
    uint32_t cr2;     /* 04: Control 2 */
    uint32_t oar1;    /* 08: Own address 1 */
    uint32_t oar2;    /* 0C: Own address 2 */
    uint32_t dr;      /* 10: Data */
    uint32_t sr1;     /* 14: Status 1 */
    uint32_t sr2;     /* 18: Status 2 */
    uint32_t ccr;     /* 1C: Clock control */
    uint32_t trise;   /* 20: Rise time */
};

#define I2C_CR1_SWRST     (1u<<15)
#define I2C_CR1_ALERT     (1u<<13)
#define I2C_CR1_PEC       (1u<<12)
#define I2C_CR1_POS       (1u<<11)
#define I2C_CR1_ACK       (1u<<10)
#define I2C_CR1_STOP      (1u<< 9)
#define I2C_CR1_START     (1u<< 8)
#define I2C_CR1_NOSTRETCH (1u<< 7)
#define I2C_CR1_ENGC      (1u<< 6)
#define I2C_CR1_ENPEC     (1u<< 5)
#define I2C_CR1_ENARP     (1u<< 4)
#define I2C_CR1_SMBTYPE   (1u<< 3)
#define I2C_CR1_SMBUS     (1u<< 1)
#define I2C_CR1_PE        (1u<< 0)

#define I2C_CR2_LAST      (1u<<12)
#define I2C_CR2_DMAEN     (1u<<11)
#define I2C_CR2_ITBUFEN   (1u<<10)
#define I2C_CR2_ITEVTEN   (1u<< 9)
#define I2C_CR2_ITERREN   (1u<< 8)
#define I2C_CR2_FREQ(x)   (x)

#define I2C_SR1_SMBALERT  (1u<<15)
#define I2C_SR1_TIMEOUT   (1u<<14)
#define I2C_SR1_PECERR    (1u<<12)
#define I2C_SR1_OVR       (1u<<11)
#define I2C_SR1_AF        (1u<<10)
#define I2C_SR1_ARLO      (1u<< 9)
#define I2C_SR1_BERR      (1u<< 8)
#define I2C_SR1_ERRORS    0xdf00
#define I2C_SR1_TXE       (1u<< 7)
#define I2C_SR1_RXNE      (1u<< 6)
#define I2C_SR1_STOPF     (1u<< 4)
#define I2C_SR1_ADD10     (1u<< 3)
#define I2C_SR1_BTF       (1u<< 2)
#define I2C_SR1_ADDR      (1u<< 1)
#define I2C_SR1_SB        (1u<< 0)
#define I2C_SR1_EVENTS    0x001f

#define I2C_SR2_PEC(x)    ((x)<<15)
#define I2C_SR2_DUALF     (1u<< 7)
#define I2C_SR2_SMBHOST   (1u<< 6)
#define I2C_SR2_SMBDEFAULT (1u<< 5)
#define I2C_SR2_GENCALL   (1u<< 4)
#define I2C_SR2_TRA       (1u<< 2)
#define I2C_SR2_BUSY      (1u<< 1)
#define I2C_SR2_MSL       (1u<< 0)

#define I2C_CCR_FS        (1u<<15)
#define I2C_CCR_DUTY      (1u<<14)
#define I2C_CCR_CCR(x)    (x)

#define I2C1_BASE 0x40005400
#define I2C2_BASE 0x40005800

/* USART */
struct usart {
    uint32_t sr;   /* 00: Status */
    uint32_t dr;   /* 04: Data */
    uint32_t brr;  /* 08: Baud rate */
    uint32_t cr1;  /* 0C: Control 1 */
    uint32_t cr2;  /* 10: Control 2 */
    uint32_t cr3;  /* 14: Control 3 */
    uint32_t gtpr; /* 18: Guard time and prescaler */
};

#define USART_SR_CTS         (1u<<9)
#define USART_SR_LBD         (1u<<8)
#define USART_SR_TXE         (1u<<7)
#define USART_SR_TC          (1u<<6)
#define USART_SR_RXNE        (1u<<5)
#define USART_SR_IDLE        (1u<<4)
#define USART_SR_ORE         (1u<<3)
#define USART_SR_NE          (1u<<2)
#define USART_SR_FE          (1u<<1)
#define USART_SR_PE          (1u<<0)

#define USART_CR1_UE         (1u<<13)
#define USART_CR1_M          (1u<<12)
#define USART_CR1_WAKE       (1u<<11)
#define USART_CR1_PCE        (1u<<10)
#define USART_CR1_PS         (1u<< 9)
#define USART_CR1_PEIE       (1u<< 8)
#define USART_CR1_TXEIE      (1u<< 7)
#define USART_CR1_TCIE       (1u<< 6)
#define USART_CR1_RXNEIE     (1u<< 5)
#define USART_CR1_IDLEIE     (1u<< 4) 
#define USART_CR1_TE         (1u<< 3)
#define USART_CR1_RE         (1u<< 2)
#define USART_CR1_RWU        (1u<< 1)
#define USART_CR1_SBK        (1u<< 0)

#define USART_CR3_CTSIE      (1u<<10)
#define USART_CR3_CTSE       (1u<< 9)
#define USART_CR3_RTSE       (1u<< 8)
#define USART_CR3_DMAT       (1u<< 7)
#define USART_CR3_DMAR       (1u<< 6)
#define USART_CR3_SCEN       (1u<< 5)
#define USART_CR3_NACK       (1u<< 4)
#define USART_CR3_HDSEL      (1u<< 3)
#define USART_CR3_IRLP       (1u<< 2)
#define USART_CR3_IREN       (1u<< 1)
#define USART_CR3_EIE        (1u<< 0)

#define USART1_BASE 0x40013800
#define USART2_BASE 0x40004400
#define USART3_BASE 0x40004800

/* USB On-The-Go Full Speed interface */
struct usb_otg {
    uint32_t gotctl;   /* 00: Control and status */
    uint32_t gotgint;  /* 04: Interrupt */
    uint32_t gahbcfg;  /* 08: AHB configuration */
    uint32_t gusbcfg;  /* 0C: USB configuration */
    uint32_t grstctl;  /* 10: Reset */
    uint32_t gintsts;  /* 14: Core interrupt */
    uint32_t gintmsk;  /* 18: Interrupt mask */
    uint32_t grxstsr;  /* 1C: Receive status debug read */
    uint32_t grxstsp;  /* 20: Receive status debug pop */
    uint32_t grxfsiz;  /* 24: Receive FIFO size */
    union {
        uint32_t hnptxfsiz;  /* 28: Host non-periodic transmit FIFO size */
        uint32_t dieptxf0;   /* 28: Endpoint 0 transmit FIFO size */
    };
    uint32_t hnptxsts; /* 2C: Non-periodic transmit FIFO/queue status */
    uint32_t _0[2];
    uint32_t gccfg;    /* 38: General core configuration */
    uint32_t cid;      /* 3C: Core ID */
    uint32_t _1[48];
    uint32_t hptxfsiz; /* 100: Host periodic transmit FIFO size */
    uint32_t dieptxf1; /* 104: Device IN endpoint transmit FIFO #1 size */
    uint32_t dieptxf2; /* 108: Device IN endpoint transmit FIFO #2 size */
    uint32_t dieptxf3; /* 10C: Device IN endpoint transmit FIFO #3 size */
    uint32_t _2[188];
    uint32_t hcfg;     /* 400: Host configuration */
    uint32_t hfir;     /* 404: Host frame interval */
    uint32_t hfnum;    /* 408: Host frame number / frame time remaining */
    uint32_t _3[1];    /* 40C: */
    uint32_t hptxsts;  /* 410: Host periodic transmit FIFO / queue status */
    uint32_t haint;    /* 414: Host all channels interrupt status */
    uint32_t haintmsk; /* 418: Host all channels interrupt mask */
    uint32_t _4[9];
    uint32_t hprt;     /* 440: Host port control and status */
    uint32_t _5[47];
    struct {
        uint32_t charac; /* +00: Host channel-x characteristics */
        uint32_t _0[1];
        uint32_t intsts; /* +08: Host channel-x interrupt status */
        uint32_t intmsk; /* +0C: Host channel-x interrupt mask */
        uint32_t tsiz;   /* +10: Host channel x transfer size */
        uint32_t _1[3];
    } hc[8];           /* 500..5E0: */
    uint32_t _6[128];

    uint32_t dcfg;     /* 800: Device configuration */
    uint32_t dctl;     /* 804: Device control */
    uint32_t dsts;     /* 808: Device status */
    uint32_t _7[1];
    uint32_t diepmsk;  /* 810: Device IN endpoint common interrupt mask */
    uint32_t doepmsk;  /* 814: Device OUT endpoint common interrupt mask */
    uint32_t daint;    /* 818: Device all endpoints interrupt status */
    uint32_t daintmsk; /* 81C: Device all endpoints interrupt mask */
    uint32_t _8[2];
    uint32_t dvbusdis; /* 828: Device VBUS discharge time */
    uint32_t dvbuspulse; /* 82C: Device VBUS pulsing time */
    uint32_t _9[1];
    uint32_t diepempmsk; /* 834: Device IN endpoint FIFO empty int. mask */
    uint32_t _10[50];
    struct {
        uint32_t ctl;    /* +00: Device IN endpoint-x control */
        uint32_t _0[1];
        uint32_t intsts; /* +08: Device IN endpoint-x interrupt status */
        uint32_t _1[3];
        uint32_t txfsts; /* +18: Device IN endpoint-x transmit FIFO status */
        uint32_t _2[1];
    } diep[4];         /* 900..960: */
    uint32_t _11[96];
    struct {
        uint32_t ctl;    /* +00: Device OUT endpoint-x control */
        uint32_t _0[1];
        uint32_t intsts; /* +08: Device OUT endpoint-x interrupt status */
        uint32_t _1[1];
        uint32_t tsiz;   /* +10: Device OUT endpoint-x transmit FIFO status */
        uint32_t _2[3];
    } doep[4];         /* B00..B60: */
    uint32_t _12[160];

    uint32_t pcgcctl;  /* E00: Power and clock gating control */
};

#define OTG_GAHBCFG_PTXFELVL (1u<< 8)
#define OTG_GAHBCFG_TXFELVL  (1u<< 7)
#define OTG_GAHBCFG_GINTMSK  (1u<< 0)

#define OTG_GUSBCFG_CTXPKT   (1u<<31)
#define OTG_GUSBCFG_FDMOD    (1u<<30)
#define OTG_GUSBCFG_FHMOD    (1u<<29)
#define OTG_GUSBCFG_TRDT(x)  ((x)<<10)
#define OTG_GUSBCFG_HNPCAP   (1u<< 9)
#define OTG_GUSBCFG_SRPCAP   (1u<< 8)
#define OTG_GUSBCFG_PHYSEL   (1u<< 6)
#define OTG_GUSBCFG_TOCAL(x) ((x)<< 0)

/* GINTSTS and GINTMSK */
#define OTG_GINT_WKUPINT     (1u<<31) /* Host + Device */
#define OTG_GINT_SRQINT      (1u<<30) /* H + D */
#define OTG_GINT_DISCINT     (1u<<29) /* H */
#define OTG_GINT_CIDSCHG     (1u<<28) /* H + D */
#define OTG_GINT_PTXFE       (1u<<26) /* H */
#define OTG_GINT_HCINT       (1u<<25) /* H */
#define OTG_GINT_HPRTINT     (1u<<24) /* H */
#define OTG_GINT_IPXFR       (1u<<21) /* H */
#define OTG_GINT_IISOIXFR    (1u<<20) /* D */
#define OTG_GINT_OEPINT      (1u<<19) /* D */
#define OTG_GINT_IEPINT      (1u<<18) /* D */
#define OTG_GINT_EOPF        (1u<<15) /* D */
#define OTG_GINT_ISOODRP     (1u<<14) /* D */
#define OTG_GINT_ENUMDNE     (1u<<13) /* D */
#define OTG_GINT_USBRST      (1u<<12) /* D */
#define OTG_GINT_USBSUSP     (1u<<11) /* D */
#define OTG_GINT_ESUSP       (1u<<10) /* D */
#define OTG_GINT_GONAKEFF    (1u<< 7) /* D */
#define OTG_GINT_GINAKEFF    (1u<< 6) /* D */
#define OTG_GINT_NPTXFE      (1u<< 5) /* H */
#define OTG_GINT_RXFLVL      (1u<< 4) /* H + D */
#define OTG_GINT_SOF         (1u<< 3) /* H + D */
#define OTG_GINT_OTGINT      (1u<< 2) /* H + D */
#define OTG_GINT_MMIS        (1u<< 1) /* H + D */
#define OTG_GINT_CMOD        (1u<< 0) /* H + D */

#define OTG_RXSTS_PKTSTS_IN  (2u)
#define OTG_RXSTS_PKTSTS(r)  (((r)>>17)&0xf)
#define OTG_RXSTS_BCNT(r)    (((r)>>4)&0x7ff)
#define OTG_RXSTS_CHNUM(r)   ((r)&0xf)

#define OTG_GCCFG_SOFOUTEN   (1u<<20)
#define OTG_GCCFG_VBUSBSEN   (1u<<19)
#define OTG_GCCFG_VBUSASEN   (1u<<18)
#define OTG_GCCFG_PWRDWN     (1u<<16)

#define OTG_HCFG_FSLSS       (1u<<2)
#define OTG_HCFG_FSLSPCS     (3u<<0)
#define OTG_HCFG_FSLSPCS_48  (1u<<0)
#define OTG_HCFG_FSLSPCS_6   (2u<<0)

#define OTG_HPRT_PSPD_FULL   (1u<<17)
#define OTG_HPRT_PSPD_LOW    (2u<<17)
#define OTG_HPRT_PSPD_MASK   (1u<<17) /* read-only */
#define OTG_HPRT_PPWR        (1u<<12)
#define OTG_HPRT_PRST        (1u<< 8)
#define OTG_HPRT_PSUSP       (1u<< 7)
#define OTG_HPRT_PRES        (1u<< 6)
#define OTG_HPRT_POCCHNG     (1u<< 5) /* raises HPRTINT */
#define OTG_HPRT_POCA        (1u<< 4)
#define OTG_HPRT_PENCHNG     (1u<< 3) /* raises HPRTINT */
#define OTG_HPRT_PENA        (1u<< 2)
#define OTG_HPRT_PCDET       (1u<< 1) /* raises HPRTINT */
#define OTG_HPRT_PCSTS       (1u<< 0)
#define OTG_HPRT_INTS (OTG_HPRT_POCCHNG|OTG_HPRT_PENCHNG|OTG_HPRT_PCDET| \
                       OTG_HPRT_PENA) /* PENA is also set-to-clear  */

/* HCINTSTS and HCINTMSK */
#define OTG_HCINT_DTERR      (1u<<10)
#define OTG_HCINT_FRMOR      (1u<< 9)
#define OTG_HCINT_BBERR      (1u<< 8)
#define OTG_HCINT_TXERR      (1u<< 7)
#define OTG_HCINT_NYET       (1u<< 6) /* high-speed only; not STM32F10x */
#define OTG_HCINT_ACK        (1u<< 5)
#define OTG_HCINT_NAK        (1u<< 4)
#define OTG_HCINT_STALL      (1u<< 3)
#define OTG_HCINT_CHH        (1u<< 1)
#define OTG_HCINT_XFRC       (1u<< 0)

#define OTG_HCCHAR_CHENA     (1u<<31)
#define OTG_HCCHAR_CHDIS     (1u<<30)
#define OTG_HCCHAR_ODDFRM    (1u<<29)
#define OTG_HCCHAR_DAD(x)    ((x)<<22)
#define OTG_HCCHAR_MCNT(x)   ((x)<<20)
#define OTG_HCCHAR_ETYP_CTRL (0u<<18)
#define OTG_HCCHAR_ETYP_ISO  (1u<<18)
#define OTG_HCCHAR_ETYP_BULK (2u<<18)
#define OTG_HCCHAR_ETYP_INT  (3u<<18)
#define OTG_HCCHAR_LSDEV     (1u<<17)
#define OTG_HCCHAR_EPDIR_OUT (0u<<15)
#define OTG_HCCHAR_EPDIR_IN  (1u<<15)
#define OTG_HCCHAR_EPNUM(x)  ((x)<<11)
#define OTG_HCCHAR_MPSIZ(x)  ((x)<< 0)

#define OTG_HCTSIZ_DPID_DATA0 (0u<<29)
#define OTG_HCTSIZ_DPID_DATA2 (1u<<29)
#define OTG_HCTSIZ_DPID_DATA1 (2u<<29)
#define OTG_HCTSIZ_DPID_MDATA (3u<<29)
#define OTG_HCTSIZ_DPID_SETUP (3u<<29)
#define OTG_HCTSIZ_PKTCNT(x)  ((x)<<19)
#define OTG_HCTSIZ_XFRSIZ(x)  ((x)<< 0)

#define USB_OTG_BASE 0x50000000

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
