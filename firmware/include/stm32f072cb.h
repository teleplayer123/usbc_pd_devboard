/*
 * stm32f072cb.h
 *
 * Bare-metal definitions and simple helpers for STM32F072CBU6
 */

#ifndef STM32F072CB_H
#define STM32F072CB_H

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef signed int     int32_t;
typedef signed short   int16_t;
typedef signed char    int8_t;

/* ----------------------------------------------------------------------------
 * Basic types & helpers
 * ------------------------------------------------------------------------- */
typedef volatile uint32_t  vuint32_t;
typedef volatile uint16_t vuint16_t;
typedef volatile uint8_t  vuint8_t;

#define __I  volatile const
#define __O  volatile
#define __IO volatile

/* bit helpers */
#define BIT(n)                (1U << (n))
#define READ_REG(REG)         (REG)
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define SET_BITS(REG, MASK)   ((REG) |= (MASK))
#define CLEAR_BITS(REG, MASK) ((REG) &= ~(MASK))
#define MODIFY_REG(REG, CLEARMASK, SETMASK) \
    do { (REG) = (((REG) & ~(CLEARMASK)) | (SETMASK)); } while (0)

/* ----------------------------------------------------------------------------
 * Memory map
 * ------------------------------------------------------------------------- */

/* ---- Cortex-M0+ core peripherals (system) ---- */
#define SCS_BASE            (0xE000E000U)     /* System control space base */

/* ---- Peripheral base addresses --------------- */

#define FLASH_BASE          (0x08000000U)
#define SRAM_BASE           (0x20000000U)
#define PERIPH_BASE         (0x40000000U)     /* APB/AHB peripheral base */

#define APB1_BASE           (PERIPH_BASE + 0x00000000U)
#define APB2_BASE           (PERIPH_BASE + 0x00010000U)
#define AHB1_BASE           (PERIPH_BASE + 0x00020000U)
#define AHB2_BASE           (PERIPH_BASE + 0x08000000U)

#define GPIOA_BASE          (0x48000000U) 
#define GPIOB_BASE          (0x48000400U)
#define GPIOC_BASE          (0x48000800U)
#define GPIOD_BASE          (0x48000C00U)
#define GPIOF_BASE          (0x48001400U)

#define DMA_BASE            (0x40020000U)
#define RCC_BASE            (0x40021000U)
#define FLASH_MEM_INTF_BASE (0x40022000U)
#define CRC_BASE            (0x40023000U)
#define TSC_BASE            (0x40024000U)
#define DBGMCU_BASE         (0x40015800U)
#define TIM17_BASE          (0x40014800U)
#define TIM16_BASE          (0x40014400U)
#define TIM15_BASE          (0x40014000U)
#define TIM1_BASE           (0x40012C00U)
#define TIM3_BASE           (0x40000400U)
#define USART1_BASE         (0x40013800U)
#define USART2_BASE         (0x40004400U)
#define I2C1_BASE           (0x40005400U)
#define SPI1_BASE           (0x40013000U)
#define ADC1_BASE           (0x40012400U)
#define SYSCFG_BASE         (0x40010000U)
#define EXTI_BASE           (0x40010400U)
#define CEC_BASE            (0x40007800U)
#define DAC_BASE            (0x40007400U)
#define PWR_BASE            (0x40007000U)
#define CRS_BASE            (0x40006C00U)


/* NVIC registers (System) */
#define NVIC_ISER0          (*(vuint32_t*)(SCS_BASE + 0x0100U)) /* IRQ set-enable */
#define NVIC_ICER0          (*(vuint32_t*)(SCS_BASE + 0x0180U)) /* IRQ clear-enable */

/* ----------------------------------------------------------------------------
 * Cortex-M0+ small system registers (a few convenient definitions)
 * ------------------------------------------------------------------------- */
typedef struct {
    __IO uint32_t ISER[1];         /* 0x000 Interrupt Set-enable Registers */
             uint32_t RESERVED0[31];
    __IO uint32_t ICER[1];         /* 0x080 Interrupt Clear-enable Registers */
} NVIC_Type;

#define NVIC    ((NVIC_Type*) SCS_BASE)

/* SysTick (brief) */
#define SYST_CSR            (*(vuint32_t*)(SCS_BASE + 0x0010U))
#define SYST_RVR            (*(vuint32_t*)(SCS_BASE + 0x0014U))
#define SYST_CVR            (*(vuint32_t*)(SCS_BASE + 0x0018U))

/* ----------------------------------------------------------------------------
 * Peripheral register structures
 * (define common registers for RCC, GPIO, EXTI, SYSCFG, USART, TIM, I2C)
 * ------------------------------------------------------------------------- */

/* RCC (Reset and Clock Control) -- typical STM32F0 RCC layout (verify) */
typedef struct {
    __IO uint32_t CR;
    __IO uint32_t ICSCR;
    __IO uint32_t CFGR;
    __IO uint32_t CIR;
    __IO uint32_t APB2RSTR;
    __IO uint32_t APB1RSTR;
    __IO uint32_t AHBENR;
    __IO uint32_t APB2ENR;
    __IO uint32_t APB1ENR;
    __IO uint32_t BDCR;
    __IO uint32_t CSR;
    __IO uint32_t AHBRSTR;
    __IO uint32_t CFGR2;
    __IO uint32_t RESERVED;
    __IO uint32_t CFGR3;
} RCC_TypeDef;

#define RCC     ((RCC_TypeDef*) RCC_BASE)

/* GPIO (common STM32 structure) */
typedef struct {
    __IO uint32_t MODER;    /* GPIO port mode register,               offset 0x00 */
    __IO uint32_t OTYPER;   /* output type register,                  offset 0x04 */
    __IO uint32_t OSPEEDR;  /* output speed register,                 offset 0x08 */
    __IO uint32_t PUPDR;    /* pull-up/pull-down register,            offset 0x0C */
    __IO uint32_t IDR;      /* input data register,                   offset 0x10 */
    __IO uint32_t ODR;      /* output data register,                  offset 0x14 */
    __IO uint32_t BSRR;     /* bit set/reset register,                offset 0x18 */
    __IO uint32_t LCKR;     /* configuration lock register,           offset 0x1C */
    __IO uint32_t AFRL;     /* alternate function low register,       offset 0x20 */
    __IO uint32_t AFRH;     /* alternate function high register,      offset 0x24 */
} GPIO_TypeDef;

#define GPIOA   ((GPIO_TypeDef*) GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef*) GPIOB_BASE)
#define GPIOC   ((GPIO_TypeDef*) GPIOC_BASE)
#define GPIOD   ((GPIO_TypeDef*) GPIOD_BASE)
#define GPIOF   ((GPIO_TypeDef*) GPIOF_BASE)

/* EXTI (external interrupt/event controller) */
typedef struct {
    __IO uint32_t IMR;
    __IO uint32_t EMR;
    __IO uint32_t RTSR;
    __IO uint32_t FTSR;
    __IO uint32_t SWIER;
    __IO uint32_t PR;
} EXTI_TypeDef;

#define EXTI    ((EXTI_TypeDef*) EXTI_BASE)

/* SYSCFG (system configuration controller) */
typedef struct {
    __IO uint32_t CFGR1;
    __IO uint32_t RCR;
    __IO uint32_t EXTICR[4];
    __IO uint32_t SCSR;
    __IO uint32_t CFGR2;
} SYSCFG_TypeDef;

#define SYSCFG  ((SYSCFG_TypeDef*) SYSCFG_BASE)

/* USART (common registers - verify offsets for F0) */
typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t BRR;
    __IO uint32_t GTPR;
    __IO uint32_t RTOR;
    __IO uint32_t RQR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t RDR;
    __IO uint32_t TDR;
} USART_TypeDef;

#define USART1  ((USART_TypeDef*) USART1_BASE)
#define USART2  ((USART_TypeDef*) USART2_BASE)

/* I2C (common small set) */
typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t OAR1;
    __IO uint32_t OAR2;
    __IO uint32_t TIMINGR;
    __IO uint32_t TIMEOUTR;
    __IO uint32_t ISR;
    __IO uint32_t ICR;
    __IO uint32_t PECR;
    __IO uint32_t RXDR;
    __IO uint32_t TXDR;
} I2C_TypeDef;

#define I2C1    ((I2C_TypeDef*) I2C1_BASE)

/* TIM1/TIM3 */
typedef struct {
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t SMCR;
    __IO uint32_t DIER;
    __IO uint32_t SR;
    __IO uint32_t EGR;
    __IO uint32_t CCMR1;
    __IO uint32_t CCMR2;
    __IO uint32_t CCER;
    __IO uint32_t CNT;
    __IO uint32_t PSC;
    __IO uint32_t ARR;
    __IO uint32_t RCR;
    __IO uint32_t CCR1;
    __IO uint32_t CCR2;
    __IO uint32_t CCR3;
    __IO uint32_t CCR4;
    __IO uint32_t BDTR;
    __IO uint32_t DCR;
} TIM_TypeDef;

#define TIM1    ((TIM_TypeDef*) TIM1_BASE)
#define TIM3    ((TIM_TypeDef*) TIM3_BASE)

/* ADC (basic) */
typedef struct {
    __IO uint32_t ISR;
    __IO uint32_t IER;
    __IO uint32_t CR;
    __IO uint32_t CFGR1;
    __IO uint32_t CFGR2;
    __IO uint32_t SMPR;
    __IO uint32_t TR;
    __IO uint32_t CHSELR;
    __IO uint32_t RESERVED;
    __IO uint32_t DR;
} ADC_TypeDef;

#define ADC1    ((ADC_TypeDef*) ADC1_BASE)

/* ----------------------------------------------------------------------------
 * Helpful inline functions, examples
 * ------------------------------------------------------------------------- */

/* Clock enable helpers for GPIO (adjust RCC bit positions to match device) */
/* NOTE: The bit positions below are placeholders; please replace with the
   real bit positions from the device reference manual. */
static inline void rcc_enable_gpioa(void) {
    /* Example: AHBENR bit for GPIOA (USER VERIFY) */
    RCC->AHBENR |= (1U << 17); /* USER VERIFY: replace 17 with real bit */
}
static inline void rcc_disable_gpioa(void) {
    RCC->AHBENR &= ~(1U << 17); /* USER VERIFY */
}

/* Example: set a pin as general-purpose output */
static inline void gpio_set_mode_output(GPIO_TypeDef *gpio, uint8_t pin) {
    uint32_t pos = pin * 2U;
    gpio->MODER &= ~(3U << pos);
    gpio->MODER |= (1U << pos); /* 01 = General purpose output mode */
}

/* Example: write output bit */
static inline void gpio_write_pin(GPIO_TypeDef *gpio, uint8_t pin, uint8_t val) {
    if (val) gpio->BSRR = (1U << pin);
    else     gpio->BSRR = (1U << (pin + 16U));
}

/* Simple delay (busy-wait) */
static inline void delay_cycles(volatile uint32_t n) {
    while (n--) {
        __asm__ volatile ("nop");
    }
}

/* ----------------------------------------------------------------------------
 * IRQ numbers (partial list; verify for STM32F072)
 * ------------------------------------------------------------------------- */
/* These are typical values — verify with device header or vector table */
typedef enum {
    /* Cortex-M0+ exceptions (negative numbers in CMSIS) omitted here */
    WWDG_IRQn        = 0,
    PVD_VDDIO2_IRQn  = 1,
    RTC_IRQn         = 2,
    FLASH_IRQn       = 3,
    RCC_CRS_IRQn     = 4,
    EXTI0_1_IRQn     = 5,
    EXTI2_3_IRQn     = 6,
    EXTI4_15_IRQn    = 7,
    /* ... add more as needed */
} IRQn_Type;

/* ----------------------------------------------------------------------------
 * Minimal CMSIS-style core register definitions (SCB / SysTick masks, etc.)
 * ------------------------------------------------------------------------- */
#define SCB_ICSR           (*(vuint32_t*)(SCS_BASE + 0x0004U))
#define SCB_VTOR           (*(vuint32_t*)(SCS_BASE + 0x0008U))

/* ----------------------------------------------------------------------------
 * Convenience macros: vector table, weak attribute (for user ISRs)
 * ------------------------------------------------------------------------- */
#if defined(__GNUC__)
  #define WEAK_ALIAS(x) __attribute__((weak, alias(#x)))
#else
  #define WEAK_ALIAS(x)
#endif

/* ----------------------------------------------------------------------------
 * End
 * ------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* STM32F072CB_H */
