#ifndef STM32F072XB_BAREMETAL_H
#define STM32F072XB_BAREMETAL_H

// ============================================================================
// SECTION 1: CORE DATA TYPES AND VOLATILE MACRO
// ============================================================================

typedef unsigned int   uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef signed int     int32_t;
typedef signed short   int16_t;
typedef signed char    int8_t;

// The volatile keyword is implicitly used in the register structures, 
// but defining this helps ensure compiler optimization is avoided for memory-mapped registers.
#define __IO volatile

// ============================================================================
// SECTION 2: MEMORY BASE ADDRESSES
// ============================================================================

// Flash and SRAM Base
#define FLASH_BASE            (0x08000000UL)
#define SRAM_BASE             (0x20000000UL)

// Peripheral Bus Base Addresses
#define PERIPH_BASE           (0x40000000UL)
#define AHBPERIPH_BASE        (0x40020000UL) // Includes RCC, DMA, etc.
#define APBPERIPH_BASE        (0x40000000UL) // Includes GPIO, Timers, etc.
#define PPB_BASE              (0xE0000000UL) // Private Peripheral Bus (Cortex-M Core)

// ============================================================================
// SECTION 3: PERIPHERAL BASE ADDRESSES
// ============================================================================

// AHB Peripherals
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000UL)

// APB Peripherals (GPIO)
#define GPIOA_BASE            (APBPERIPH_BASE + 0x00000800UL)
#define GPIOB_BASE            (APBPERIPH_BASE + 0x00000C00UL)
#define GPIOC_BASE            (APBPERIPH_BASE + 0x00010000UL)
#define GPIOD_BASE            (APBPERIPH_BASE + 0x00010400UL)
#define GPIOF_BASE            (APBPERIPH_BASE + 0x00011400UL)

// Cortex-M Core Peripherals
#define SYSTICK_BASE          (PPB_BASE + 0x0000E010UL)
#define NVIC_BASE             (PPB_BASE + 0x0000E100UL)

// ============================================================================
// SECTION 4: PERIPHERAL REGISTER STRUCTURES
// ============================================================================

// --- RCC (Reset and Clock Control) Structure ---
typedef struct {
    __IO uint32_t CR;      // Clock control register,             Address offset: 0x00
    __IO uint32_t CFGR;    // Configuration register,             Address offset: 0x04
    __IO uint32_t CIR;     // Clock interrupt register,           Address offset: 0x08
    __IO uint32_t APB2RSTR;// APB2 peripheral reset register,     Address offset: 0x0C
    __IO uint32_t APB1RSTR;// APB1 peripheral reset register,     Address offset: 0x10
    __IO uint32_t AHBENR;  // AHB peripheral clock enable register,Address offset: 0x14
    __IO uint32_t APB2ENR; // APB2 peripheral clock enable register,Address offset: 0x18
    __IO uint32_t APB1ENR; // APB1 peripheral clock enable register,Address offset: 0x1C
    __IO uint32_t BDCR;    // Backup domain control register,     Address offset: 0x20
    __IO uint32_t CSR;     // Control/status register,            Address offset: 0x24
    __IO uint32_t AHBRSTR; // AHB peripheral reset register,      Address offset: 0x28
    __IO uint32_t CFGR2;   // Configuration register 2,           Address offset: 0x2C
    __IO uint32_t CFGR3;   // Configuration register 3,           Address offset: 0x30
    __IO uint32_t CR2;     // Clock control register 2,           Address offset: 0x34
} RCC_TypeDef;

// --- GPIO (General Purpose I/O) Structure ---
typedef struct {
    __IO uint32_t MODER;   // GPIO port mode register,        Address offset: 0x00
    __IO uint32_t OTYPER;  // GPIO port output type register, Address offset: 0x04
    __IO uint32_t OSPEEDR; // GPIO port output speed register,Address offset: 0x08
    __IO uint32_t PUPDR;   // GPIO port pull-up/pull-down reg,Address offset: 0x0C
    __IO uint32_t IDR;     // GPIO port input data register,  Address offset: 0x10
    __IO uint32_t ODR;     // GPIO port output data register, Address offset: 0x14
    __IO uint32_t BSRR;    // GPIO port bit set/reset reg,    Address offset: 0x18
    __IO uint32_t LCKR;    // GPIO port configuration lock reg,Address offset: 0x1C
    __IO uint32_t AFR[2];  // GPIO alternate function low/high,Address offset: 0x20-0x24
    __IO uint32_t BRR;     // GPIO port bit reset register,   Address offset: 0x28
} GPIO_TypeDef;

// --- SysTick Timer Structure ---
typedef struct {
    __IO uint32_t CTRL;    // Control and Status Register,       Address offset: 0x00
    __IO uint32_t LOAD;    // Reload Value Register,             Address offset: 0x04
    __IO uint32_t VAL;     // Current Value Register,            Address offset: 0x08
    __IO uint32_t CALIB;   // Calibration Register,              Address offset: 0x0C
} SysTick_TypeDef;

// --- NVIC (Nested Vectored Interrupt Controller) Structure ---
typedef struct {
    __IO uint32_t ISER[1];    // Interrupt Set-Enable Register
    uint8_t RESERVED0[124];
    __IO uint32_t ICER[1];    // Interrupt Clear-Enable Register
    uint8_t RESERVED1[124];
    __IO uint32_t ISPR[1];    // Interrupt Set-Pending Register
    uint8_t RESERVED2[124];
    __IO uint32_t ICPR[1];    // Interrupt Clear-Pending Register
    uint8_t RESERVED3[380];
    __IO uint32_t IPR[8];     // Interrupt Priority Registers
} NVIC_TypeDef;

// ============================================================================
// SECTION 5: PERIPHERAL POINTER DEFINITIONS (REGISTER ACCESS)
// ============================================================================

#define RCC                 ((RCC_TypeDef *) RCC_BASE)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)

#define SysTick             ((SysTick_TypeDef *) SYSTICK_BASE)
#define NVIC                ((NVIC_TypeDef *) NVIC_BASE)

// ============================================================================
// SECTION 6: BIT-FIELD DEFINITIONS (A small selection for core functionality)
// ============================================================================

// --- RCC_AHBENR Register Bit Definitions (GPIO Clock Enable) ---
#define RCC_AHBENR_GPIOAEN          (1U << 17)
#define RCC_AHBENR_GPIOBEN          (1U << 18)
// ... (GPIOCEN, GPIODEN, etc. follow)

// --- GPIO_MODER Mode Definitions ---
#define GPIO_MODER_INPUT            (0U)
#define GPIO_MODER_OUTPUT           (1U)
#define GPIO_MODER_ALT              (2U)
#define GPIO_MODER_ANALOG           (3U)

// GPIO Helper Macros
#define GPIO_MODER_PN_POS(N)        ((N) * 2U)
#define GPIO_MODER_PN_MASK(N)       (3U << GPIO_MODER_PN_POS(N))
#define GPIO_MODER_PN_OUTPUT(N)     (GPIO_MODER_OUTPUT << GPIO_MODER_PN_POS(N))
#define GPIO_BSRR_SET(N)            (1U << (N))
#define GPIO_BSRR_RESET(N)          (1U << ((N) + 16U))

// --- SysTick_CTRL Register Bit Definitions ---
#define SYSTICK_CTRL_ENABLE         (1U << 0)
#define SYSTICK_CTRL_TICKINT        (1U << 1)
#define SYSTICK_CTRL_CLKSOURCE      (1U << 2)

// --- NVIC IRQ Definitions (Enumeration) ---
typedef enum {
    // ... (Core Exceptions defined previously)
    WWDG_IRQn            = 0,
    RTC_IRQn             = 2,
    FLASH_IRQn           = 3,
    EXTI0_1_IRQn         = 5,
    EXTI4_15_IRQn        = 7,
    DMA1_Channel1_IRQn   = 9,
    ADC_COMP_IRQn        = 12,
    TIM2_IRQn            = 15,
    USART1_IRQn          = 27,
    // ... (All 32 external IRQs defined previously)
} IRQn_Type;

// NVIC Helper Macros (Simplified)
#define NVIC_EnableIRQ(IRQn)        (NVIC->ISER[0] = (1U << (IRQn)))
#define NVIC_DisableIRQ(IRQn)       (NVIC->ICER[0] = (1U << (IRQn)))


#endif // STM32F072XB_BAREMETAL_H