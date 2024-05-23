/*
 *  ======== ti_drivers_config.c ========
 *  Configured TI-Drivers module definitions
 *
 *  DO NOT EDIT - This file is generated for the LP_EM_CC2340R5
 *  by the SysConfig tool.
 */

#include <stddef.h>
#include <stdint.h>

#ifndef DeviceFamily_CC23X0R5
#define DeviceFamily_CC23X0R5
#endif

#include <ti/devices/DeviceFamily.h>

#include "ti_drivers_config.h"

/*
 *  ============================= Display =============================
 */

#include <ti/display/Display.h>
#include <ti/display/DisplayUart2.h>

#define CONFIG_Display_COUNT 1


#define Display_UART2BUFFERSIZE 128
static char displayUART2Buffer[Display_UART2BUFFERSIZE];

DisplayUart2_Object displayUart2Object;

const DisplayUart2_HWAttrs displayUart2HWAttrs = {
    .uartIdx      = CONFIG_DISPLAY_UART,
    .baudRate     = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf       = displayUART2Buffer,
    .strBufLen    = Display_UART2BUFFERSIZE
};

const Display_Config Display_config[CONFIG_Display_COUNT] = {
    /* CONFIG_Display_0 */
    /* XDS110 UART */
    {
        .fxnTablePtr = &DisplayUart2Ansi_fxnTable,
        .object      = &displayUart2Object,
        .hwAttrs     = &displayUart2HWAttrs
    },
};

const uint_least8_t Display_count = CONFIG_Display_COUNT;

/*
 *  =============================== AESCCM ===============================
 */

#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMLPF3.h>

#define CONFIG_AESCCM_COUNT 1
AESCCMLPF3_Object AESCCMLPF3_objects[CONFIG_AESCCM_COUNT];

/*
 *  ======== AESCCMLPF3_hwAttrs ========
 */
const AESCCMLPF3_HWAttrs AESCCMLPF3_hwAttrs[CONFIG_AESCCM_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const AESCCM_Config AESCCM_config[CONFIG_AESCCM_COUNT] = {
    {   /* CONFIG_AESCCM0 */
        .object  = &AESCCMLPF3_objects[CONFIG_AESCCM0],
        .hwAttrs = &AESCCMLPF3_hwAttrs[CONFIG_AESCCM0]
    },
};

const uint_least8_t CONFIG_AESCCM0_CONST = CONFIG_AESCCM0;
const uint_least8_t AESCCM_count = CONFIG_AESCCM_COUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */

#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

#define CONFIG_AESCTRDRBG_COUNT 1

/*
 *  ======== aesctrdrbgXXObjects ========
 */
AESCTRDRBGXX_Object aesctrdrbgXXObjects[CONFIG_AESCTRDRBG_COUNT];

/*
 *  ======== aesctrdrbgXXHWAttrs ========
 */
const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CONFIG_AESCTRDRBG_COUNT] = {
    /* CONFIG_AESCTRDRBG_0 */
    {
        .aesctrHWAttrs.intPriority = (~0)
    },
};

/*
 *  ======== AESCTRDRBG_config ========
 */
const AESCTRDRBG_Config AESCTRDRBG_config[CONFIG_AESCTRDRBG_COUNT] = {
    /* CONFIG_AESCTRDRBG_0 */
    {
        .object = &aesctrdrbgXXObjects[CONFIG_AESCTRDRBG_0],
        .hwAttrs = &aesctrdrbgXXHWAttrs[CONFIG_AESCTRDRBG_0]
    },
};

const uint_least8_t CONFIG_AESCTRDRBG_0_CONST = CONFIG_AESCTRDRBG_0;
const uint_least8_t AESCTRDRBG_count = CONFIG_AESCTRDRBG_COUNT;

/*
 *  =============================== AESECB ===============================
 */

#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBLPF3.h>

#define CONFIG_AESECB_COUNT 1
AESECBLPF3_Object AESECBLPF3_objects[CONFIG_AESECB_COUNT];

/*
 *  ======== AESECBLPF3_hwAttrs ========
 */
const AESECBLPF3_HWAttrs AESECBLPF3_hwAttrs[CONFIG_AESECB_COUNT] = {
    {
        .intPriority = (~0),
    },
};

const AESECB_Config AESECB_config[CONFIG_AESECB_COUNT] = {
    {   /* CONFIG_AESECB0 */
        .object  = &AESECBLPF3_objects[CONFIG_AESECB0],
        .hwAttrs = &AESECBLPF3_hwAttrs[CONFIG_AESECB0]
    },
};


const uint_least8_t CONFIG_AESECB0_CONST = CONFIG_AESECB0;
const uint_least8_t AESECB_count = CONFIG_AESECB_COUNT;

/*
 *  =============================== DMA ===============================
 */

#include <ti/drivers/dma/UDMALPF3.h>
#include <ti/devices/cc23x0r5/inc/hw_memmap.h>

const UDMALPF3_Config UDMALPF3_config = {
        .CtrlBaseAddr = UDMALPF3_CONFIG_BASE,
};

/*
 *  =============================== ECDH ===============================
 */

#include <ti/drivers/ECDH.h>
#include <ti/drivers/ecdh/ECDHLPF3SW.h>

#define CONFIG_ECDH_COUNT 1

ECDHLPF3SW_Object ecdhLpf3swObjects[CONFIG_ECDH_COUNT];

/*
 *  ======== ecdhLpf3swHWAttrs ========
 */
const ECDHLPF3SW_HWAttrs ecdhLpf3swHWAttrs[CONFIG_ECDH_COUNT] = {
    {0},
};

const ECDH_Config ECDH_config[CONFIG_ECDH_COUNT] = {
    {   /* CONFIG_ECDH0 */
        .object         = &ecdhLpf3swObjects[CONFIG_ECDH0],
        .hwAttrs        = &ecdhLpf3swHWAttrs[CONFIG_ECDH0]
    },
};

const uint_least8_t CONFIG_ECDH0_CONST = CONFIG_ECDH0;
const uint_least8_t ECDH_count = CONFIG_ECDH_COUNT;

/*
 *  =============================== GPIO ===============================
 */

#include <ti/drivers/GPIO.h>

/* The range of pins available on this device */
const uint_least8_t GPIO_pinLowerBound = 0;
const uint_least8_t GPIO_pinUpperBound = 25;

/*
 *  ======== gpioPinConfigs ========
 *  Array of Pin configurations
 */
GPIO_PinConfig gpioPinConfigs[26] = {
    GPIO_CFG_NO_DIR, /* DIO_0 */
    GPIO_CFG_NO_DIR, /* DIO_1 */
    GPIO_CFG_NO_DIR, /* DIO_2 */
    GPIO_CFG_NO_DIR, /* DIO_3 */
    GPIO_CFG_NO_DIR, /* DIO_4 */
    GPIO_CFG_NO_DIR, /* DIO_5 */
    GPIO_CFG_NO_DIR, /* DIO_6 */
    GPIO_CFG_NO_DIR, /* DIO_7 */
    GPIO_CFG_NO_DIR, /* DIO_8 */
    /* Owned by CONFIG_BUTTON_1 as Button GPIO */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_NONE_INTERNAL, /* CONFIG_GPIO_BUTTON_1_INPUT */
    /* Owned by CONFIG_BUTTON_0 as Button GPIO */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_NONE_INTERNAL, /* CONFIG_GPIO_BUTTON_0_INPUT */
    GPIO_CFG_NO_DIR, /* DIO_11 */
    GPIO_CFG_NO_DIR, /* DIO_12 */
    GPIO_CFG_NO_DIR, /* DIO_13 */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW, /* CONFIG_GPIO_LED_RED */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW, /* CONFIG_GPIO_LED_GREEN */
    GPIO_CFG_NO_DIR, /* DIO_16 */
    GPIO_CFG_NO_DIR, /* DIO_17 */
    GPIO_CFG_NO_DIR, /* DIO_18 */
    GPIO_CFG_NO_DIR, /* DIO_19 */
    /* Owned by CONFIG_DISPLAY_UART as TX */
    GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH, /* CONFIG_PIN_UART_TX */
    GPIO_CFG_NO_DIR, /* DIO_21 */
    /* Owned by CONFIG_DISPLAY_UART as RX */
    GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_IN_INT_NONE | GPIO_CFG_PULL_DOWN_INTERNAL, /* CONFIG_PIN_UART_RX */
    GPIO_CFG_NO_DIR, /* DIO_23 */
    GPIO_CFG_NO_DIR, /* DIO_24 */
    GPIO_CFG_NO_DIR, /* DIO_25 */
};

/*
 *  ======== gpioCallbackFunctions ========
 *  Array of callback function pointers
 *  Change at runtime with GPIO_setCallback()
 */
GPIO_CallbackFxn gpioCallbackFunctions[26];

/*
 *  ======== gpioUserArgs ========
 *  Array of user argument pointers
 *  Change at runtime with GPIO_setUserArg()
 *  Get values with GPIO_getUserArg()
 */
void* gpioUserArgs[26];

const uint_least8_t CONFIG_GPIO_LED_GREEN_CONST = CONFIG_GPIO_LED_GREEN;
const uint_least8_t CONFIG_GPIO_LED_RED_CONST = CONFIG_GPIO_LED_RED;
const uint_least8_t CONFIG_GPIO_BUTTON_0_INPUT_CONST = CONFIG_GPIO_BUTTON_0_INPUT;
const uint_least8_t CONFIG_GPIO_BUTTON_1_INPUT_CONST = CONFIG_GPIO_BUTTON_1_INPUT;
const uint_least8_t CONFIG_PIN_UART_TX_CONST = CONFIG_PIN_UART_TX;
const uint_least8_t CONFIG_PIN_UART_RX_CONST = CONFIG_PIN_UART_RX;

/*
 *  ======== GPIO_config ========
 */
const GPIO_Config GPIO_config = {
    .configs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .userArgs = gpioUserArgs,
    .intPriority = (~0)
};

/*
 *  =============================== NVS ===============================
 */

#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSLPF3.h>

/*
 *  NVSLPF3 Internal NVS flash region definitions
 *
 * Place uninitialized char arrays at addresses
 * corresponding to the 'regionBase' addresses defined in
 * the configured NVS regions. These arrays are used as
 * place holders so that the linker will not place other
 * content there.
 *
 * For GCC targets, the char arrays are each placed into
 * the shared ".nvs" section. The user must add content to
 * their GCC linker command file to place the .nvs section
 * at the lowest 'regionBase' address specified in their NVS
 * regions.
 */

#if defined(__TI_COMPILER_VERSION__) || defined(__clang__)

static char flashBuf0[0x4000] __attribute__ ((retain, noinit, location(0x7c000)));

#elif defined(__IAR_SYSTEMS_ICC__)

__no_init static char flashBuf0[0x4000] @ 0x7c000;

#elif defined(__GNUC__)

__attribute__ ((section (".nvs")))
static char flashBuf0[0x4000];

#endif

NVSLPF3_Object NVSLPF3_objects[1];

static const NVSLPF3_HWAttrs NVSLPF3_hwAttrs[1] = {
    /* CONFIG_NVSINTERNAL */
    {
        .regionBase = (void *) flashBuf0,
        .regionSize = 0x4000
    },
};

#define CONFIG_NVS_COUNT 1

const NVS_Config NVS_config[CONFIG_NVS_COUNT] = {
    /* CONFIG_NVSINTERNAL */
    {
        .fxnTablePtr = &NVSLPF3_fxnTable,
        .object = &NVSLPF3_objects[0],
        .hwAttrs = &NVSLPF3_hwAttrs[0],
    },
};

const uint_least8_t CONFIG_NVSINTERNAL_CONST = CONFIG_NVSINTERNAL;
const uint_least8_t NVS_count = CONFIG_NVS_COUNT;

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include "ti_drivers_config.h"
#include DeviceFamily_constructPath(driverlib/ckmd.h)
#include DeviceFamily_constructPath(driverlib/pmctl.h)

extern void PowerCC23X0_standbyPolicy(void);

const PowerCC23X0_Config PowerCC23X0_config = {
    .policyInitFxn            = NULL,
    .policyFxn                = PowerCC23X0_standbyPolicy,
};





/*
 *  =============================== RNG ===============================
 */

#include <ti/drivers/RNG.h>
#include <ti/drivers/rng/RNGLPF3RF.h>

#define CONFIG_RNG_COUNT 1

const size_t RNG_poolByteSize = RNG_POOL_BYTE_SIZE;
#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=4
#else
__attribute__((aligned(4)))
#endif
uint8_t  RNG_instancePool[RNG_POOL_BYTE_SIZE];

const RNG_ReturnBehavior RNGLPF3RF_returnBehavior = RNG_RETURN_BEHAVIOR_POLLING;

const RNGLPF3RF_HWAttrs RNGLPF3RF_hwAttrs = {
    .intPriority = (~0)
};

RNGLPF3RF_Object RNGLPF3RF_objects[CONFIG_RNG_COUNT];

const RNG_Config RNG_config[CONFIG_RNG_COUNT] = {
    {   /* CONFIG_RNG_0 */
        .object         = &RNGLPF3RF_objects[CONFIG_RNG_0],
        .hwAttrs        = &RNGLPF3RF_hwAttrs
    },
};

const uint_least8_t CONFIG_RNG_0_CONST = CONFIG_RNG_0;
const uint_least8_t RNG_count = CONFIG_RNG_COUNT;

const uint32_t RNGLPF3RF_noiseInputWordLen = 152;

uint32_t RNGLPF3RF_noiseConditioningKeyWord0 = 0x111de874;
uint32_t RNGLPF3RF_noiseConditioningKeyWord1 = 0x6cecb00e;
uint32_t RNGLPF3RF_noiseConditioningKeyWord2 = 0x7fb76dc5;
uint32_t RNGLPF3RF_noiseConditioningKeyWord3 = 0x8e020ca2;

/*
 *  =============================== UART2 ===============================
 */

#include <ti/drivers/UART2.h>
#include <ti/drivers/uart2/UART2LPF3.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/dma/UDMALPF3.h>
#include <ti/drivers/Power.h>
#include <ti/devices/cc23x0r5/inc/hw_memmap.h>
#include <ti/devices/cc23x0r5/inc/hw_ints.h>
#include <ti/devices/cc23x0r5/inc/hw_evtsvt.h>

#define CONFIG_UART2_COUNT 1

UART2LPF3_Object UART2LPF3_objects[CONFIG_UART2_COUNT];

static unsigned char uart2RxRingBuffer0[32];
/* TX ring buffer allocated to be used for nonblocking mode */
static unsigned char uart2TxRingBuffer0[32];



ALLOCATE_CONTROL_TABLE_ENTRY(dmaChannel0ControlTableEntry, 0);
ALLOCATE_CONTROL_TABLE_ENTRY(dmaChannel1ControlTableEntry, 1);

static const UART2LPF3_HWAttrs UART2LPF3_hwAttrs[CONFIG_UART2_COUNT] = {
  {
    .baseAddr           = UART0_BASE,
    .intNum             = INT_UART0_COMB,
    .intPriority        = (~0),
    .rxPin              = CONFIG_PIN_UART_RX,
    .txPin              = CONFIG_PIN_UART_TX,
    .ctsPin             = GPIO_INVALID_INDEX,
    .rtsPin             = GPIO_INVALID_INDEX,
    .flowControl        = UART2_FLOWCTRL_NONE,
    .rxBufPtr           = uart2RxRingBuffer0,
    .rxBufSize          = sizeof(uart2RxRingBuffer0),
    .txBufPtr           = uart2TxRingBuffer0,
    .txBufSize          = sizeof(uart2TxRingBuffer0),
    .txPinMux           = GPIO_MUX_PORTCFG_PFUNC2,
    .rxPinMux           = GPIO_MUX_PORTCFG_PFUNC2,
    .ctsPinMux          = GPIO_MUX_GPIO_INTERNAL,
    .rtsPinMux          = GPIO_MUX_GPIO_INTERNAL,
    .dmaRxTableEntryPri = &dmaChannel0ControlTableEntry,
    .dmaTxTableEntryPri = &dmaChannel1ControlTableEntry,
    .rxChannelMask      = UDMA_CHANNEL_0_M,
    .txChannelMask      = UDMA_CHANNEL_1_M,
    .rxChannelEvtMux    = EVTSVT_DMACH0SEL_IPID_UART0RXTRG,
    .txChannelEvtMux    = EVTSVT_DMACH1SEL_IPID_UART0TXTRG,
    .codingScheme       = UART2LPF3_CODING_UART,
    .concatenateFIFO    = false,
    .irLPClkDivider     = 0,
    .powerID            = PowerLPF3_PERIPH_UART0
  },
};

const UART2_Config UART2_config[CONFIG_UART2_COUNT] = {
    {   /* CONFIG_DISPLAY_UART */
        .object      = &UART2LPF3_objects[CONFIG_DISPLAY_UART],
        .hwAttrs     = &UART2LPF3_hwAttrs[CONFIG_DISPLAY_UART]
    },
};

const uint_least8_t CONFIG_DISPLAY_UART_CONST = CONFIG_DISPLAY_UART;
const uint_least8_t UART2_count = CONFIG_UART2_COUNT;


/*
 *  =============================== Button ===============================
 */
#include <ti/drivers/apps/Button.h>

#define CONFIG_BUTTON_COUNT 2
Button_Object ButtonObjects[CONFIG_BUTTON_COUNT];

static const Button_HWAttrs ButtonHWAttrs[CONFIG_BUTTON_COUNT] = {
    /* CONFIG_BUTTON_0 */
    /* LaunchPad Button BTN-1 (Left) */
    {
        .gpioIndex = CONFIG_GPIO_BUTTON_0_INPUT,
        .pullMode = Button_PULL_UP,
        .internalPullEnabled = 1,
    },
    /* CONFIG_BUTTON_1 */
    /* LaunchPad Button BTN-2 (Right) */
    {
        .gpioIndex = CONFIG_GPIO_BUTTON_1_INPUT,
        .pullMode = Button_PULL_UP,
        .internalPullEnabled = 1,
    },
};

const Button_Config Button_config[CONFIG_BUTTON_COUNT] = {
    /* CONFIG_BUTTON_0 */
    /* LaunchPad Button BTN-1 (Left) */
    {
        .object = &ButtonObjects[CONFIG_BUTTON_0],
        .hwAttrs = &ButtonHWAttrs[CONFIG_BUTTON_0]
    },
    /* CONFIG_BUTTON_1 */
    /* LaunchPad Button BTN-2 (Right) */
    {
        .object = &ButtonObjects[CONFIG_BUTTON_1],
        .hwAttrs = &ButtonHWAttrs[CONFIG_BUTTON_1]
    },
};

const uint_least8_t CONFIG_BUTTON_0_CONST = CONFIG_BUTTON_0;
const uint_least8_t CONFIG_BUTTON_1_CONST = CONFIG_BUTTON_1;
const uint_least8_t Button_count = CONFIG_BUTTON_COUNT;

/*
 *  =============================== BatMon Support ===============================
 */
#include <ti/drivers/batterymonitor/BatMonSupportLPF3.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_evtsvt.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)

const BatMonSupportLPF3_Config BatMonSupportLPF3_config = {
    .intNum = INT_CPUIRQ2,
    .intPriority = (~0),
    .intMux = EVTSVT_CPUIRQ2SEL_PUBID_AON_PMU_COMB
};

#include <stdbool.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/cpu.h)

#include <ti/drivers/GPIO.h>

/* Board GPIO defines */
#define BOARD_EXT_FLASH_SPI_CS      6
#define BOARD_EXT_FLASH_SPI_CLK     18
#define BOARD_EXT_FLASH_SPI_PICO    13
#define BOARD_EXT_FLASH_SPI_POCI    12

/*
 *  ======== Board_sendExtFlashByte ========
 */
void Board_sendExtFlashByte(uint8_t byte)
{
    uint8_t i;

    /* SPI Flash CS */
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 0);

    for (i = 0; i < 8; i++) {
        GPIO_write(BOARD_EXT_FLASH_SPI_CLK, 0); /* SPI Flash CLK */

        /* SPI Flash PICO */
        GPIO_write(BOARD_EXT_FLASH_SPI_PICO, (byte >> (7 - i)) & 0x01);
        GPIO_write(BOARD_EXT_FLASH_SPI_CLK, 1);  /* SPI Flash CLK */

        /*
         * Waste a few cycles to keep the CLK high for at
         * least 45% of the period.
         * 3 cycles per loop: 8 loops @ 48 MHz = 0.5 us.
         */
        CPUDelay(8);
    }

    GPIO_write(BOARD_EXT_FLASH_SPI_CLK, 0);  /* CLK */
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 1);  /* CS */

    /*
     * Keep CS high at least 40 us
     * 3 cycles per loop: 700 loops @ 48 MHz ~= 44 us
     */
    CPUDelay(700);
}

/*
 *  ======== Board_wakeUpExtFlash ========
 */
void Board_wakeUpExtFlash(void)
{
    /* SPI Flash CS*/
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_CS, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);

    /*
     *  To wake up we need to toggle the chip select at
     *  least 20 ns and ten wait at least 35 us.
     */

    /* Toggle chip select for ~20ns to wake ext. flash */
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 0);
    /* 3 cycles per loop: 1 loop @ 48 MHz ~= 62 ns */
    CPUDelay(1);
    GPIO_write(BOARD_EXT_FLASH_SPI_CS, 1);
    /* 3 cycles per loop: 560 loops @ 48 MHz ~= 35 us */
    CPUDelay(560);
}

/*
 *  ======== Board_shutDownExtFlash ========
 */
void Board_shutDownExtFlash(void)
{
    /*
     *  To be sure we are putting the flash into sleep and not waking it,
     *  we first have to make a wake up call
     */
    Board_wakeUpExtFlash();

    /* SPI Flash CS*/
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_CS, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);
    /* SPI Flash CLK */
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_CLK, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_MED);
    /* SPI Flash PICO */
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_PICO, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW | GPIO_CFG_OUT_STR_MED);
    /* SPI Flash POCI */
    GPIO_setConfig(BOARD_EXT_FLASH_SPI_POCI, GPIO_CFG_IN_PD);

    uint8_t extFlashShutdown = 0xB9;

    Board_sendExtFlashByte(extFlashShutdown);

    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_CS);
    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_CLK);
    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_PICO);
    GPIO_resetConfig(BOARD_EXT_FLASH_SPI_POCI);
}


#include <ti/drivers/Board.h>

/*
 *  ======== Board_initHook ========
 *  Perform any board-specific initialization needed at startup.  This
 *  function is declared weak to allow applications to override it if needed.
 */
void __attribute__((weak)) Board_initHook(void)
{
}

/*
 *  ======== Board_init ========
 *  Perform any initialization needed before using any board APIs
 */
void Board_init(void)
{
    /* ==== /ti/drivers/Power initialization ==== */

    Power_init();
    PowerLPF3_selectLFXT();
    PMCTLSetVoltageRegulator(PMCTL_VOLTAGE_REGULATOR_DCDC);



    /* ==== /ti/drivers/GPIO initialization ==== */
    /* Setup GPIO module and default-initialise pins */
    GPIO_init();

    /* ==== /ti/drivers/DMA initialization ==== */
    UDMALPF3_init();

    /* ==== /ti/drivers/RCL initialization ==== */

    Board_shutDownExtFlash();

    Board_initHook();
}

