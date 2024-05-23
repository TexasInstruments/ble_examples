/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the LP_EM_CC2340R5
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_LP_EM_CC2340R5
#ifndef DeviceFamily_CC23X0R5
#define DeviceFamily_CC23X0R5
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== AESCCM ========
 */

extern const uint_least8_t                  CONFIG_AESCCM0_CONST;
#define CONFIG_AESCCM0                      0
#define CONFIG_TI_DRIVERS_AESCCM_COUNT      1


/*
 *  ======== AESCTRDRBG ========
 */

extern const uint_least8_t                      CONFIG_AESCTRDRBG_0_CONST;
#define CONFIG_AESCTRDRBG_0                     0
#define CONFIG_TI_DRIVERS_AESCTRDRBG_COUNT      1


/*
 *  ======== AESECB ========
 */

extern const uint_least8_t                  CONFIG_AESECB0_CONST;
#define CONFIG_AESECB0                      0
#define CONFIG_TI_DRIVERS_AESECB_COUNT      1


/*
 *  ======== ECDH ========
 */

extern const uint_least8_t              CONFIG_ECDH0_CONST;
#define CONFIG_ECDH0                    0
#define CONFIG_TI_DRIVERS_ECDH_COUNT    1


/*
 *  ======== GPIO ========
 */
extern const uint_least8_t CONFIG_GPIO_LED_GREEN_CONST;
#define CONFIG_GPIO_LED_GREEN 15

extern const uint_least8_t CONFIG_GPIO_LED_RED_CONST;
#define CONFIG_GPIO_LED_RED 14

/* Owned by CONFIG_BUTTON_0 as  */
extern const uint_least8_t CONFIG_GPIO_BUTTON_0_INPUT_CONST;
#define CONFIG_GPIO_BUTTON_0_INPUT 10

/* Owned by CONFIG_BUTTON_1 as  */
extern const uint_least8_t CONFIG_GPIO_BUTTON_1_INPUT_CONST;
#define CONFIG_GPIO_BUTTON_1_INPUT 9

/* Owned by CONFIG_DISPLAY_UART as  */
extern const uint_least8_t CONFIG_PIN_UART_TX_CONST;
#define CONFIG_PIN_UART_TX 20

/* Owned by CONFIG_DISPLAY_UART as  */
extern const uint_least8_t CONFIG_PIN_UART_RX_CONST;
#define CONFIG_PIN_UART_RX 22

/* The range of pins available on this device */
extern const uint_least8_t GPIO_pinLowerBound;
extern const uint_least8_t GPIO_pinUpperBound;

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== NVS ========
 */

extern const uint_least8_t              CONFIG_NVSINTERNAL_CONST;
#define CONFIG_NVSINTERNAL              0
#define CONFIG_TI_DRIVERS_NVS_COUNT     1





/*
 *  ======== RNG ========
 */

extern const uint_least8_t              CONFIG_RNG_0_CONST;
#define CONFIG_RNG_0                    0
#define CONFIG_TI_DRIVERS_RNG_COUNT     1

#define RNG_POOL_BYTE_SIZE 32

#define RNG_NOISE_INPUT_WORD_SIZE 152


/*
 *  ======== UART2 ========
 */

/*
 *  TX: DIO20A11
 *  RX: DIO22A9
 *  XDS110 UART
 */
extern const uint_least8_t                  CONFIG_DISPLAY_UART_CONST;
#define CONFIG_DISPLAY_UART                 0
#define CONFIG_TI_DRIVERS_UART2_COUNT       1


/*
 *  ======== Button ========
 */

extern const uint_least8_t                  CONFIG_BUTTON_0_CONST;
#define CONFIG_BUTTON_0                     0
extern const uint_least8_t                  CONFIG_BUTTON_1_CONST;
#define CONFIG_BUTTON_1                     1
#define CONFIG_TI_DRIVERS_BUTTON_COUNT      2


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
