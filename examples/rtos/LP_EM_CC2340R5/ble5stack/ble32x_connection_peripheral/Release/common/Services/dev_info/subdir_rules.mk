################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
common/Services/dev_info/dev_info_service.o: C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/services/dev_info/dev_info_service.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/bin/tiarmclang.exe" -c @"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/config/build_components.opt" @"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/config/factory_config.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -I"C:/Users/a0509812/Bitbucket/Peripheral/ble_examples/examples/rtos/LP_EM_CC2340R5/ble5stack/ble32x_connection_peripheral" -I"C:/Users/a0509812/Bitbucket/Peripheral/ble_examples/examples/rtos/LP_EM_CC2340R5/ble5stack/ble32x_connection_peripheral/Release" -I"C:/Users/a0509812/Bitbucket/Peripheral/ble_examples/examples/rtos/LP_EM_CC2340R5/ble5stack/ble32x_connection_peripheral/app" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/controller/cc26xx/inc" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/inc" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/rom" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/common/cc26xx" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/inc" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/hal/src/target/_common" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/common/cc26xx/npi/stack" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/hal/src/inc" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/heapmgr" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/profiles/dev_info" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/profiles/simple_profile" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/src/inc" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/npi/src" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/osal/src/inc" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/services/src/saddr" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/services/src/sdata" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/common/nv" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/common/cc26xx" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/src" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/profiles/health_thermometer" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/services/health_thermometer" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/drivers/rcl" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/posix/ticlang" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/third_party/freertos/include" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/source/third_party/freertos/portable/GCC/ARM_CM0" -I"C:/ti/simplelink_lowpower_f3_sdk_7_40_00_64/kernel/freertos" -DICALL_NO_APP_EVENTS -DCC23X0 -DNVOCMP_NWSAMEITEM=1 -DFLASH_ONLY_BUILD -DUSE_RCL -DFREERTOS -DNVOCMP_POSIX_MUTEX -gdwarf-3 -ffunction-sections -MMD -MP -MF"common/Services/dev_info/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/a0509812/Bitbucket/Peripheral/ble_examples/examples/rtos/LP_EM_CC2340R5/ble5stack/ble32x_connection_peripheral/Release/syscfg" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


