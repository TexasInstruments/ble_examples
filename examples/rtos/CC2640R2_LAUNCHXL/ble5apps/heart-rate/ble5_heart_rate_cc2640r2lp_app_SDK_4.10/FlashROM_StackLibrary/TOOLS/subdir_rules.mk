################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-927661265:
	@$(MAKE) --no-print-directory -Onone -f TOOLS/subdir_rules.mk build-927661265-inproc

build-927661265-inproc: ../TOOLS/app_ble.cfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_51_02_21_core/xs" --xdcpath="C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source;C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/kernel/tirtos/packages;C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640R2F -r release -c "C:/ti/ccs1000/ccs/tools/compiler/ti-cgt-arm_16.9.11.LTS" --compileOptions "-mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/profiles/hid_dev/cc26xx\" --include_path=\"C:/Users/a0488519/workspace_v10/ble5_heart_rate_cc2640r2lp_app_SDK_4.10\" --include_path=\"C:/Users/a0488519/workspace_v10/ble5_heart_rate_cc2640r2lp_app_SDK_4.10/Application\" --include_path=\"C:/Users/a0488519/workspace_v10/ble5_heart_rate_cc2640r2lp_app_SDK_4.10/Startup\" --include_path=\"C:/Users/a0488519/workspace_v10/ble5_heart_rate_cc2640r2lp_app_SDK_4.10/PROFILES\" --include_path=\"C:/Users/a0488519/workspace_v10/ble5_heart_rate_cc2640r2lp_app_SDK_4.10/Include\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/controller/cc26xx/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/rom\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/common/cc26xx\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/common/cc26xx/menu\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/icall/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/target\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/hal/src/target/_common\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/hal/src/target/_common/cc26xx\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/hal/src/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/heapmgr\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/icall/src/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/osal/src/inc\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/services/src/saddr\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/ble5stack/services/src/sdata\" --include_path=\"C:/ti/simplelink_cc2640r2_sdk_4_10_00_10/source/ti/devices/cc26x0r2\" --include_path=\"C:/ti/ccs1000/ccs/tools/compiler/ti-cgt-arm_16.9.11.LTS/include\" --define=DeviceFamily_CC26X0R2 -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi " "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

configPkg/linker.cmd: build-927661265 ../TOOLS/app_ble.cfg
configPkg/compiler.opt: build-927661265
configPkg/: build-927661265


