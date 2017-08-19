PROJECT=$(BLE_REPO)/examples/rtos/CC2640R2_LAUNCHXL/blestack/ancs/tirtos/iar
BLE_REPO=../../../../../../..
IAR_PATH=C:/Program\ Files\ \(x86\)/IAR\ Systems/EWARM-$(IAR_VERSION)/common/bin
IAR_VERSION=8.11.1

all: ancs_src

release: clean ancs

ancs:
	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack.ewp \
	-build \
	FlashROM_Library \
	-log info \
	-parallel 1 \
	-varfile ancs.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app.ewp \
	-build \
	FlashROM_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ancs.custom_argvars

ancs_src:
	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack_src.ewp \
	-build \
	FlashOnly_Library \
	-log info \
	-parallel 1 \
	-varfile ancs_src.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app_src.ewp \
	-build \
	FlashOnly_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ancs_src.custom_argvars

clean:
	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack.ewp \
	-clean \
	FlashROM_Library \
	-log info \
	-parallel 1 \
	-varfile ancs.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app.ewp \
	-clean \
	FlashROM_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ancs.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/stack/cc2640r2lp_stack_src.ewp \
	-clean \
	FlashOnly_Library \
	-log info \
	-parallel 1 \
	-varfile ancs_src.custom_argvars

	$(IAR_PATH)/iarbuild.exe $(PROJECT)/app/cc2640r2lp_app_src.ewp \
	-clean \
	FlashOnly_StackLibrary \
	-log info \
	-parallel 1 \
	-varfile ancs_src.custom_argvars
