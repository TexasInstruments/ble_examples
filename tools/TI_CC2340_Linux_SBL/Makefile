.Default_GOAL := main.c
main.c:
	gcc -o ./Target/SBL.out ./Source/main.c ./Source/Linux_Serial.c ./Source/myFile.c Source/sbl_device_cc2340.c Source/sbl_device.c
run:
	./Target/SBL.out /dev/ttyACM0 ./binfiles/fw_full.bin ./binfiles/ccfg.bin

clean:	
	rm -rf ./Target/*.out
	
