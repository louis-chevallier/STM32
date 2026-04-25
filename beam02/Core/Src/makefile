
start :
	make clean  -C ../../Debug
	date
	-which arm-none-eabi-gcc 
	-make  all -C ../../Debug
	echo compile done

	which STM32_Programmer_CLI
	STM32_Programmer_CLI -q -vb 2 -c port=SWD freq=4000 -w ../../Debug/beam02.elf
#	STM32_Programmer_CLI -q -vb 2 -c port=SWD -swv freq=4000 portnumber=0 -RA 	
	STM32_Programmer_CLI -q -vb 2 -c port=SWD  -startswv freq=4000 portnumber=0 -RA 

