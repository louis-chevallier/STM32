
start :
	make -j8 all -C ../../Debug
	STM32_Programmer_CLI -q -vb 2 -c port=SWD freq=4000 -w ../../Debug/beam02.elf
	STM32_Programmer_CLI -q -vb 2 -c port=SWD -swv freq=4000 portnumber=0 -RA

