TARGET:=ws2812_uart_driver
SRC=/lib/modules/$(shell uname -r)/build

obj-m := ws2812_uart_driver.o
all:
	$(MAKE) -C $(SRC)  M=$(PWD) modules

clean:
	$(MAKE) -C $(SRC) M=$(PWD) clean

load:
	sudo insmod $(TARGET).ko

unload:
	sudo rmmod $(TARGET)
	
