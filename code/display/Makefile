TARGET:=display_driver
SRC=/lib/modules/$(shell uname -r)/build

obj-m := $(TARGET).o
all:
	$(MAKE) -C $(SRC)  M=$(PWD) modules

clean:
	$(MAKE) -C $(SRC) M=$(PWD) clean

load:
	./install.sh

unload:
	sudo rmmod $(TARGET)
	