CC = arm-linux-gnueabihf-gcc
INCLUDE= 
#LIBS   = -lssl -lcrypto -lpaho-mqtt3as -lpaho-mqtt3c
#LIB_PATH= 
CFLAGS = -Wall 
OBJS   = master_test.o modbusCRC.o modbus_master.o serial_drive.o 
TARGET =  master_test.exe 

all:$(TARGET)

$(TARGET):$(OBJS) 
	$(CC) -o $@ $^
 
 %.o:%.c
	$(CC) $(CFLAGS) -c $< -o $@  -DLOG 
.PHONY: all clean cleanobject
cleanobject:
	rm -rf *.o
clean: 
	rm -rf *.exe *.o 

