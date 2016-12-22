CC = arm-linux-gnueabi-gcc
CFLAGS = -Wall -Wextra -O2 -march=armv5te
LDFLAGS =
INCLUDES = -lpthread -lm -lrt
DEFINES  =  
OPTIMIZE =  
CFLAGS   = $(DEFINES)  $(OPTIMIZE) 

OBJS = exe_main

$(OBJS) : 
	$(CC) -static -o $@ $^ $(LDFLAGS) $(CFLAGS) $(INCLUDES)

clean:
	rm -f $(OBJS).o

exe_main : exe_main.o
exe_main.o : exe_main.c
 

