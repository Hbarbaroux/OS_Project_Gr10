CC=gcc
CFLAGS=
LFLAGS= -I/home/robot/src/headers -L/home/robot/ev3dev-c/lib -lev3dev-c -lm -pthread 

%.o : %.c
	$(CC) $(CFLAGS) $(LFLAGS) -c $< -o $@

all : main

main : main.o bt_communication.o move_functions.o mapping_functions.o obstacle_detection.o move_obstacle.o
	$(CC) $(CFLAGS) $(LFLAGS) $^ -o $@


clean : 
	rm -rf *.o main

 
