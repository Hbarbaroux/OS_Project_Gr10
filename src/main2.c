//////////////
// INCLUDES //
//////////////


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "bt_communication.h"
//#include "mapping_functions.h"
#include "move_functions.h"
#include "obstacle_detection.h"
#include "move_obstacle.h"

#ifndef MAPPING
#define MAPPING

#include "mapping_functions.h"

#endif



// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

//////////////////////
// GLOBAL VARIABLES //
//////////////////////

#define EMPTY 0
#define OBSTACLE_WALL 1
#define UNDEFINED 2

#define SMALL 0
#define BIG 1


#define MSG_ACK     0
#define MSG_START    1
#define MSG_STOP   2
#define MSG_KICK    3
#define MSG_POSITION 4
#define MSG_MAPDATA     5
#define MSG_MAPDONE 6
#define MSG_OBSTACLE 7
#define Sleep( msec ) usleep(( msec ) * 1000 )
#define TEAM_ID 10

#define M_PI 3.14159265358979323846

#define M_MOTOR_PORT      OUTPUT_A
#define M_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define S_MOTOR_PORT      OUTPUT_D
#define S_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define L_MOTOR_PORT      OUTPUT_C
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define IR_CHANNEL        0

#define SPEED_LINEAR      240
#define SPEED_CIRCULAR    100

extern const char const *color_list[];
#define COLOR_COUNT  (( int )( sizeof( color_list ) / sizeof( color_list[ 0 ])))

#define MOD(a,b) ((((a)%(b))+(b))%(b))

//#define SERV_ADDR "00:19:0E:10:72:CB"
#define SERV_ADDR "00:1A:7D:DA:71:06"
//#define SERV_ADDR "DC:53:60:AD:61:90"



#define RELEASING_TIME 10000
#define DETECTION_DISTANCE 60


int app_alive;
int max_speed;  /* Motor maximal speed */
int back_arm_max_speed;

enum { L , R };
enum { M , S };

enum {
	MODE_REMOTE,  /* IR remote control */
	MODE_AUTO,    /* Self-driving */
};

int mode;  /* Driving mode */

enum {
	MOVE_NONE,
	MOVE_FORWARD,
	MOVE_BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
	TURN_ANGLE,
	STEP_BACKWARD,
};

int moving;   /* Current moving */
int command;  /* Command for the 'drive' coroutine */

uint8_t compass, touch;  /* Sequence  of sensors */
extern uint8_t sonar, color, ir;


extern uint8_t motorLR[ 2 ], motorMS[ 2 ];

extern double x,y;

//extern struct pixel map[100][100];

int angle = 0;
int s;
char string[9];
int stop;
int act; /*Either 0 if obstacle release or 1 if obstacle picked */
int angle_compass; /* value read from compass sensor */
int direction[]= {0,0,0,0};
int distance_us;
int angle_us;
extern Pixel small_map[40][24];
extern Pixel big_map[78][78];
int map_type;
int cell_type = UNDEFINED;
int area;

pthread_mutex_t mutexCoord = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexAng = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexStop = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexCompass = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexCell = PTHREAD_MUTEX_INITIALIZER;



////////////////////
// INITIALIZATION //
////////////////////


int app_init( void )
{
	char s[ 16 ];

	if ( ev3_search_tacho_plugged_in( L_MOTOR_PORT, L_MOTOR_EXT_PORT, motorLR + L, 0 )) {
		get_tacho_max_speed( motorLR[ L ], &max_speed );
		/* Reset the motor */
		set_tacho_command_inx( motorLR[ L ], TACHO_RESET );
	} else {
		printf( "LEFT motor (%s) is NOT found.\n", ev3_port_name( L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s ));
		/* Inoperative without left motor */
		return ( 0 );
	}
	if ( ev3_search_tacho_plugged_in( R_MOTOR_PORT, R_MOTOR_EXT_PORT, motorLR + R, 0 )) {
		/* Reset the motor */
		set_tacho_command_inx( motorLR[ R ], TACHO_RESET );
	} else {
		printf( "RIGHT motor (%s) is NOT found.\n", ev3_port_name( R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s ));
		/* Inoperative without right motor */
		return ( 0 );
	}

	 if ( ev3_search_tacho_plugged_in( M_MOTOR_PORT, M_MOTOR_EXT_PORT, motorMS + M, 0 )) {
                get_tacho_max_speed( motorMS[ M ], &back_arm_max_speed );
                /* Reset the motor */
                set_tacho_command_inx( motorMS[ M ], TACHO_RESET );
        } else {
                printf( "M motor (%s) is NOT found.\n", ev3_port_name( M_MOTOR_PORT, M_MOTOR_EXT_PORT, 0, s ));
                /* Inoperative without left motor */
                return ( 0 );
        }

	 if ( ev3_search_tacho_plugged_in( S_MOTOR_PORT, S_MOTOR_EXT_PORT, motorMS + S, 0 )) {
                //get_tacho_max_speed( motor[ S ], &max_speed );
                /* Reset the motor */
                set_tacho_command_inx( motorMS[ S ], TACHO_RESET );
        } else {
                printf( "S motor (%s) is NOT found.\n", ev3_port_name( S_MOTOR_PORT, S_MOTOR_EXT_PORT, 0, s ));
                /* Inoperative without left motor */
                return ( 0 );
        }


	command	= moving = MOVE_NONE;

	multi_set_tacho_stop_action_inx(motorLR, TACHO_BRAKE);
	multi_set_tacho_stop_action_inx(motorMS, TACHO_BRAKE);
	

	printf( "Press BACK on the EV3 brick for EXIT...\n" );
	return ( 1 );
}



//////////////////////
// USEFUL FUNCTIONS //
//////////////////////



///////////////////////
// THREADS FUNCTIONS //
///////////////////////


void *thread_non_movable(void *arg) {
	Sleep(60000);
	_release_obstacle();
	pthread_mutex_lock(&mutexCompass);
	int obstacle_angle = angle_compass;
	pthread_mutex_unlock(&mutexCompass);

	int obstacle_angle_bis = (double)((direction[0]-obstacle_angle)%360)*M_PI/180.0;
	double diffx = -30.0*sin(obstacle_angle_bis);
	double diffy = 30.0*cos(obstacle_angle_bis);

	pthread_mutex_lock(&mutexCoord);
	send_obstacle((x-diffx),(y+diffy),0);
	pthread_mutex_unlock(&mutexCoord);

	pthread_exit(NULL);
}	

/*
void *thread_set_coord(void *arg) {
    double diff;
    int speed1;
    int speed2;
    for (;;) {
    	Sleep(100);
    	get_tacho_speed_sp(motorLR[L],&speed1);
        get_tacho_speed_sp(motorLR[R],&speed2);
        if ((speed1 != 0) && (speed1 == speed2)) {
    	    diff = (0.1)*((double)speed1*M_PI*5.5/360.0+1);
            pthread_mutex_lock (&mutexCoord);
            pthread_mutex_lock (&mutexAng);
            update_coord(diff,angle);
            pthread_mutex_unlock (&mutexAng);
            pthread_mutex_unlock (&mutexCoord);
        }
    }
    pthread_exit(NULL);
}*/

// Thread to be used with update_coord_compass

void *thread_set_coord(void *arg) {
    double diff;
    int speed1;
    int speed2;
	int angle_before;
	int angle_after;
    for (;;) {
        pthread_mutex_lock (& mutexCompass );
        angle_before = angle_compass;
        pthread_mutex_unlock (& mutexCompass );
        Sleep(100);
        pthread_mutex_lock (& mutexCompass );
        angle_after = angle_compass;
        pthread_mutex_unlock (& mutexCompass );
        get_tacho_speed_sp(motorLR[L],&speed1);
        get_tacho_speed_sp(motorLR[R],&speed2);
        if (((speed1 == SPEED_LINEAR) || (speed1 == -SPEED_LINEAR)) && (speed1 == speed2)) {
            diff = (0.1)*((double)speed1*M_PI*5.5/360.0);
            pthread_mutex_lock (&mutexCoord);
            pthread_mutex_lock (&mutexAng);
            update_coord_compass(diff, (double) angle_after);
            pthread_mutex_unlock (&mutexAng);
            pthread_mutex_unlock (&mutexCoord);
        }
    }
    pthread_exit(NULL);
}

void *thread_send_position(void *arg)
{
    for(;;){
		Sleep(2000);
		pthread_mutex_lock (&mutexCoord);
    	printf("[%f,%f]\n",x,y);
		send_position(x,y);
		pthread_mutex_unlock (&mutexCoord);
    }
    pthread_exit(NULL);
}

void *thread_read_server(void *arg){
    for(;;){
		read_from_server(s, string, 9);
		if(string[4]==MSG_STOP){
	 		pthread_mutex_lock(&mutexStop);
			stop=1;
			pthread_mutex_unlock(&mutexStop);
			printf("Received stop message !\n");
			close(s);
			break;
		}
		
		if(string[4]==MSG_KICK && string[5]==TEAM_ID){
	 		pthread_mutex_lock(&mutexStop);
			stop=1;
			pthread_mutex_unlock(&mutexStop);
			printf("Got kicked out of the game !\n");
			close(s);
			break;
		}
    }
    pthread_exit(NULL);
}

/*void *thread_read_server(void *arg){
    char c;
    for(;;){
        c=getch();
	printf("**** Char typed !! **** : %c\n", c);
        if (c == 'r') {
            pthread_mutex_lock(&mutexStop);
            stop=1;
            pthread_mutex_unlock(&mutexStop);
            break;
        }
    }
	printf("end\n");
    printf("%c", c);
    pthread_exit(NULL);
}*/

void *thread_read_compass(void *arg) {   
	//generate the north south east and west(direction 0~3)
	pthread_mutex_lock (& mutexCompass );
    get_sensor_value( 0, compass, &angle_compass );
    direction[0]=angle_compass;
    pthread_mutex_unlock (& mutexCompass );
    direction[1]= (direction[0]+90) % 360;
    direction[2]= (direction[1]+90) % 360;
    direction[3]= (direction[2]+90) % 360;       
    for (;;) {
    	pthread_mutex_lock (& mutexCompass );
    	get_sensor_value( 0, compass, &angle_compass );
    	pthread_mutex_unlock (& mutexCompass );
    }
    pthread_exit(NULL);
}

void *thread_get_us(void *angle_final) {
	
	printf("Entering thread for distance\n");
	int value_final = *((int *)angle_final);
	int myangle;
	float distance;
	
	angle_us = 400;
	distance_us = 1000;

	pthread_mutex_lock(&mutexCompass);
	myangle = angle_compass;
	pthread_mutex_unlock(&mutexCompass);
	
	while (myangle != value_final) {
		get_sensor_value0(sonar, &distance);
		if (distance_us > distance) {
			distance_us = (int)distance;
			angle_us = myangle;
		}
		pthread_mutex_lock(&mutexCompass);
        	myangle = angle_compass;
        	pthread_mutex_unlock(&mutexCompass);
	}
	
	printf("Exiting thread for distance\n");
	pthread_exit(NULL);
}


/*void *thread_draw_map(void *arg) {
	if (map_type == SMALL) {
		for(;;) {
			Sleep(700);
			pthread_mutex_lock (& mutexCoord);
			pthread_mutex_lock (& mutexCell);
			draw_map(40, 24, small_map,y,x,cell_type);
			pthread_mutex_unlock (& mutexCoord);
			pthread_mutex_unlock (& mutexCell);
		}
	}
	else {
		for(;;) {
                        Sleep(700);
                        pthread_mutex_lock (& mutexCoord);
                        pthread_mutex_lock (& mutexCell);
                        draw_map(100, 100, big_map,y,x,cell_type);
                        pthread_mutex_unlock (& mutexCoord);
                        pthread_mutex_unlock (& mutexCell);
                }
	}
}*/


void *thread_draw_map (void *arg) {
	Sleep(220000);
	if (map_type == SMALL) {
		send_map(0);
		Sleep(1000);
		send_map_done();
	}
	else {
		
		send_map(1);
                Sleep(1000);
                send_map_done();

	}
}


void move_little() {
    _run_timed(SPEED_LINEAR,SPEED_LINEAR, 250000.0/(double)SPEED_LINEAR);
}

int scan_arena() {
    
}

void turn_angle_detected(int angle) {
    
}

void turn_random() {
    
}

int getch(void) {
    int c=0;
    
    struct termios org_opts, new_opts;
    int res=0;
    //-----  store old settings -----------
    res=tcgetattr(STDIN_FILENO, &org_opts);
    assert(res==0);
    //---- set new terminal parms --------
    memcpy(&new_opts, &org_opts, sizeof(new_opts));
    new_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOKE | ICRNL);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_opts);
    c=getchar();
    //------  restore old settings ---------
    res=tcsetattr(STDIN_FILENO, TCSANOW, &org_opts);
    assert(res==0);
    return(c);
}

/*void display(struct pair *obstacles, int index) {
	printf("***** OBSTACLES *****\n");
	int i;
	for (i = 0 ; i < index ; i++) {
		printf("(%d,%d)\n", obstacles[i].detected_angle, obstacles[i].detected_distance);
	}
	printf("*********************\n");
}*/

///////////////////
// MAIN FUNCTION //
///////////////////


int main( int argc, char **argv )
{

	// DECLARATIONS //

	set_sensor_mode(color, "COL-COLOR");
	int randomBit;
	struct timespec tstart={0,0}, tend={0,0};
	double diff_sec,diff_cm;
	int i;
	int realSpeed;
    double diff;
    struct sockaddr_rc addr = { 0 };
    int status;
    int temp;
	int speed = 240;
	map_type = atoi(argv[1]);
	printf("### MAP_TYPE ### %d\n", map_type);
	area = atoi(argv[2]);
	printf("### AREA ### %d\n", area);
	int compt = 0;
	

	initialize_small_map(40,24,small_map);
	//////////////////



	// BLUETOOTH INITIALIZATION //
    
    printf("Welcome to main, address of the server is %s \n", SERV_ADDR);
    
    /* allocate a socket */
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    
    /* set the connection parameters (who to connect to) */
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba (SERV_ADDR, &addr.rc_bdaddr);
    
    /* connect to server */
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
    printf("status  = %d \n",status);
    
    /* if connected */
    if( status == 0 ) {
        /* Wait for START message */
        read_from_server (s, string, 9);
        if(string[4] == MSG_START) {
            printf ("Received start message!\n");
        }
    }
    ////////////////////////////

	/*char c = getch();
    printf("*** First char typed *** : %c\n",c);	
*/
    // INITIALIZATION OF THE THREADS //

	pthread_t threadGetUS;
	pthread_t threadSetCoord;
	pthread_t threadSendPosition;
	pthread_t threadReadServer;
	pthread_t threadReadCompass;
	pthread_t threadNonMovable;
	pthread_t threadDrawMap;


	if (pthread_create(&threadDrawMap, NULL, thread_draw_map, NULL)) {
        perror("pthread_create for thread_draw_map\n");
        return EXIT_FAILURE;
    }


	if (pthread_create(&threadNonMovable, NULL, thread_non_movable, NULL)) {
        perror("pthread_create for thread_non_movable\n");
        return EXIT_FAILURE;
    }
	
	if (pthread_create(&threadSendPosition, NULL, thread_send_position, NULL)) {
    	perror("pthread_create for thread_send_position\n");
    	return EXIT_FAILURE;
    }

  	if (pthread_create(&threadSetCoord, NULL, thread_set_coord, NULL)) {
  		perror("pthread_create for thread_set_coord\n");
   		return EXIT_FAILURE;
   	}
	
	if (pthread_create(&threadReadServer, NULL, thread_read_server, NULL)) {
        perror("pthread_create for thread_read_server\n");
        return EXIT_FAILURE;
    }

	if (pthread_create(&threadReadCompass, NULL, thread_read_compass, NULL)) {
    	perror("pthread_create for thread_read_compass\n");
    	return EXIT_FAILURE;
    } 

    //////////////////////////////////



	// INITIALIZATION OF THE ROBOT //

	reset_coord(map_type, area);
	printf("(%f,%f)\n",x,y);
	srand(time(NULL));

	printf( "Waiting the EV3 brick online...\n" );
	if ( ev3_init() < 1 ) return ( 1 );

	printf( "*** ( EV3 ) Hello! ***\n" );

	ev3_sensor_init();
	ev3_tacho_init();

	app_alive = app_init();

	if ( !ev3_search_sensor( LEGO_EV3_US, &sonar, 0 )) {
		printf("## Sonar not found\n");
	}

	if ( !ev3_search_sensor( LEGO_EV3_COLOR, &color, 0 )) {
        printf("## Color not found\n");
    }
    if ( ev3_search_sensor( HT_NXT_COMPASS, &compass, 0 )) {
    	printf( "## HT_NXT_COMPASS sensor is found\n" );
    }

	printf( "start\n" );

	pthread_mutex_lock(&mutexStop);
	temp = stop;
	pthread_mutex_unlock(&mutexStop);

	/////////////////////////////////



	// MAIN LOOP //

    
    //struct pair *obstacles;
    int index = 0;
	Pair pair1;
	Pair pair2;
	int aim_angle;
	int aim_distance;
	int obstacle_angle;
	double obstacle_angle_bis;    
	double diffx;
	double diffy;
	int angle_final;
	int diff_angle = 70;
	int original_angle;
	int myY;

    while (!temp) {

	while (!temp) {

		if (map_type == BIG) {
			pthread_mutex_lock(&mutexCoord);
			myY = y;
			pthread_mutex_unlock(&mutexCoord);
			if (y<0) {
				turn_to_certain_direction(direction[0]);
			}
		}	
		//index = searching_obstacle_right_array(obstacles, index);
		//index = searching_obstacle_left_array(obstacles, index);
		
		pthread_mutex_lock(&mutexCompass);
		original_angle = angle_compass;
		pthread_mutex_unlock(&mutexCompass);
		angle_final = MOD((diff_angle+original_angle),360);
		
		printf("ORIGINAL ANGLE : %d\n", original_angle);		

		if (pthread_create(&threadGetUS, NULL, thread_get_us, (void *)&angle_final)) {
        		perror("pthread_create for thread_non_movable\n");
        		return EXIT_FAILURE;
    		}

		//turn_to_certain_direction(angle_final);
		_run_to_rel_pos(100, 150, -100, -150);
		Sleep(2000);
		_stop();
		pair1.detected_angle = angle_us;
                pair1.detected_distance = distance_us;
		turn_to_certain_direction(original_angle);
		_stop();
		
		printf("after turning right\n");
		angle_final = MOD((original_angle-diff_angle),360);

		if (pthread_create(&threadGetUS, NULL, thread_get_us, (void *)&angle_final)) {
                        perror("pthread_create for thread_non_movable\n");
                        return EXIT_FAILURE;
                }

		//turn_to_certain_direction(angle_final);
                _run_to_rel_pos(-100, -150, 100, 150);
		Sleep(2000);
		_stop();
		pair2.detected_angle = angle_us;
		pair2.detected_distance = distance_us;
                turn_to_certain_direction(original_angle);
                _stop();

		printf("after turning left\n");

		if (pair1.detected_distance < pair2.detected_distance) {
			aim_angle = pair1.detected_angle;
			aim_distance = pair1.detected_distance;
		}
		else {
			aim_angle = pair2.detected_angle;
			aim_distance = pair2.detected_distance;
		}
		
		printf("aimdistance : %d", aim_distance);
	
		if (aim_distance < 300) {
			turn_to_certain_direction(aim_angle);
			_stop();
			_run_forever(SPEED_LINEAR, SPEED_LINEAR);
			while(!there_is_obstacle());
			_stop();
			Sleep(500);

			
			pthread_mutex_lock(&mutexCompass);
			obstacle_angle = angle_compass;
			pthread_mutex_unlock(&mutexCompass);
			
			obstacle_angle_bis = (double)(MOD((int)(direction[0]-obstacle_angle),360))*M_PI/180.0;
			diffx = -12.0*sin(obstacle_angle_bis);
			diffy = 12.0*cos(obstacle_angle_bis);


			if (what_kind_of_obstacle()) {
				act = 1;
			_stop();
			Sleep(2000);
			_take_front_obstacle();
			send_obstacle(x+diffx,y+diffy,act);
			act = 0;
			Sleep(2000);
			_stop();
			Sleep(2000);
			turn_right();
			while(_is_running());
			_stop();
			Sleep(2000);
			_release_front_obstacle();
			diffx = -12.0*sin(obstacle_angle_bis-M_PI/2.0);
			diffy = 12.0*cos(obstacle_angle_bis-M_PI/2.0);
			send_obstacle(x+diffx,y+diffy,act);
			Sleep(2000);
			_stop();
			Sleep(2000);
			turn_left();
			while(_is_running());
			//pthread_mutex_lock(& mutexCell);
			//cell_type = EMPTY;
			//pthread_mutex_unlock(& mutexCell);
			_stop();
			}
			else {
				if (map_type == SMALL) {
					if (!((y>195)||(y<5)||(x>115)||(x<5))) {
						pthread_mutex_lock(&mutexCoord);
						printf("\n *** OBSTACLE FOUND : (%f,%f)\n\n", (x+diffx), (y+diffy));
						draw_map(40,24,small_map,(int) ((x+diffx)/5.0),(int) ((y+diffy)/5.0), OBSTACLE_WALL);
						pthread_mutex_unlock(&mutexCoord);
					}
				}
				else {
					pthread_mutex_lock(&mutexCoord);
                                                printf("\n *** OBSTACLE FOUND : (%f,%f)\n\n", (x+diffx), (y+diffy));
                                                draw_map(78, 78, big_map,(int) ((x+diffx)/5.0),(int) ((y+diffy)/5.0), OBSTACLE_WALL);
                                                pthread_mutex_unlock(&mutexCoord);
				}
			_run_to_rel_pos(-SPEED_LINEAR, -200, -SPEED_LINEAR, -200);
			Sleep(4000);
			_stop();
			if (compt == 0) {
				turn_right_certain_degree(80);
				compt = 1;
			}
			else {
				turn_left_certain_degree(80);
				compt = 0;
			}
			_stop();
			}
		}
		else {
			_run_timed(SPEED_LINEAR, SPEED_LINEAR,360000.0/(double)SPEED_LINEAR);
			Sleep(4000);
			_stop();
		}
		
		if (map_type == BIG) {
			print_map(78, 78, big_map, BIG);
		}
		else {
			print_map(40, 24, small_map, SMALL);
		}

		pthread_mutex_lock(&mutexStop);
        	temp = stop;
        	pthread_mutex_unlock(&mutexStop);
	}
	printf("STOPPED\n");
	_stop();
	//display(obstacles, index);
	
	

       /* if (direction == 400) {
            move_little();
            while (!there_is_obstacle(6cm));
            _stop();
            if (there_is_obstacle(6cm)) {
                Sleep(3000);
            }
        }
        else {
            turn_angle_detected(direction);
            _run_forever(SPEED_LINEAR,SPEED_LINEAR);
            while (!there_is_obstacle(6cm));
            _stop();
            
            if (there_is_obstacle() && what_kind_of_obstacle()) {
                act = 1;
                _stop();
                Sleep(2000);
                _take_front_obstacle();
                send_obstacle(x,y,act);
                act = 0;
                Sleep(2000);
                _stop();
                Sleep(2000);
                turn_right();
                while(_is_running());
                _stop();
                Sleep(2000);
                _release_front_obstacle();
                send_obstacle(x,y,act);
                Sleep(2000);
                _stop();
                Sleep(2000);
                turn_left();
                while(_is_running());
                _stop();
                Sleep(2000);
                move_little();
            }
            else {
                update_map(wall);
                _run_timed(SPEED_LINEAR,SPEED_LINEAR, 330000.0/(double)SPEED_LINEAR);
                turn_random();
            }
        }*/


        pthread_mutex_lock(&mutexStop);
        temp = stop;
        pthread_mutex_unlock(&mutexStop);
    }



	// UNINIT //

	printf("stop = %d", temp);
	_stop();
	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );

	////////////


	return ( 0 );
}
