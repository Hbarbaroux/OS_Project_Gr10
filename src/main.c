//////////////
// INCLUDES //
//////////////


#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "communication.h"
#include "motor.h"

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

#define SPEED_LINEAR      180
#define SPEED_CIRCULAR    100

const char const *color_list[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color_list ) / sizeof( color_list[ 0 ])))

#define MOD(a,b) ((((a)%(b))+(b))%(b))

#define SERV_ADDR "30:E3:7A:10:9F:28"

#define RELEASING_TIME 10000
#define DETECTION_DISTANCE 100

int app_alive;
int max_speed;  /* Motor maximal speed */
int back_arm_max_speed;

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
extern uint8_t sonar, color, ir


extern uint8_t motorLR[ 2 ], motorMS[ 2 ];

extern double x,y;
extern struct pixel;
extern pixel map[100][100];

int angle = 0;
int s;
char string[9];
int stop;
int act; /*Either 0 if obstacle release or 1 if obstacle picked*/
int angle_compass; /* value read from compass sensor */
int direction[]= {0,0,0,0};


pthread_mutex_t mutexCoord = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexAng = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexStop = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexCompass = PTHREAD_MUTEX_INITIALIZER;


}


////////////////////
// INITIALIZATION //
////////////////////


int app_init( void )
{
	char s[ 16 ];

	if ( ev3_search_tacho_plugged_in( L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0 )) {
		get_tacho_max_speed( motorLR[ L ], &max_speed );
		/* Reset the motor */
		set_tacho_command_inx( motorLR[ L ], TACHO_RESET );
	} else {
		printf( "LEFT motor (%s) is NOT found.\n", ev3_port_name( L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s ));
		/* Inoperative without left motor */
		return ( 0 );
	}
	if ( ev3_search_tacho_plugged_in( R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0 )) {
		/* Reset the motor */
		set_tacho_command_inx( motorLR[ R ], TACHO_RESET );
	} else {
		printf( "RIGHT motor (%s) is NOT found.\n", ev3_port_name( R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s ));
		/* Inoperative without right motor */
		return ( 0 );
	}

	 if ( ev3_search_tacho_plugged_in( M_MOTOR_PORT, M_MOTOR_EXT_PORT, motor + M, 0 )) {
                get_tacho_max_speed( motorMS[ M ], &back_arm_max_speed );
                /* Reset the motor */
                set_tacho_command_inx( motorMS[ M ], TACHO_RESET );
        } else {
                printf( "M motor (%s) is NOT found.\n", ev3_port_name( M_MOTOR_PORT, M_MOTOR_EXT_PORT, 0, s ));
                /* Inoperative without left motor */
                return ( 0 );
        }

	 if ( ev3_search_tacho_plugged_in( S_MOTOR_PORT, S_MOTOR_EXT_PORT, motor + S, 0 )) {
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
	Sleep(20000);
	_release_obstacle();
	pthread_exit(NULL);
}	

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



///////////////////
// MAIN FUNCTION //
///////////////////


int main( void )
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
	int speed = 180;

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



    // INITIALIZATION OF THE THREADS //

	pthread_t threadSetCoord;
	pthread_t threadSendPosition;
	pthread_t threadReadServer;
	pthread_t threadReadCompass;
	pthread_t threadNonMovable;

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

	reset_coord();
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

	while (!temp) {
        

		// TEST FOR N/S OR E/W DIRECTIONS //

		if (angle % 2 == 0) {
        	_run_forever(speed,speed);
			while (!there_is_obstacle());
            _stop();
		}

		else {
			clock_gettime(CLOCK_MONOTONIC, &tstart);
			_run_timed(speed,speed, 500000.0/(double)speed);
			printf("%f\n",500000.0/(double)speed);
			clock_gettime(CLOCK_MONOTONIC, &tend);
			diff = ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
			while ((!there_is_obstacle()) && (diff < 500.0/(double)speed)) {
				clock_gettime(CLOCK_MONOTONIC, &tend);
				diff = ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec);
				//printf("%f,\n",diff);
			}
			printf("1\n");
			_stop();
		}

		///////////////////////////////////



		// TEST FOR OBSTACLE DETECTION //
		
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
		}
		else {
			randomBit = rand() % 2;
			pthread_mutex_lock (&mutexAng);
			if (randomBit == 0) {
				turn_left();
				angle = MOD(angle-1,4);
			}
			else {
				turn_right();
				angle = MOD(angle+1,4);
			}
			pthread_mutex_unlock (&mutexAng);
			Sleep(3000);
		}

		/////////////////////////////////
        
        pthread_mutex_lock(&mutexStop);
        temp = stop;
        pthread_mutex_unlock(&mutexStop);
	}

	////////////////



	// UNINIT //

	printf("stop = %d", temp);
	_stop();
	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );

	////////////


	return ( 0 );
}
