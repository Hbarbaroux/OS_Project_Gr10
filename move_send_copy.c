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
#include "send_position.h"

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

const char const *color_list[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color_list ) / sizeof( color_list[ 0 ])))

#define M_PI 3.14159265358979323846

#define L_MOTOR_PORT      OUTPUT_C
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define IR_CHANNEL        0

#define SPEED_LINEAR      75  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR    50  /* ... for circular motion */

int max_speed;  /* Motor maximal speed */
int direction[]= {0,0,0,0};  /* save the direction of north east west  */

#define DEGREE_TO_COUNT( d )  (( d ) * 260 / 90 )

#define MOD(a,b) ((((a)%(b))+(b))%(b))

#define SERV_ADDR "30:E3:7A:10:9F:2C"


int app_alive;

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
int angle;    /* Angle of rotation */

uint8_t ir, touch;  /* Sequence numbers of sensors */
uint8_t sonar, color, compass;  /* Sequence  of sensors */
enum { L, R, M, S };
uint8_t motor[ 4 ] = { DESC_LIMIT, DESC_LIMIT, DESC_LIMIT, DESC_LIMIT };  /* Sequence numbers of motors */

typedef struct data {
    int var;
    pthread_mutex_t mutex;
} data;

static void _set_mode( int value )
{
	if ( value == MODE_AUTO ) {
		/* IR measuring of distance */
		set_sensor_mode_inx( ir, LEGO_EV3_IR_IR_PROX );
		mode = MODE_AUTO;
	} else {
		/* IR remote control */
		set_sensor_mode_inx( ir, LEGO_EV3_IR_IR_REMOTE );
		mode = MODE_REMOTE;
	}
}

static void _run_forever( int l_speed, int r_speed )
{
	set_tacho_speed_sp( motor[ L ], l_speed );
	set_tacho_speed_sp( motor[ R ], r_speed );
	multi_set_tacho_command_inx( motor, TACHO_RUN_FOREVER );
}

static void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos )
{
	set_tacho_speed_sp( motor[ L ], l_speed );
	set_tacho_speed_sp( motor[ R ], r_speed );
	set_tacho_position_sp( motor[ L ], l_pos );
	set_tacho_position_sp( motor[ R ], r_pos );
	multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
}

static void _run_timed( int l_speed, int r_speed, int ms )
{
	set_tacho_speed_sp( motor[ L ], l_speed );
	set_tacho_speed_sp( motor[ R ], r_speed );
	multi_set_tacho_time_sp( motor, ms );
	multi_set_tacho_command_inx( motor, TACHO_RUN_TIMED );
}

static int _is_running( void )
{
	FLAGS_T state = TACHO_STATE__NONE_;

	get_tacho_state_flags( motor[ L ], &state );
	if ( state != TACHO_STATE__NONE_ ) return ( 1 );
	get_tacho_state_flags( motor[ R ], &state );
	if ( state != TACHO_STATE__NONE_ ) return ( 1 );

	return ( 0 );
}

static void _stop( void )
{
	multi_set_tacho_command_inx( motor, TACHO_STOP );
}

int app_init( void )
{
	char s[ 16 ];

	if ( ev3_search_tacho_plugged_in( L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0 )) {
		get_tacho_max_speed( motor[ L ], &max_speed );
		/* Reset the motor */
		set_tacho_command_inx( motor[ L ], TACHO_RESET );
	} else {
		printf( "LEFT motor (%s) is NOT found.\n", ev3_port_name( L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s ));
		/* Inoperative without left motor */
		return ( 0 );
	}
	if ( ev3_search_tacho_plugged_in( R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0 )) {
		/* Reset the motor */
		set_tacho_command_inx( motor[ R ], TACHO_RESET );
	} else {
		printf( "RIGHT motor (%s) is NOT found.\n", ev3_port_name( R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s ));
		/* Inoperative without right motor */
		return ( 0 );
	}
	command	= moving = MOVE_NONE;

	if ( ev3_search_sensor( LEGO_EV3_IR, &ir, 0 )) {
		_set_mode( MODE_REMOTE );
	} else {
		printf( "IR sensor is NOT found.\n" );
		/* Inoperative without IR sensor */
		return ( 0 );
	}
	printf(	"IR remote control:\n"
	"RED UP & BLUE UP     - forward\n"
	"RED DOWN & BLUE DOWN - backward\n"
	"RED UP | BLUE DOWN   - left\n"
	"RED DOWN | BLUE UP   - right\n"
	"To switch between IR remote control and\n"
	"self-driving" );
	if ( ev3_search_sensor( LEGO_EV3_TOUCH, &touch, 0 )) {
		printf( " use the TOUCH sensor.\n" );
	} else {
		touch = DESC_LIMIT;
		printf( " press UP on the EV3 brick.\n" );
	}
	printf( "Press BACK on the EV3 brick for EXIT...\n" );
	return ( 1 );
}



double x,y;
int angle = 0;
int s;
char string[9];
int stop;
int angle_compass; /* value read from compass sensor */
int max_speed;  /* Motor maximal speed */
int back_arm_max_speed;

bool non_movable_encountered() {
	Sleep(10000);
	return true;
}

int there_is_obstacle()
{
  float us_value;
  get_sensor_value0(sonar, &us_value);
  // printf("%f\n", us_value);
  if ( us_value <=  100) 
    return 1;
  return 0;
}

int what_kind_of_obstacle() 
{
    int color_value;
    if ( !get_sensor_value( 0, color, &color_value ) || ( color_value < 0 ) || ( color_value >= COLOR_COUNT )) {
        color_value = 0;
    }
    printf( "\r(%s)\n", color_list[ color_value ]);
    if(color_value==5) return 1; /* RED : movable obstacle */
    else return 0; /* Non-movable obstacle */
}

int detection(){
	float distance;
	int obstacle_variable;
	//ultrasonic shows the distance to obstacle (float)
	get_sensor_value0(sonar, &distance);
	if(distance>100){
		obstacle_variable=0;
		}
		else{
			//wall
			if(what_kind_of_obstacle()==0){
			obstacle_variable=1;
			}
			//movable 
			else{obstacle_variable=2;}
	}
	return obstacle_variable; 
}

pthread_mutex_t mutexCompass = PTHREAD_MUTEX_INITIALIZER;

void turn_left() {
	multi_set_tacho_stop_action_inx( motor, TACHO_BRAKE );
	printf("turning left 90 start\n");

	pthread_mutex_lock (& mutexCompass );
	int original = angle_compass;
	pthread_mutex_unlock (& mutexCompass );

	_run_to_rel_pos(150,160,-150,-160);
	Sleep(1000);
	_stop();
	printf("11111111turning left 90 , previous= %d,   now=% d\n", original,angle_compass);
	_run_forever(10,-10);
	Sleep(10);
	if (original<90){

		//while( angle_compass < 90  || abs(angle_compass - original)>=270);
		while( angle_compass - original >= 270);
		_stop();
	}
	else{
		while( original - angle_compass <=89);
		_stop();
	}
	_stop();
	printf("turning left 90 , previous= %d,   now=% d\n", original,angle_compass);
}

void turn_right() {
	multi_set_tacho_stop_action_inx( motor, TACHO_BRAKE );

	printf("turning right 90 start\n");
	
	pthread_mutex_lock (& mutexCompass );
	int original = angle_compass;
	pthread_mutex_unlock (& mutexCompass );

	_run_to_rel_pos(-150,-160,150,160);
	Sleep(1000);
	_stop();
	printf("11111111turning right 90 , previous= %d,   now=% d\n", original,angle_compass);	
	_run_forever(-10,10);
	Sleep(10);
	if (original>270){
		//while( angle_compass >270 || abs(angle_compass - original)>=270);
		while( original - angle_compass >=270);
		_stop();
	}
	else{
		while( angle_compass - original <=89);
		_stop();
	}
	_stop();
	printf("turning right 90 , previous= %d,   now=% d\n", original,angle_compass);
}


void reset_coord() {
	x=0;
	y=0;
}

void _take_front_obstacle() {
	set_tacho_speed_sp( motor[S], -100 );
	//set_tacho_speed_sp( motor[ R ], r_speed );
	set_tacho_position_sp( motor[S], -160 );
	//set_tacho_position_sp( motor[ R ], r_pos );
	set_tacho_command_inx( motor[S], TACHO_RUN_TO_REL_POS );
	Sleep(500);

}

void _release_front_obstacle() {
	set_tacho_speed_sp( motor[S], 100 );
	//set_tacho_speed_sp( motor[ R ], r_speed );
	set_tacho_position_sp( motor[S], 170 );
	//set_tacho_position_sp( motor[ R ], r_pos );
	set_tacho_command_inx( motor[S], TACHO_RUN_TO_REL_POS );
	printf("release inside\n");
	Sleep(500);
}

void _release_obstacle(){
	set_tacho_speed_sp( motor[M], -180 );
	//set_tacho_speed_sp( motor[ R ], r_speed );
	set_tacho_position_sp( motor[M], -60 );
	//set_tacho_position_sp( motor[ R ], r_pos );
	set_tacho_command_inx( motor[M], TACHO_RUN_TO_REL_POS );
	Sleep(1200);

	set_tacho_speed_sp( motor[M], 180 );
	//set_tacho_speed_sp( motor[ R ], r_speed );
	set_tacho_position_sp( motor[M], 60 );
	//set_tacho_position_sp( motor[ R ], r_pos );
	set_tacho_command_inx( motor[M], TACHO_RUN_TO_REL_POS );
	printf("release inside\n");
	Sleep(1200);
}

void update_coord(double diff, int angle) {
	switch (angle) {
	case 0:
		x = x+diff;
		break;
	case 1:
		y = y+diff;
		break;
	case 2:
		x = x-diff;
		break;
	case 3:
		y = y-diff;
		break;
	}
}

///////////////////////
// THREADS FUNCTIONS //
///////////////////////

pthread_mutex_t mutexCoord = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexAng = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexStop = PTHREAD_MUTEX_INITIALIZER;

void *thread_set_coord(void *arg) {
    double diff;
    int speed1;
    int speed2;
    for (;;) {
    	Sleep(100);
    	get_tacho_speed_sp(motor[L],&speed1);
        get_tacho_speed_sp(motor[R],&speed2);
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
    }
    pthread_exit(NULL);
}

void *thread_read_compass(void *arg)
{    


	//generate the north south east and west(direction 0~3)
	pthread_mutex_lock (& mutexCompass );
    get_sensor_value( 0, compass, &angle_compass );
    direction[0]=angle_compass;
    //printf("\nangle= %d\n",angle_compass);
    pthread_mutex_unlock (& mutexCompass );
    direction[1]= (direction[0]+90) % 360;
    direction[2]= (direction[1]+90) % 360;
    direction[3]= (direction[2]+90) % 360;   
    //     
    for (;;) {
    	//Sleep(1);
    	pthread_mutex_lock (& mutexCompass );
    	get_sensor_value( 0, compass, &angle_compass );
    	//printf("\nangle= %d\n",angle_compass);
    	pthread_mutex_unlock (& mutexCompass );

    }
    pthread_exit(NULL);
}

int main( void )
{
    printf("Welcome to main, address of the server is %s \n", SERV_ADDR);
    struct sockaddr_rc addr = { 0 };
    int status;

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

	pthread_t threadSetCoord;
	pthread_t threadSendPosition;
	pthread_t threadReadServer;
	pthread_t threadReadCompass;
	
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

	reset_coord();
	printf("(%f,%f)\n",x,y);
	srand(time(NULL));
	// To the north: angle = 0, to the east: angle = 1, to the south: angle = 2, to the west: angle = 3

	int randomBit;

	if(ev3_search_sensor( LEGO_EV3_US, &sonar, 0 )) printf("%s\n", "US found");
	struct timespec tstart={0,0}, tend={0,0};
	int speed = 180;

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
    	printf( "HT_NXT_COMPASS sensor is found\n" );
    }

	printf( "start\n" );

	double diff_sec,diff_cm;
	int i;
	int temp;
	double diff;
	int realSpeed;
	set_sensor_mode(color, "COL-COLOR" );
	pthread_mutex_lock(&mutexStop);
	temp = stop;
	pthread_mutex_unlock(&mutexStop);

	
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
			_take_front_obstacle();
			turn_right();
			_release_front_obstacle();
			turn_left();
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

	printf("stop = %d", temp);
	_stop();
	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );
}
	return ( 0 );
}
