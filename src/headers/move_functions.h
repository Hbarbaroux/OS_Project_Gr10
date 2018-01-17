//this part is responsible by Jianzhong LIANG
//used to perceive the current position and make next movement accurately, in charge of the use of motor for wheel and the compass sensor.
void _set_mode( int value );

 void _run_forever( int l_speed, int r_speed );

 void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos );

 void _run_timed( int l_speed, int r_speed, int ms );

 int _is_running( void );

 void _stop( void );

void turn_to_certain_direction (int direc);// by Jianzhong, implement the accurate rotation.
// accurate rotate with the accuracy of +-1 degree, implement the feedback to keep robot heading to certain degree excatly.

void keep_go_straight (int distance);// by Jianzhong

void turn_right_certain_degree(int degree);// by Jianzhong 
//for accurately turning, some tuning for general utilization, some trade off in adjust time and speed.

void turn_left_certain_degree(int degree);// by Jianzhong
//for accurately turning

void turn_left(int degree);
// by Jianzhong, tuning version for scaning version

void turn_right(int degree);
// by Jianzhong, tuning version for scaning version


int searching_obstacle_right_array(Pair *obstacles, int index); //Made by Pawel & modified by Hugo.
	//Both these functions (this one and the one below) are used to look around for searching obstacles. It turns for and scans by ultrasonic sensor and returns the closest values. After reaching the max angle it goes back to initial position.
int searching_obstacle_left_array(Pair *obstacles, int index); // Made by Pawel & modified by Hugo.
	


typedef struct pair {
	int detected_angle;
	int detected_distance;
} Pair;
