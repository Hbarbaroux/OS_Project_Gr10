 void _set_mode( int value );

 void _run_forever( int l_speed, int r_speed );

 void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos );

 void _run_timed( int l_speed, int r_speed, int ms );

 int _is_running( void );

 void _stop( void );

void turn_to_certain_direction (int direc);

void turn_right_certain_degree(int degree);

void turn_left_certain_degree(int degree);

void turn_left(void);

void turn_right(void);

int searching_obstacle_right_array(Pair *obstacles, int index); //Made by Pawel & modified by Hugo.
	//Both these functions (this one and the one below) are used to look around for searching obstacles. It turns for and scans by ultrasonic sensor and returns the closest values. After reaching the max angle it goes back to initial position.
int searching_obstacle_left_array(Pair *obstacles, int index); // Made by Pawel & modified by Hugo.
	


typedef struct pair {
	int detected_angle;
	int detected_distance;
} Pair;
