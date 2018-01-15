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


typedef struct pair {
	int detected_angle;
	int detected_distance;
} Pair;
