#include <string.h>
#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

static bool _check_yellow( uint8_t sn )
{
	int val;

	if ( sn == SENSOR__NONE_ ) {
		return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
	}
	return ( get_sensor_value( 0, sn, &val ) && ( val == 4 ));
}

int main( void )
{
	char s[ 256 ];
	int val;
	uint32_t n, i, ii;
	uint8_t sn_color;

	printf( "Waiting the EV3 brick online...\n" );
	if ( ev3_init() < 1 ) return ( 1 );

	printf( "*** ( EV3 ) Hello! ***\n" );
	ev3_sensor_init();

  if ( ev3_search_sensor( LEGO_EV3_COLOR, &sn_color, 0 )) {
  		printf( "COLOR sensor is found, reading COLOR..., read YELLOW to exit\n" );
  		set_sensor_mode( sn_color, "COL-COLOR" );
  		for ( ; ; ) {
  			if ( !get_sensor_value( 0, sn_color, &val ) || ( val < 0 ) || ( val >= COLOR_COUNT )) {
  				val = 0;
  			}
  			printf( "\r(%s)", color[ val ]);
  			fflush( stdout );
  			if ( _check_yellow( sn_color )) break;
  			Sleep( 200 );
  			printf( "\r        " );
  			fflush( stdout );
  			if ( _check_yellow( sn_color )) break;
  			Sleep( 200 );
  		}
  	} else {
  		printf( "COLOR sensor is NOT found\n" );
  		while ( !_check_yellow( sn_color )) Sleep( 100 );
  	}
    ev3_uninit();
    	printf( "\n*** ( EV3 ) Bye! ***\n" );

    	return ( 0 );
}
