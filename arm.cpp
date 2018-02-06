/*
 * SCIOLY program for RobotARM using Bluetooth library.
 * Compile: use build.sh
 *
 * Sets up the PS3 remote via /dev/input/js0. Waits for the js0 device to become
 * available.
 * Setup the PS4 remote by first running 'ds4drv' the DualShock 4 driver.
 * If
 * Reads controller every 10ms and prints the buttons that are pressed to the console.
 * pressing 'select' on the controller exits this code.
 * 
 * Currently supports eight motors for soft PWM driving motors via external
 * H-Bridge.
 *
 */
 
#include <wiringPi.h> 
#include <softPwm.h>
#include <stdio.h> 
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define JOY_DEV "/dev/input/js0"

// pin states at power up
// group 1: L/R 1/1
// group 2: L/R 0/1
// group 3: L/R 1/1
// group 4: L/R 0/0
// group 5: L/R 0/0
// group 6: L/R 1/1
// group 7: L/R 0/0
// group 8: L/R 0/1

// cc change group 2 and 8 to L/R=0/0
// 0/0 is stop or 1/1 is break should be OK for now
 



// group 2: pin 7,11 OK
// group 3: pin 8,10 OK
// group 4: pin 13,15 OK
// group 5: pin 12,16 OK
// group 6: not GPIO ... not used for motor signal
// group 7: pin 18,22 ?? NG
// group 8: pin 31,33 OK 
 
#define READDELAYMS 10
#define state_REV 2
#define state_FW 1
#define state_STOP 0

//
// define IO's for motor drives:
//
#define	GROUP1_IN1	3
#define	GROUP1_IN2 	5

#define	GROUP2_IN1	7
#define	GROUP2_IN2 	11

#define	GROUP3_IN1	8
#define	GROUP3_IN2 	10

#define	GROUP4_IN1	13
#define	GROUP4_IN2 	15

#define	GROUP5_IN1	12
#define	GROUP5_IN2 	16

#define	GROUP6_IN1	27
#define	GROUP6_IN2 	29

#define	GROUP7_IN1	18
#define	GROUP7_IN2 	22

#define	GROUP8_IN1	31
#define	GROUP8_IN2 	33

#define	NULL_OUT 	0

// RJSX : RIGHT JOY STICK X DIR (HORIZONTAL)
//#define	RJSX_PWM_PIN_PLUS	GROUP3_IN2
//#define	RJSX_PWM_PIN_MINUS	GROUP3_IN1

// RJSX : RIGHT JOY STICK X DIR (HORIZONTAL)
#define	RJSX_PWM_PIN_PLUS	NULL_OUT
#define	RJSX_PWM_PIN_MINUS	NULL_OUT

// RJSY : RIGHT JOY STICK Y DIR (VERTICAL)
// move to new slider: 
// right joy stick drives turret (x) and slide in out (y)
#define	RJSY_PWM_PIN_PLUS	GROUP7_IN1
//GROUP8_IN1
#define	RJSY_PWM_PIN_MINUS	GROUP7_IN2
//GROUP8_IN2

// LJSX : LEFT JOY STICK X DIR (HORIZONTAL)
#define	LJSX_PWM_PIN_PLUS	GROUP2_IN2
//GROUP5_IN2
//NULL_OUT
#define	LJSX_PWM_PIN_MINUS	GROUP2_IN1
//GROUP5_IN1
//NULL_OUT

// LJSY : LEFT JOY STICK Y DIR (VERTICAL)
#define	LJSY_PWM_PIN_PLUS	NULL_OUT
#define	LJSY_PWM_PIN_MINUS	NULL_OUT
//NULL_OUT


// Trigger Left 1 and 2: 
//#define	RIGHT_TRIGGER_PWM_PIN_PLUS	GROUP2_IN2
//#define	RIGHT_TRIGGER_PWM_PIN_MINUS	GROUP2_IN1
// Trigger Left 1 and 2: 
#define	RIGHT_TRIGGER_PWM_PIN_PLUS	GROUP3_IN2
#define	RIGHT_TRIGGER_PWM_PIN_MINUS	GROUP3_IN1

// Trigger Right 1 and 2: 
#define	LEFT_TRIGGER_PWM_PIN_PLUS	GROUP1_IN2
#define	LEFT_TRIGGER_PWM_PIN_MINUS	GROUP1_IN1

// button pad triangle and X (TX): 
#define	BTN_TX_PWM_PIN_PLUS		GROUP8_IN1
#define	BTN_TX_PWM_PIN_MINUS	GROUP8_IN2

// Gamepad (TY): 
#define	GAMEPAD_LR_PWM_PIN_PLUS		GROUP5_IN1
#define	GAMEPAD_LR_PWM_PIN_MINUS	GROUP5_IN2

// Gamepad (TX): 
#define	GAMEPAD_UD_PWM_PIN_PLUS		GROUP6_IN2
//NULL_OUT
#define	GAMEPAD_UD_PWM_PIN_MINUS	GROUP6_IN1
//NULL_OUT

//
// Scale for JS output range of 0-128: use 64 for 50%	
// 
#define	LJSX_PWM_SCALE	0
#define	LJSY_PWM_SCALE	50
#define	RJSX_PWM_SCALE	0
#define	RJSY_PWM_SCALE	0

#define	LJSX_LT_PWM_SCALE	0
#define	LJSX_RT_PWM_SCALE	0
#define	LJSY_FD_PWM_SCALE	32
#define	LJSY_RV_PWM_SCALE	32
#define	RJSX_LT_PWM_SCALE	32
#define	RJSY_RT_PWM_SCALE	0
#define	RJSX_FD_PWM_SCALE	32
#define	RJSY_RV_PWM_SCALE	0

#define	LEFT_TRIGGER_PWM_PIN_VAL	127
#define	RIGHT_TRIGGER_PWM_PIN_VAL	127
#define	BTN_TX_PWM_PIN_VAL	127
#define	BTN_UD_PWM_PIN_VAL	127

#define PS3 1
#define PS4 0

// define joystick/gamepad i/o for use later:
int jsps3 = 0, jsps4 = 0;
int ljsx_raw_val, ljsy_raw_val, ljs_pb, rjsx_raw_val, rjsy_raw_val, rjs_pb;
int bnt_x, btn_x_raw_val, btn_o, btn_o_raw_val, btn_tri, btn_tri_raw_val, btn_sqr, btn_sqr_raw_val;
int gp_btn_up, gp_btn_up_raw_val, gp_btn_dn, gp_btn_dn_raw_val, gp_btn_lt, gp_btn_lt_raw_val, gp_btn_rt, gp_btn_rt_raw_val;
int trig_2_rt, trig_2_rt_raw_val, trig_1_rt, trig_1_rt_raw_val, trig_1_lt, trig_1_lt_raw_val, trig_2_lt, trig_2_lt_raw_val;
int btn_select_share, btn_start_options;


// this is the one shot count for shooter
#define L2MAX_COUNT 50
#define L2MID_COUNT 35

//
// PWM output for push buttons
//
#define	GAMEPAD_LR_PWM_PIN_VAL 25
#define	GAMEPAD_UP_PWM_PIN_VAL 50
#define	GAMEPAD_DN_PWM_PIN_VAL 35
// fix for non working lt pressure pad
int gp_lt_cnt = 0, gp_lt_val = GAMEPAD_LR_PWM_PIN_VAL;

//
// state def: 1=fw ; 2= rev ; 0 = stop
//
int rjsx_state = state_STOP ; 
int rjsy_state = state_STOP ; 
int ljsx_state = state_STOP ; 
int ljsy_state = state_STOP ; 

int left_trigger_state  = state_STOP ;
int right_trigger_state  = state_STOP ;
int btn_tx_state = state_STOP ;
int btn_ud_state = state_STOP ;

int right_trigger_state_count = 0 ;

int GAMEPAD_UD_state = state_STOP ;
int GAMEPAD_LR_state = state_STOP ;

int btn_gp_fwd;
int btn_gp_rev;
int btn_gp_lt;
int btn_gp_rt;

int rjsx, rjsy;
int ljsx, ljsy;

int main(int argc, char **argv) 
{
	char *progname = *argv++; argc--;
	
	if (wiringPiSetupPhys () == -1)
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", 
		strerror (errno)) ;
		return 1 ;
	}

	// drive all outputs off
	printf("driving all outputs low\n");
	digitalWrite(RJSX_PWM_PIN_PLUS, LOW);
	digitalWrite(RJSX_PWM_PIN_MINUS, LOW);
	digitalWrite(RJSY_PWM_PIN_PLUS, LOW);
	digitalWrite(RJSY_PWM_PIN_MINUS, LOW);
	digitalWrite(LJSX_PWM_PIN_PLUS, LOW);
	digitalWrite(LJSX_PWM_PIN_MINUS, LOW);
	digitalWrite(LJSY_PWM_PIN_PLUS, LOW);
	digitalWrite(LJSY_PWM_PIN_MINUS, LOW);
	digitalWrite(RIGHT_TRIGGER_PWM_PIN_PLUS, LOW);
	digitalWrite(RIGHT_TRIGGER_PWM_PIN_MINUS, LOW);
	digitalWrite(LEFT_TRIGGER_PWM_PIN_PLUS, LOW);
	digitalWrite(LEFT_TRIGGER_PWM_PIN_MINUS, LOW);
	digitalWrite(BTN_TX_PWM_PIN_PLUS, LOW);
	digitalWrite(BTN_TX_PWM_PIN_MINUS, LOW);
	digitalWrite(GAMEPAD_LR_PWM_PIN_PLUS, LOW);
	digitalWrite(GAMEPAD_LR_PWM_PIN_MINUS, LOW);
	digitalWrite(GAMEPAD_UD_PWM_PIN_PLUS, LOW);
	digitalWrite(GAMEPAD_UD_PWM_PIN_MINUS, LOW);
	printf("stopping all soft pwm\n");
	softPwmStop(GAMEPAD_LR_PWM_PIN_MINUS);
	softPwmStop(GAMEPAD_LR_PWM_PIN_PLUS);
	softPwmStop(GAMEPAD_UD_PWM_PIN_MINUS);
	softPwmStop(GAMEPAD_UD_PWM_PIN_PLUS);
	softPwmStop(LEFT_TRIGGER_PWM_PIN_MINUS);
	softPwmStop(LEFT_TRIGGER_PWM_PIN_PLUS);
	softPwmStop(LJSX_PWM_PIN_MINUS);
	softPwmStop(LJSX_PWM_PIN_PLUS);
	softPwmStop(LJSY_PWM_PIN_MINUS);
	softPwmStop(LJSY_PWM_PIN_PLUS);
	softPwmStop(RIGHT_TRIGGER_PWM_PIN_MINUS);
	softPwmStop(RIGHT_TRIGGER_PWM_PIN_PLUS);
	softPwmStop(RJSX_PWM_PIN_MINUS);
	softPwmStop(RJSX_PWM_PIN_PLUS);
	softPwmStop(RJSY_PWM_PIN_MINUS);
	softPwmStop(RJSY_PWM_PIN_PLUS);

	//
	//////////// Bluetooth PS3 stuff ////////////////////////////
	// 
	int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0;
	char *button=NULL, name_of_joystick[80];
	char ps3[80], ps4[80];
	int jstype;
	
	struct js_event js;
	    sprintf(ps3, "PLAYSTATION(R)3 Controller");
	    sprintf(ps4, "Sony Computer Entertainment Wireless Controller");
	printf("waiting for bluetooth controller to pair\n");
	while( 1 ) {
		if( ( joy_fd = open( JOY_DEV , O_RDONLY)) != -1 )
		{
				break ;
			// printf( "Couldn't open joystick\n" );
			// return -1;
		}
	}
		

	ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
	ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
	ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

	axis = (int *) calloc( num_of_axis, sizeof( int ) );
	button = (char *) calloc( num_of_buttons, sizeof( char ) );

	printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
		, name_of_joystick
		, num_of_axis
		, num_of_buttons );

	fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */
	
	if (	strcmp(ps3, name_of_joystick)== 0) {
		jstype = PS3;
	} else {
		jstype = PS4;
	}
	int nextRead = int(READDELAYMS);
	//int delaycount = 0;
	//int pcount = 0 ;
	
	//printf("joystick names: %s, %s, %s,\n",ps3, ps4, name_of_joystick);

	// 
	// Major while loop: this re-reads the controller every 10 ms and
	// loops to control the motor output based on the joystick.
	//
	while (1)	{
		if (int(millis()) > nextRead) {
			nextRead += READDELAYMS;
			
			//
			// Read the controller.
			//
			read(joy_fd, &js, sizeof(struct js_event));
		
			/* see what to do with the event */
			switch (js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_AXIS:
					axis   [ js.number ] = js.value;
					break;
				case JS_EVENT_BUTTON:
					button [ js.number ] = js.value;
					break;
			}

            //if (name_of_joystick == ps3){
			if (	jstype == PS3 ) { //strcmp(ps3, name_of_joystick)== 0) {
				//printf("using ps3 %s\n",ps3);
                // This is the PS3
                //
                // right joystick x is axis[2]
                // right joystick y is axis[3]
                // left  joystick x is axis[0]
                // left  joystick y is axis[1]
                rjsx_raw_val = axis[2];
                rjsy_raw_val = axis[3];
                ljsx_raw_val = axis[0];
                ljsy_raw_val = axis[1];
                // buttons:
                //		button[0] = select
                //		button[1] = Left Joystick
                //		button[2] = Right Joystick
                //		button[3] = start
                //		button[4] = GP UP
                //		button[5] = GP RT
                //		button[6] = GP DN
                //		button[7] = GP LT
                //		button[8] = L2
                //		button[9] = R2
                //		button[10] = L1
                //		button[11] = R1
                //		button[12] = TRIANGLE
                //		button[13] = CIRCLE
                //		button[14] = CROSS
                //		button[15] = SQUARE
				btn_select_share	= button[0];
				btn_start_options 	= button[3];
                
                trig_2_rt = button[9];
                trig_1_rt = button[11];
                trig_1_lt = button[8];
                trig_2_lt = button[10];
                ljs_pb 	  = button[1];
                rjs_pb    = button[2]; 
                gp_btn_up = button[4]; 
                gp_btn_dn = button[6];
                gp_btn_lt = button[7];
                gp_btn_rt = button[5];
                
           //} else if (name_of_joystick == ps4){
			} else { //if (strcmp(ps4, name_of_joystick)== 0) {

                // This is the PS4
                /*Axes:
                axis[0] LJSX
                axis[1] LJSY
                axis[2] RJSX
                axis[3] TRIGGER LEFT 2
                axis[4] TRIGGER RIGHT 2
                axis[5] RJSY
                axis[6] GPLR
                axis[7] GPUD
                axis[8]
                axis[9]  TOUCHX
                axis[10] TOUCHY
                axis[11]
                
                 Buttons:
                button[0]  SQUARE
                button[1]  X
                button[2]  CIRCLE
                button[3]  TRIANGLE
                button[4]  TRIGGER LEFT 1
                button[5]  TRIGGER RIGHT 1
                button[6]  TRIGGER LEFT 2
                button[7]  TRIGGER RIGHT 2
                button[8]  SHARE
                button[9]  OPTIONS
                button[10] LJSB
                button[11] RJSB
                button[12] PS
                button[13] TOUCHB
             */
                ljsx_raw_val = axis[0];
                ljsy_raw_val = axis[1];
                rjsx_raw_val = axis[2];
                rjsy_raw_val = axis[5];
                
                trig_2_rt = button[5];
                trig_1_rt = button[7];
                trig_1_lt = button[4];
                trig_2_lt = button[6];
				btn_select_share	= button[8];
				btn_start_options 	= button[9];
                ljs_pb = button[10];
                rjs_pb = button[11];   

                gp_btn_up = button[4]; 
                gp_btn_dn = button[4];
                gp_btn_lt = button[4];
                gp_btn_rt = button[4];
             }
            
			
			// Just going to check for the START button.
			if (btn_start_options == 1) {
				printf("BTN_START has been pushed DOWN\n");
			}
			
			//
			// <<<<<<<<<<<<< JOYSTICK BEGIN >>>>>>>>>>>>>>>>
            // Right Joystick 
			//
            // Right Joy   Vertical block start
            //
            if( rjsy != rjsy_raw_val )			{
			// right joystick x is rjsx_raw_val
			// right joystick y is rjsy_raw_val
				printf("RAW RJSY = %d\n", rjsy);
				rjsy = rjsy_raw_val;

				if( rjsy < 0  ) {
					int rjsy_val = (-1 * rjsy) / 256 ;
					if(rjsy_val > 120)
						rjsy_val = 120;
						printf("Right JS Y forward Value = %d\n", rjsy_val);

					if( rjsy_state == state_FW ) {
						// we are already in forward moving motor state;
						softPwmWrite( RJSY_PWM_PIN_PLUS, rjsy_val);
						digitalWrite (RJSY_PWM_PIN_MINUS, LOW) ;
					
					} else {
						// we are NOT in forward moving motor state; check if we are in rev
						if( rjsy_state == state_REV ) {
							softPwmStop(RJSY_PWM_PIN_MINUS);
							//printf("softPwmStop REV  Vertical = %d\n", rjsy_val);
                    } // we are not in either forward or reverse; start here
						rjsy_state = state_FW ;
						softPwmCreate( RJSY_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( RJSY_PWM_PIN_PLUS, rjsy_val);
						digitalWrite (RJSY_PWM_PIN_MINUS, LOW) ;
						//printf("digitalWrite FW  Vertical = %d\n", rjsy_val);					
					}

				} else if ( rjsy > 0 ) {
					int rjsy_val = rjsy / 256;
					if(rjsy_val > 120)
						rjsy_val = 120;
						printf("Right JS Y back Value = %d\n", rjsy_val);
					if( rjsy_state == state_REV ) {
						// we are in reverse moving motor state;
						softPwmWrite( RJSY_PWM_PIN_MINUS, rjsy_val);
						digitalWrite (RJSY_PWM_PIN_PLUS, LOW) ;	
						//printf("digitalWrite REV  Vertical = %d\n", rjsy_val);
					} else {
						// we are NOT in rev moving motor state; check if we are in fw
						if( rjsy_state == state_FW ) {
							softPwmStop(RJSY_PWM_PIN_PLUS);
							//printf("softPwmStop FW  Vertical = %d\n", rjsy_val);
                    }
						rjsy_state = state_REV ;
						softPwmCreate( RJSY_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( RJSY_PWM_PIN_MINUS, rjsy_val);
						digitalWrite (RJSY_PWM_PIN_PLUS, LOW) ;
						//printf("digitalWrite REV  Vertical = %d\n", rjsy_val);
					}

				} else {
					if( rjsy_state == state_FW ) {
						softPwmStop(RJSY_PWM_PIN_PLUS);
						//printf("softPwmStop FW  Vertical = %d\n", pips2.PS2data[6]);

					}
					if( rjsy_state == state_REV ) {
						softPwmStop(RJSY_PWM_PIN_MINUS);
						//printf("softPwmStop REV  Vertical = %d\n", pips2.PS2data[6]);
					}
					rjsy_state = state_STOP ;

					digitalWrite (RJSY_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (RJSY_PWM_PIN_MINUS, HIGH) ;	// On					
				}
					
			}
            //
			// Right Joy   Vertical  block end
			//

			//
			// Right Joy   Horizontal block start
            //
            // Check to see if register has been updated:
 			if( (rjsx != rjsx_raw_val))	{
			// right joystick x is rjsx_raw_val >> turret rotate l/r
				//printf("Right Joy Turret Spin RAW = %d\n", rjsx);
				rjsx = rjsx_raw_val;
				if( rjsx < 0  ) {
					int rjsx_val = ( -1 * rjsx )/256 - RJSX_PWM_SCALE ;
					if(rjsx_val < 0)
						rjsx_val = 10;
						//printf("Right JS X turret left Scaled = %d\n", rjsx_val);

					if( rjsx_state == state_FW ) {
						// we are in forward moving motor state;
						softPwmWrite( RJSX_PWM_PIN_PLUS, rjsx_val);
						digitalWrite (RJSX_PWM_PIN_MINUS, LOW) ;
					
					} else {
						// we are NOT in forward moving motor state; check if we are in rev
						if( rjsx_state == state_REV ) {
							softPwmStop(RJSX_PWM_PIN_MINUS);
							//printf("softPwmStop REV  Horizontal = %d\n", rjsx_val);
                    }
						rjsx_state = state_FW ;
						softPwmCreate( RJSX_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( RJSX_PWM_PIN_PLUS, rjsx_val);
						digitalWrite (RJSX_PWM_PIN_MINUS, LOW) ;
						//printf("digitalWrite FW  Horizontal = %d\n", rjsx_val);					
					}

				} else if ( rjsx > 0 ) {
					int rjsx_val = rjsx/256 - RJSX_PWM_SCALE;
					if(rjsx_val < 0)
						rjsx_val = 10;
						//printf("Right JS X turret right Scaled = %d\n", rjsx_val);
					if( rjsx_state == state_REV ) {
						// we are in reverse moving motor state;
						softPwmWrite( RJSX_PWM_PIN_MINUS, rjsx_val);
						digitalWrite (RJSX_PWM_PIN_PLUS, LOW) ;	
						//printf("digitalWrite REV  Horizontal = %d\n", rjsx_val);
					} else {
						// we are NOT in rev moving motor state; check if we are in fw
						if( rjsx_state == state_FW ) {
							softPwmStop(RJSX_PWM_PIN_PLUS);
							//printf("softPwmStop FW  Horizontal = %d\n", rjsx_val);
                    }
						rjsx_state = state_REV ;
						softPwmCreate( RJSX_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( RJSX_PWM_PIN_MINUS, rjsx_val);
						digitalWrite (RJSX_PWM_PIN_PLUS, LOW) ;
						//printf("digitalWrite REV  Horizontal = %d\n", rjsx_val);
					}

				} else {
					if( rjsx_state == state_FW ) {
						softPwmStop(RJSX_PWM_PIN_PLUS);
						//printf("softPwmStop FW  Horizontal = %d\n", pips2.PS2data[5]);

					}
					if( rjsx_state == state_REV ) {
						softPwmStop(RJSX_PWM_PIN_MINUS);
						//printf("softPwmStop REV  Horizontal = %d\n", pips2.PS2data[5]);
					}
					rjsx_state = state_STOP ;

					digitalWrite (RJSX_PWM_PIN_PLUS, HIGH) ;	// Off
					digitalWrite (RJSX_PWM_PIN_MINUS, HIGH) ;// Off					
				}
			}
			// Right Joy   Horizontal block end
			// Right Joystick end
			////////////////////////////////////////

			// right joy stick button push
			if (rjs_pb) {
				printf("RIGHT_JOY_BTN is pressed\n");
			}
			
			//
			// 	Left Joystick
			//	ljsx = ljsx_raw_val;
			//	ljsy = ljsy_raw_val;
			//
			// Left Joy   Horizontal start
			//
			if( ljsx != ljsx_raw_val){
			//	ljsx = ljsx_raw_val; : spin coin chute
				printf("Left Joy   Horizontal RAW = %d\n", ljsx);
				ljsx = ljsx_raw_val;
				if( ( ljsx < 0 ) ) {
					int ljsx_val = (-1 * ljsx/256) - LJSX_PWM_SCALE ;
					if(ljsx_val < 0)
						ljsx_val = 10;
					printf("Left JS Left  Horizontal = %d\n", ljsx_val);
					if( ljsx_state == state_FW ) {
						// we are in forward moving motor state;
						softPwmWrite( LJSX_PWM_PIN_PLUS, ljsx_val);
						digitalWrite (LJSX_PWM_PIN_MINUS, LOW) ;
						//printf("digitalWrite FW  Horizontal = %d\n", ljsx_val);
					
					} else {
						// we are NOT in forward moving motor state; check if we are in rev
						if( ljsx_state == state_REV ) {
							softPwmStop(LJSX_PWM_PIN_MINUS);
							//printf("softPwmStop REV  Horizontal = %d\n", ljsx_val);
						}
						ljsx_state = state_FW ;
						softPwmCreate( LJSX_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( LJSX_PWM_PIN_PLUS, ljsx_val);
						digitalWrite (LJSX_PWM_PIN_MINUS, LOW) ;
						//printf("digitalWrite FW  Horizontal = %d\n", ljsx_val);	
					}

				} else if ( ( ljsx > 0 )) {
					int ljsx_val = (ljsx/256) - LJSX_PWM_SCALE;
					if(ljsx_val < 0)
						ljsx_val = 10;
					printf("Left JS Right  Horizontal = %d\n", ljsx_val);
					if( ljsx_state == state_REV ) {
						// we are in reverse moving motor state;
						softPwmWrite( LJSX_PWM_PIN_MINUS, ljsx_val);
						digitalWrite (LJSX_PWM_PIN_PLUS, LOW) ;	
						//printf("digitalWrite REV  Horizontal = %d\n", ljsx_val);
					} else {
						// we are NOT in rev moving motor state; check if we are in fw
						if( ljsx_state == state_FW ) {
							softPwmStop(LJSX_PWM_PIN_PLUS);
							//printf("softPwmStop FW  Horizontal = %d\n", ljsx_val);
						}
						ljsx_state = state_REV ;
						softPwmCreate( LJSX_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( LJSX_PWM_PIN_MINUS, ljsx_val);
						digitalWrite (LJSX_PWM_PIN_PLUS, LOW) ;
						//printf("digitalWrite REV  Horizontal = %d\n", ljsx_val);
					}

				} else {
					if( ljsx_state == state_FW ) {
						softPwmStop(LJSX_PWM_PIN_PLUS);
						//printf("softPwmStop FW  Horizontal = %d\n", pips2.PS2data[7]);
					}
					if( ljsx_state == state_REV ) {
						softPwmStop(LJSX_PWM_PIN_MINUS);
						//printf("softPwmStop REV  Horizontal = %d\n", pips2.PS2data[7]);
					}
					ljsx_state = state_STOP ;

					digitalWrite (LJSX_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (LJSX_PWM_PIN_MINUS, HIGH) ;	// On					
				}				
			}

			// Left Joy   Horizontal  block end
			//
			// Left Joy   Vertical start			
			//
			if ( ljsy != ljsy_raw_val )	{
			//	ljsy = ljsy_raw_val : up down for crane
				printf("RAW Left Joystick VERTICAL = %d\n", ljsy);
				ljsy = ljsy_raw_val;
				// Joystick pressed forward
				if( ( ljsy < 0 ) ) {
					int ljsy_val = (-1* ljsy/256) - LJSY_PWM_SCALE;
					
					printf("Left JS Forward  Vertical = %d\n", ljsy_val);

					if( ljsy_state == state_FW ) {
						// we are in forward moving motor state;
						softPwmWrite( LJSY_PWM_PIN_PLUS, ljsy_val);
						digitalWrite (LJSY_PWM_PIN_MINUS, LOW) ;
						//printf("digitalWrite FW  VERTICAL = %d\n", ljsy_val);
					
					} else {
						// we are NOT in forward moving motor state; check if we are in rev
						if( ljsy_state == state_REV ) {
							softPwmStop(LJSY_PWM_PIN_MINUS);
							//printf("softPwmStop REV  VERTICAL = %d\n", ljsy_val);

						}
						ljsy_state = state_FW ;
						softPwmCreate( LJSY_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( LJSY_PWM_PIN_PLUS, ljsy_val);
						digitalWrite (LJSY_PWM_PIN_MINUS, LOW) ;
						//printf("digitalWrite FW  VERTICAL = %d\n", ljsy_val);
	
					
					}
				// Joystick pulled back
				} else if ( ( ljsy > 0 )) {
					int ljsy_val = ljsy/256 - LJSY_PWM_SCALE;
					if(ljsy_val < 0)
						ljsy_val = 10;
					printf("Left JS Back  Vertical = %d\n", ljsy_val);

					if( ljsy_state == state_REV ) {
						// we are in reverse moving motor state;
						softPwmWrite( LJSY_PWM_PIN_MINUS, ljsy_val);
						digitalWrite (LJSY_PWM_PIN_PLUS, LOW) ;	
						//printf("digitalWrite REV  VERTICAL = %d\n", ljsy_val);
					} else {
						// we are NOT in rev moving motor state; check if we are in fw
						if( ljsy_state == state_FW ) {
							softPwmStop(LJSY_PWM_PIN_PLUS);
							//printf("softPwmStop FW  VERTICAL = %d\n", ljsy_val);
						}
						ljsy_state = state_REV ;
						softPwmCreate( LJSY_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( LJSY_PWM_PIN_MINUS, ljsy_val);
						digitalWrite (LJSY_PWM_PIN_PLUS, LOW) ;
						//printf("digitalWrite REV  VERTICAL = %d\n", ljsy_val);
					}

				} else {
					if( ljsy_state == state_FW ) {
						softPwmStop(LJSY_PWM_PIN_PLUS);
						//printf("softPwmStop FW  VERTICAL = %d\n", pips2.PS2data[8]);

					}
					if( ljsy_state == state_REV ) {
						softPwmStop(LJSY_PWM_PIN_MINUS);
						//printf("softPwmStop REV  VERTICAL = %d\n", pips2.PS2data[8]);
					}
					ljsy_state = state_STOP ;

					digitalWrite (LJSY_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (LJSY_PWM_PIN_MINUS, HIGH) ;	// On			
				}		
				
			}
			// Left Joy   Vertical block end
			// Left Joystick end
			///////////////////////////////////
			if(ljsy_raw_val == 0){// stop runaway
					if( ljsy_state == state_FW ) {
						softPwmStop(LJSY_PWM_PIN_PLUS);
						//printf("softPwmStop FW  VERTICAL = %d\n", pips2.PS2data[8]);

					}
					if( ljsy_state == state_REV ) {
						softPwmStop(LJSY_PWM_PIN_MINUS);
						//printf("softPwmStop REV  VERTICAL = %d\n", pips2.PS2data[8]);
					}
					ljsy_state = state_STOP ;

					digitalWrite (LJSY_PWM_PIN_PLUS, LOW) ;	// On
					digitalWrite (LJSY_PWM_PIN_MINUS, LOW) ;	// On			
				}		
			if(rjsx_raw_val == 0){// add this to stop runaway turret
					if( rjsx_state == state_FW ) {
						softPwmStop(RJSX_PWM_PIN_PLUS);
						//printf("softPwmStop FW  Horizontal = %d\n", pips2.PS2data[5]);

					}
					if( rjsx_state == state_REV ) {
						softPwmStop(RJSX_PWM_PIN_MINUS);
						//printf("softPwmStop REV  Horizontal = %d\n", pips2.PS2data[5]);
					}
					rjsx_state = state_STOP ;

					digitalWrite (RJSX_PWM_PIN_PLUS, HIGH) ;	// Off
					digitalWrite (RJSX_PWM_PIN_MINUS, HIGH) ;// Off					
				
			}
 			if(rjsy_raw_val == 0){
					if( rjsy_state == state_FW ) {
						softPwmStop(RJSY_PWM_PIN_PLUS);
						//printf("softPwmStop FW  Vertical = %d\n", pips2.PS2data[6]);

					}
					if( rjsy_state == state_REV ) {
						softPwmStop(RJSY_PWM_PIN_MINUS);
						//printf("softPwmStop REV  Vertical = %d\n", pips2.PS2data[6]);
					}
					rjsy_state = state_STOP ;

					digitalWrite (RJSY_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (RJSY_PWM_PIN_MINUS, HIGH) ;	// On					
				}				

			if (ljs_pb) {
				printf("LEFT_JOY_BTN is pressed\n");	
			}		
			// Joystick ends
			//
			
			//
			// check trigger buttons
			// 										
			//		button[10] 	= L1
			//		button[8] 	= L2
			//
			//		button[11] 	= R1
			//		button[9] 	= R2
			//
			
			if (trig_2_rt || trig_1_rt)   { // Right trigger 1/2
				int rt_trig_val = (axis[15]+32767 ) / 512;
				if(trig_2_rt){	
					if( right_trigger_state == state_FW ) {
					//
					// we are in forward moving motor state;	
					//	
						if(rt_trig_val > 90)
						  rt_trig_val = 90;
						softPwmWrite( RIGHT_TRIGGER_PWM_PIN_MINUS, rt_trig_val);
					} else {
						printf("Right trigger R2 is pressed\n");
						// we are NOT in forward moving motor state; check if we are in rev
						if( right_trigger_state == state_REV ) {
							softPwmStop(RIGHT_TRIGGER_PWM_PIN_MINUS);
						}
						right_trigger_state = state_FW ;
						softPwmCreate( RIGHT_TRIGGER_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( RIGHT_TRIGGER_PWM_PIN_PLUS, RIGHT_TRIGGER_PWM_PIN_VAL);
						digitalWrite (RIGHT_TRIGGER_PWM_PIN_MINUS, LOW) ;
						printf("RIGHT_TRIGGER_PWM_PIN_PLUS = %d\n", RIGHT_TRIGGER_PWM_PIN_VAL);					
					}				
				} else if (trig_1_rt) { // R1 shoot trigger				
						if(rt_trig_val > 90)
						  rt_trig_val = 90;
						printf("Right trigger R1 is %d speed %d\n", trig_1_rt, rt_trig_val);
						// we are NOT in rev moving motor state; check if we are in fw
						if( right_trigger_state == state_FW ) {
							softPwmStop(RIGHT_TRIGGER_PWM_PIN_PLUS);
						}
						right_trigger_state = state_REV ;
						right_trigger_state_count = L2MAX_COUNT ;					
						softPwmCreate( RIGHT_TRIGGER_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( RIGHT_TRIGGER_PWM_PIN_MINUS, rt_trig_val);
						digitalWrite (RIGHT_TRIGGER_PWM_PIN_PLUS, LOW) ;
						printf("rt_trig_val REV   = %d\n", rt_trig_val);
				}
			} else {
				if( right_trigger_state == state_FW ) {
					softPwmStop(RIGHT_TRIGGER_PWM_PIN_PLUS);
				}
				if( right_trigger_state == state_REV ) {
					softPwmStop(RIGHT_TRIGGER_PWM_PIN_MINUS);
				}
				if( right_trigger_state != state_STOP ) {

					right_trigger_state = state_STOP ;
					printf("Right trigger R1/2 released\n");
					digitalWrite (RIGHT_TRIGGER_PWM_PIN_PLUS, HIGH) ;	// On brake
					digitalWrite (RIGHT_TRIGGER_PWM_PIN_MINUS, HIGH) ;	// On	
				}				
			}
			
			if (trig_1_lt || trig_2_lt){ // left trigger 1/2
				if (trig_1_lt)	{
					if( left_trigger_state == state_FW ) {
						// we are in forward moving motor state;
					} else {
						printf("Left trigger L2 is pressed\n");
						// we are NOT in forward moving motor state; check if we are in rev
						if( left_trigger_state == state_REV ) {
							softPwmStop(LEFT_TRIGGER_PWM_PIN_MINUS);
						}
						left_trigger_state = state_FW ;
						softPwmCreate( LEFT_TRIGGER_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( LEFT_TRIGGER_PWM_PIN_PLUS, LEFT_TRIGGER_PWM_PIN_VAL);
						digitalWrite (LEFT_TRIGGER_PWM_PIN_MINUS, LOW) ;
						printf("LEFT_TRIGGER_PWM_PIN_PLUS = %d\n", LEFT_TRIGGER_PWM_PIN_VAL);
					}				
				}	else if (trig_2_lt)	{
					if( left_trigger_state == state_REV ) {
						// we are in reverse moving motor state;
					} else {
						printf("Left trigger L1 is pressed\n");
	
						// we are NOT in rev moving motor state; check if we are in fw
						if( left_trigger_state == state_FW ) {
							softPwmStop(LEFT_TRIGGER_PWM_PIN_PLUS);
						}
						left_trigger_state = state_REV ;
						softPwmCreate( LEFT_TRIGGER_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( LEFT_TRIGGER_PWM_PIN_MINUS, LEFT_TRIGGER_PWM_PIN_VAL);
						digitalWrite (LEFT_TRIGGER_PWM_PIN_PLUS, LOW) ;
						printf("LEFT_TRIGGER_PWM_PIN_MINUS REV   = %d\n", LEFT_TRIGGER_PWM_PIN_VAL);
					}
				}
			} else {
				if( left_trigger_state == state_FW ) {
					softPwmStop(LEFT_TRIGGER_PWM_PIN_PLUS);
				}
				if( left_trigger_state == state_REV ) {
					softPwmStop(LEFT_TRIGGER_PWM_PIN_MINUS);
				}
				if( left_trigger_state != state_STOP ) {
					left_trigger_state = state_STOP ;
					printf("Left trigger L1/2 released\n");
					digitalWrite (LEFT_TRIGGER_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (LEFT_TRIGGER_PWM_PIN_MINUS, HIGH) ;	// On
				}				
			}
					

			//
			// Check GAMEPAD
			//
			//		button[4] = GP UP
			//		button[5] = GP RT
			//		button[6] = GP DN
			//		button[7] = GP LT
			if (button[4] || button[6]){
					int gp_up_val = axis[8];
					int gp_dn_val = axis[10];
					printf("RAW gp val: %d up %d dn \n",gp_up_val, gp_dn_val);
				if (button[4])  {
					printf("button 4 pushed: game pad up\n");			
					if( GAMEPAD_UD_state == state_FW ) {
						if(gp_up_val > 0){
							gp_up_val = (gp_up_val/256)< GAMEPAD_UP_PWM_PIN_VAL ? GAMEPAD_UP_PWM_PIN_VAL:gp_up_val/256;
						} else {
							gp_up_val = GAMEPAD_UP_PWM_PIN_VAL;
						}
						printf("\ntilt up GAMEPAD_UP is pressed force = %d\n", gp_up_val);
						softPwmWrite( GAMEPAD_UD_PWM_PIN_PLUS, gp_up_val);
						// we are in forward moving motor state;			
					} else {
						if(gp_up_val > 0){
							gp_up_val = (gp_up_val/256)< GAMEPAD_UP_PWM_PIN_VAL ? GAMEPAD_UP_PWM_PIN_VAL:gp_up_val/256;
						} else {
							gp_up_val = GAMEPAD_UP_PWM_PIN_VAL;
						}
						printf("\ntilt up GAMEPAD_UP is pressed force = %d\n", gp_up_val);
						// we are NOT in forward moving motor state; check if we are in rev
						if( GAMEPAD_UD_state == state_REV ) {
							softPwmStop(GAMEPAD_UD_PWM_PIN_MINUS);
						}
						GAMEPAD_UD_state = state_FW ;
						softPwmCreate( GAMEPAD_UD_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( GAMEPAD_UD_PWM_PIN_PLUS, gp_up_val);
						digitalWrite (GAMEPAD_UD_PWM_PIN_MINUS, LOW) ;
						printf("tilt up GAMEPAD_UD_PWM_PIN_PLUS = %d\n", GAMEPAD_UP_PWM_PIN_VAL);
					}
				} else if (button[6])  {
					//printf("button 6 pushed: game pad down\n");			
						if(gp_dn_val > 0){
							gp_dn_val = (gp_dn_val/256)< GAMEPAD_DN_PWM_PIN_VAL ? GAMEPAD_DN_PWM_PIN_VAL:gp_dn_val/256 ;
						} else {
							gp_dn_val = GAMEPAD_DN_PWM_PIN_VAL;
						}
						softPwmWrite( GAMEPAD_UD_PWM_PIN_MINUS, gp_dn_val);
					if( GAMEPAD_UD_state == state_REV ) {
						// we are in reverse moving motor state;
					} else {
						printf("\ntilt down GAMEPAD_DOWN is pressed force = %d\n", gp_dn_val);
						// we are NOT in rev moving motor state; check if we are in fw
						if( GAMEPAD_UD_state == state_FW ) {
							softPwmStop(GAMEPAD_UD_PWM_PIN_PLUS);
						}
						GAMEPAD_UD_state = state_REV ;
						softPwmCreate( GAMEPAD_UD_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( GAMEPAD_UD_PWM_PIN_MINUS, gp_dn_val);
						digitalWrite (GAMEPAD_UD_PWM_PIN_PLUS, LOW) ;
						printf("tilt down GAMEPAD_UD_PWM_PIN_MINUS REV   = %d\n", GAMEPAD_DN_PWM_PIN_VAL);
					}
				}
			} else {
				if( (GAMEPAD_UD_state == state_FW )  ){
					softPwmStop(GAMEPAD_UD_PWM_PIN_PLUS);
				printf("button 4 & 6 released: state FW\n");			
				}
				if( (GAMEPAD_UD_state == state_REV )  ){
					softPwmStop(GAMEPAD_UD_PWM_PIN_MINUS);
				printf("button 4 & 6 released: state REV\n");			
				}
				if( GAMEPAD_UD_state != state_STOP ) {
					GAMEPAD_UD_state = state_STOP ;
					printf("GAMEPAD UP/DOWN released\nState STOP w. BRAKE (ON, ON)\n");
					digitalWrite (GAMEPAD_UD_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (GAMEPAD_UD_PWM_PIN_MINUS, HIGH) ;	// On	
				}				
			}
				
			if (button[5] || button[7]){
				if (button[5] ) {
					int gp_rt_val = axis[9]/256;
					if( GAMEPAD_LR_state == state_REV ) {
						// we are in reverse moving motor state;
						if(gp_rt_val < GAMEPAD_LR_PWM_PIN_VAL){
							gp_rt_val = GAMEPAD_LR_PWM_PIN_VAL;
						} 
						  
						softPwmWrite( GAMEPAD_LR_PWM_PIN_PLUS, gp_rt_val);
						printf("spin right GAMEPAD_LR_PWM_PIN_PLUS REV   = %d\n", gp_rt_val);
					} else {
						printf("spin right: GAMEPAD_RIGHT is pressed\n");
						// we are NOT in rev moving motor state; check if we are in fw
						if( GAMEPAD_LR_state == state_FW ) {
							softPwmStop(GAMEPAD_LR_PWM_PIN_MINUS);
						}
						GAMEPAD_LR_state = state_REV ;
						softPwmCreate( GAMEPAD_LR_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( GAMEPAD_LR_PWM_PIN_PLUS, gp_rt_val);
						digitalWrite (GAMEPAD_LR_PWM_PIN_MINUS, LOW) ;
						printf("spin right GAMEPAD_LR_PWM_PIN_PLUS REV   = %d\n", gp_rt_val);
					}
	
				} else if (button[11])  {				
					//int gp_lt_val = GAMEPAD_LR_PWM_PIN_VAL;
					// fix for non working pressure pad
					int gp_lt_val = axis[7]/256;
					if( GAMEPAD_LR_state == state_FW ) {	
						// we are in forward moving motor state;
						//if(gp_rt_val < GAMEPAD_LR_PWM_PIN_VAL){
							//gp_rt_val = GAMEPAD_LR_PWM_PIN_VAL;
						//} 
						  gp_lt_cnt += 1;
						  if(gp_lt_cnt == 50){
						    gp_lt_val += 5;
						    gp_lt_cnt = 0;
						}
						//if (gp_lt_val > 127)
						//	gp_lt_val = 127 ;
						printf("running left: count %d force %d \n",gp_lt_cnt, gp_lt_val);
						softPwmWrite( GAMEPAD_LR_PWM_PIN_MINUS, gp_lt_val);
					} else {
						printf("spin left :GAMEPAD_LEFT is pressed\n");
						// we are NOT in forward moving motor state; check if we are in rev
						if( GAMEPAD_LR_state == state_REV ) {
							softPwmStop(GAMEPAD_LR_PWM_PIN_PLUS);
						}
						GAMEPAD_LR_state = state_FW ;
						softPwmCreate( GAMEPAD_LR_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( GAMEPAD_LR_PWM_PIN_MINUS, GAMEPAD_LR_PWM_PIN_VAL);
						digitalWrite (GAMEPAD_LR_PWM_PIN_PLUS, LOW) ;
						printf("spin left GAMEPAD_LR_PWM_PIN_MINUS = %d\n", GAMEPAD_LR_PWM_PIN_VAL);	
					}
				}
			}  else {
				if( (GAMEPAD_LR_state == state_FW )  ){// button[7]
					softPwmStop(GAMEPAD_LR_PWM_PIN_MINUS);
				printf("button 5 & 7 released: state FW\n");			
				}
				if( (GAMEPAD_LR_state == state_REV )  ){
					softPwmStop(GAMEPAD_LR_PWM_PIN_PLUS);
				printf("button 5 & 7 released: state REV\n");			
				}
				if( GAMEPAD_LR_state != state_STOP ) {
					GAMEPAD_LR_state = state_STOP ;
					gp_lt_cnt = 0;
					gp_lt_val = GAMEPAD_LR_PWM_PIN_VAL;
					printf("GAMEPAD LEFT/RIGHT released\nState STOP\n");
					digitalWrite (GAMEPAD_LR_PWM_PIN_PLUS, HIGH) ;	// On
					digitalWrite (GAMEPAD_LR_PWM_PIN_MINUS, HIGH) ;	// On	
				}				
			}  

			//
			// GAMEPAD ends
			//
			
			//
			// check symbol pad
			// 										
			//		button[12] = TRIANGLE
			//		button[13] = CIRCLE
			//		button[14] = CROSS
			//		button[15] = SQUARE
			if(button[12] || button[14]){
				if (button[12])   {	
					printf("Button TRI pushed: extend/lower robot arm...\n");		
					if( btn_tx_state == state_FW ) {
						// we are in forward moving motor state;
						//softPwmWrite( BTN_TX_PWM_PIN_PLUS, BTN_TX_PWM_PIN_VAL);
						//digitalWrite (BTN_TX_PWM_PIN_MINUS, LOW) ;	
						//printf("BTN_TX_PWM_PIN_MINUS = %d\n", BTN_TX_PWM_PIN_VAL);
					} else {
						// we are NOT in rev moving motor state; check if we are in fw
						if( btn_tx_state == state_REV ) {
							softPwmStop(BTN_TX_PWM_PIN_MINUS);
						}
						btn_tx_state = state_FW ;
						softPwmCreate( BTN_TX_PWM_PIN_PLUS, 0, 127 ) ;
						softPwmWrite( BTN_TX_PWM_PIN_PLUS, BTN_TX_PWM_PIN_VAL);
						digitalWrite (BTN_TX_PWM_PIN_MINUS, LOW) ;
						printf("BTN_TX_PWM_PIN_MINUS REV   = %d\n", BTN_TX_PWM_PIN_VAL);
					}
				} 
				
				if (button[14]  ) {	
					printf("button cross pressed: put away robot arm...\n");		
					if( btn_tx_state == state_REV ) {
						// we are in reverse moving motor state;
						//softPwmWrite( BTN_TX_PWM_PIN_MINUS, BTN_TX_PWM_PIN_VAL);
						//digitalWrite (BTN_TX_PWM_PIN_PLUS, LOW) ;	
						//printf("BTN_TX_PWM_PIN_MINUS = %d\n", BTN_TX_PWM_PIN_VAL);
					} else {
						// we are NOT in rev moving motor state; check if we are in fw
						if( btn_tx_state == state_FW ) {
							softPwmStop(BTN_TX_PWM_PIN_PLUS);
						}
						btn_tx_state = state_REV ;
						softPwmCreate( BTN_TX_PWM_PIN_MINUS, 0, 127 ) ;
						softPwmWrite( BTN_TX_PWM_PIN_MINUS, BTN_TX_PWM_PIN_VAL);
						digitalWrite (BTN_TX_PWM_PIN_PLUS, LOW) ;
						printf("BTN_TX_PWM_PIN_MINUS REV   = %d\n", BTN_TX_PWM_PIN_VAL);
					}
				} 
			} else {
				if( btn_tx_state == state_REV ) {
					printf("from state reverse to stop\n");
					softPwmStop(BTN_TX_PWM_PIN_MINUS);
				}
				if( btn_tx_state == state_FW ) {
					printf("from state forward to stop\n");
					softPwmStop(BTN_TX_PWM_PIN_PLUS);
				}
				if( btn_tx_state != state_STOP ) {

					btn_tx_state = state_STOP ;
					printf("BTN_TRIANGLE or BTN_X released\n");

					digitalWrite (BTN_TX_PWM_PIN_PLUS, LOW) ;	// On
					digitalWrite (BTN_TX_PWM_PIN_MINUS, LOW) ;	// On	
				}				
			}
												
			if (button[13])
				printf("BTN_CIRCLE is pressed\n");

			if (button[15])
				printf("BTN_SQUARE is pressed\n"); 
			//
			// check for exit 
			//
			if (btn_select_share) {
				printf("BTN_SELECT is pressed: exiting\n");
				softPwmStop(RIGHT_TRIGGER_PWM_PIN_MINUS);
				softPwmStop(RIGHT_TRIGGER_PWM_PIN_PLUS);
				softPwmStop(LEFT_TRIGGER_PWM_PIN_MINUS);
				softPwmStop(LEFT_TRIGGER_PWM_PIN_PLUS);
				softPwmStop(GAMEPAD_LR_PWM_PIN_MINUS);
				softPwmStop(GAMEPAD_LR_PWM_PIN_PLUS);
				softPwmStop(GAMEPAD_UD_PWM_PIN_MINUS);
				softPwmStop(GAMEPAD_UD_PWM_PIN_PLUS);
				softPwmStop(LJSX_PWM_PIN_MINUS);
				softPwmStop(LJSX_PWM_PIN_PLUS);
				softPwmStop(LJSY_PWM_PIN_MINUS);
				softPwmStop(LJSY_PWM_PIN_PLUS);
				softPwmStop(RJSX_PWM_PIN_MINUS);
				softPwmStop(RJSX_PWM_PIN_PLUS);
				softPwmStop(RJSY_PWM_PIN_MINUS);
				softPwmStop(RJSY_PWM_PIN_PLUS);
				digitalWrite(RJSX_PWM_PIN_PLUS, LOW);
				digitalWrite(RJSX_PWM_PIN_MINUS, LOW);
				digitalWrite(RJSY_PWM_PIN_PLUS, LOW);
				digitalWrite(RJSY_PWM_PIN_MINUS, LOW);
				digitalWrite(LJSX_PWM_PIN_PLUS, LOW);
				digitalWrite(LJSX_PWM_PIN_MINUS, LOW);
				digitalWrite(LJSY_PWM_PIN_PLUS, LOW);
				digitalWrite(LJSY_PWM_PIN_MINUS, LOW);
				digitalWrite(RIGHT_TRIGGER_PWM_PIN_PLUS, LOW);
				digitalWrite(RIGHT_TRIGGER_PWM_PIN_MINUS, LOW);
				digitalWrite(LEFT_TRIGGER_PWM_PIN_PLUS, LOW);
				digitalWrite(LEFT_TRIGGER_PWM_PIN_MINUS, LOW);
				digitalWrite(BTN_TX_PWM_PIN_PLUS, LOW);
				digitalWrite(BTN_TX_PWM_PIN_MINUS, LOW);
				digitalWrite(GAMEPAD_LR_PWM_PIN_PLUS, LOW);
				digitalWrite(GAMEPAD_LR_PWM_PIN_MINUS, LOW);
				digitalWrite(GAMEPAD_UD_PWM_PIN_PLUS, LOW);
				digitalWrite(GAMEPAD_UD_PWM_PIN_MINUS, LOW);

				break;					
			} 
			
		}
	}
	// end while (1) loop
	
	printf("robotArm exits...\n"); 

	
	
	return 0;
}
