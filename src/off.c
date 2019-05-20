/**
 * @file rc_blink.c
 * @example    rc_blink
 *
 * This is an example program to demonstrate use of LEDs and button handlers in
 * the Robot Control API. Once started, blink will flash the green and red LEDs.
 * Pressing the mode button will cycle through 3 blinking speeds, slow medium,
 * and fast. Momentarily pressing the pause button will stop and start the
 * blinking by toggling the global state between PAUSED and RUNNING. If the user
 * holds the pause button for more than 1.5 seconds then the blink program will
 * flash the red LED and exit cleanly.
 *
 * This should be used as a reference for how to handle buttons and how to
 * control program flow cleanly utilizing rc_get_state() and rc_set_state().
 **/

#include <stdio.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#define PERILDO	1000	// check every 1/1000 second

int duty_cicle = 50;
int dir = 0;


/**
 * main function sits in one while loop blinking LEDs while button handlers
 * control the blink speed and program state
 */
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()<0){
		fprintf(stderr,"ERROR: failed to complete rc_enable_signal_handler\n");
		return -1;
	}

	// start with both LEDs off
	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 0)==-1){
		fprintf(stderr, "ERROR in rc_blink, failed to set RC_LED_RED\n");
		return -1;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// prepare to run
	rc_set_state(RUNNING);
	
	//int rc_gpio_init (0, ,0) 	
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,0);
	// Run the main loop untill state is EXITING which is set by hitting ctrl-c
	// or holding down the pause button for more than the quit timeout period


	return 0;
}
