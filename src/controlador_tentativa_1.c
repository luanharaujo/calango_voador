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
#define DEBUG 1

#include <stdio.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/mpu.h>
#include <getopt.h>
#include <signal.h>

#define I2C_BUS 2

#define PERILDO	333	// 3000Hz
#define DT 0.000333333333

#define KP 900
#define KI 700
#define KD 0.1

// possible modes, user selected with command line arguments
typedef enum g_mode_t{
	G_MODE_RAD,
	G_MODE_DEG,
	G_MODE_RAW
} g_mode_t;
static int enable_warnings = 0;

int dir;
int duty_cicle = 0;

float pot=0;
float err=0;
float derr=0;
float old_err=0;
float ierr=0;
float ref = 0;
float omega = 0;


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
	rc_mpu_data_t data; //struct to hold new data
	g_mode_t g_mode = G_MODE_DEG; // gyro default to degree mode.
	
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.show_warnings = enable_warnings;
	
	//FILE *fp;
	//float tempo=0;
	//fp = fopen("log.csv","w");
    //fprintf(fp,"KP, KI, KD\n%f, %f, %f\nTempo(ms), Omega(gras/s) , pot(%%)\n", KP, KI, KD);
	
	
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
	
		
	rc_mpu_initialize(&data, conf);
	
	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// prepare to run
	rc_set_state(RUNNING);
	
	//int rc_gpio_init (0, ,0) 	
	
	// Run the main loop untill state is EXITING which is set by hitting ctrl-c
	// or holding down the pause button for more than the quit timeout period
	while(rc_get_state()!=EXITING){
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		rc_mpu_read_gyro(&data);
		omega = -data.gyro[2];
		
		old_err = err;
		err = ref - omega;
		
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		if( ( (err>0)&&(pot<100) ) || ( (err<0)&&(pot>-100) ) )
		{
			ierr += err*DT;
		}
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		derr = (err-old_err)/DT;
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		pot = err*KP + derr*KD + ierr*KI;
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		if(pot>0){
			dir = 1;
			duty_cicle = pot;
		}
		else {
			dir = 0;
			duty_cicle = -pot;
		}
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		if(pot > 100) {
			pot = 100;
		}
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		if(dir) {
			rc_led_set(RC_LED_GREEN,1);
		}
		else {
			rc_led_set(RC_LED_GREEN,0);
		}
		
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		//liga
		if(duty_cicle)
		{
			rc_led_set(RC_LED_RED,1);
			rc_usleep((PERILDO*duty_cicle)/100);
		}
		//fprintf(fp,"%f, %f, %d\n", tempo, omega, pot);
		//desliga
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		rc_led_set(RC_LED_RED,0);
		rc_usleep((PERILDO*(100-duty_cicle))/100);
		//tempo+=DT;
		if(DEBUG) printf("%lu ", rc_nanos_thread_time());
		printf("!\n");
		
	
	}
	//fclose(fp);
	return 0;
}
