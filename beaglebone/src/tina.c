/**
* @example rc_balance
*
* Reference solution for balancing EduMiP
* Adaptação para estabilização rotacional no eixo Z utiliando a roda de reção do TG da Maria Cristina
**/
#define DEBUG 0

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI
#include <robotcontrol.h>
#include <time.h>
#include <sys/timeb.h> //for time in miliseconds


#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/deprecated.h>
#include <rc/dsm.h>
#include <rc/encoder_eqep.h>
#include <rc/encoder_pru.h>
#include <rc/encoder.h>
#include <rc/gpio.h>
#include <rc/i2c.h>
#include <rc/led.h>
#include <rc/math.h>
#include <rc/mavlink_udp.h>
#include <rc/mavlink_udp_helpers.h>
#include <rc/model.h>
#include <rc/motor.h>
#include <rc/mpu.h>
#include <rc/pinmux.h>
#include <rc/pru.h>
#include <rc/pthread.h>
#include <rc/pwm.h>
#include <rc/servo.h>
#include <rc/spi.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/uart.h>
#include <rc/version.h>
#include <rc/i2c.h>
#include <stdio.h>
#include <rc/time.h>
#include <unistd.h>
#include <linux/i2c.h>


#define BH1750_BUS 	1
#define BH1750_HADD	0x5C
#define BH1750_LADD     0x23	
#define HRES_CONT	0x10 	//Constinuosly H-Resolution Mode
#define HRES2_CONT	0x11	//Constinuosly H-Resolution Mode2	
#define LRES_CONT	0x13	//Constinuosly L-Resolution Mode


#ifndef RC_BALANCE_CONFIG
#define RC_BALANCE_CONFIG

#define SAMPLE_RATE_HZ		100	// main filter and control loop speed

// Structural properties of eduMiP
#define BOARD_MOUNT_ANGLE	0.49 // increase if mip tends to roll forward
#define GEARBOX			35.577
#define ENCODER_RES		60
#define WHEEL_RADIUS_M		0.034
#define TRACK_WIDTH_M		0.035
#define V_NOMINAL		7.4

// inner loop controller 100hz
#define D1_GAIN			1.05
#define D1_ORDER		2
#define D1_NUM			{-4.945, 8.862, -3.967}
#define D1_DEN			{ 1.000, -1.481, 0.4812}
#define D1_NUM_LEN		3
#define D1_DEN_LEN		3
#define D1_SATURATION_TIMEOUT	0.4


// outer loop controller 100hz
#define D2_GAIN			0.9
#define	D2_ORDER		2
#define D2_NUM			{0.18856,  -0.37209,  0.18354}
#define D2_DEN			{1.00000,  -1.86046,   0.86046}
#define D2_NUM_LEN		3
#define D2_DEN_LEN		3
#define THETA_REF_MAX		0.33

// steering controller
#define D3_KP			1.0
#define D3_KI			0.3
#define D3_KD			0.05
#define STEERING_INPUT_MAX	0.5

// electrical hookups
#define MOTOR_CHANNEL_L		3
#define MOTOR_CHANNEL_R		2
#define MOTOR_POLARITY_L	1
#define MOTOR_POLARITY_R	-1
#define ENCODER_CHANNEL_L	3
#define ENCODER_CHANNEL_R	2
#define ENCODER_POLARITY_L	1
#define ENCODER_POLARITY_R	-1

//	drive speeds when using remote control (dsm2)
#define DRIVE_RATE_NOVICE	16
#define TURN_RATE_NOVICE	6
#define DRIVE_RATE_ADVANCED	26
#define TURN_RATE_ADVANCED	10

// DSM channel config
#define DSM_DRIVE_POL		1
#define DSM_TURN_POL		1
#define DSM_DRIVE_CH		3
#define DSM_TURN_CH		2
#define DSM_DEAD_ZONE		0.04

// Thread Loop Rates
#define BATTERY_CHECK_HZ	5
#define SETPOINT_MANAGER_HZ	100
#define PRINTF_HZ		50

// other
#define TIP_ANGLE		0.85
#define START_ANGLE		0.3
#define START_DELAY		0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC		0.7

#endif	// endif RC_BALANCE_CONFIG

#define PERILDO 333     // 3000Hz
#define DT 0.01

#define KP 50
#define KI 10
#define KD 0.1
#define LIMIT 5


//rc_mpu_data_t data;

int dir;
int duty_cicle = 0;

float pot=0;
float err=0;
float derr=0;
float old_err=0;
float ierr=0;
float ref = 0;
float refw = 0;
float omega = 0;
float theta = 0;
clock_t begin, start, end;
uint16_t light_read[360] = {0};
uint16_t hi_data, lo_data;
FILE* f; 


struct timeb timer_msec_i;
long long int  msec_i;
        
        
/**
 * NOVICE: Drive rate and turn rate are limited to make driving easier.
 * ADVANCED: Faster drive and turn rate for more fun.
 */
typedef enum drive_mode_t{
        NOVICE,
        ADVANCED
}drive_mode_t;
/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
        ARMED,
        DISARMED
}arm_state_t;
/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.clock_t start, end;
 */
typedef struct setpoint_t{
        arm_state_t arm_state;  ///< see arm_state_t declaration
        drive_mode_t drive_mode;///< NOVICE or ADVANCED
        double theta;           ///< body lean angle (rad)
        double phi;             ///< wheel position (rad)
        double phi_dot;         ///< rate at which phi reference updates (rad/s)
        double gamma;           ///< body turn angle (rad)
        double gamma_dot;       ///< rate at which gamma setpoint updates (rad/s)
}setpoint_t;
/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
        double wheelAngleR;     ///< wheel rotation relative to body
        double wheelAngleL;
        double theta;           ///< body angle radians
        double phi;             ///< average wheel angle in global frame
        double gamma;           ///< body turn (yaw) angle radians
        double vBatt;           ///< battery voltage
        double d1_u;            ///< output of balance controller D1 to motors
        double d2_u;            ///< output of position controller D2 (theta_ref)
        double d3_u;            ///< output of steering controller D3 to motors
        double mot_drive;       ///< u compensated for battery voltage
} core_state_t;
// possible modes, user selected with command line arguments
typedef enum m_input_mode_t{
        NONE,
        DSM,
        STDIN
} m_input_mode_t;
static void __print_usage(void);
static void __balance_controller(void);         ///< mpu interrupt routine
static void* __setpoint_manager(void* ptr);     ///< background thread
static void* __battery_checker(void* ptr);      ///< background thread
static void* __printf_loop(void* ptr);          ///< background thread
static void* __light_loop(void* ptr);    
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);
static int __wait_for_starting_condition(void);
static void __on_pause_press(void);
static void __on_mode_release(void);
static void* __by_hand_pwm(void* ptr);
static float __mult_abs_val (float a, float b);

// global variables
core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D1 = RC_FILTER_INITIALIZER;
rc_filter_t D2 = RC_FILTER_INITIALIZER;
rc_filter_t D3 = RC_FILTER_INITIALIZER;
rc_mpu_data_t mpu_data;
m_input_mode_t m_input_mode = DSM;
/*
 * printed if some invalid argument was given
 */
static void __print_usage(void)
{
        printf("\n");
        printf("-i {dsm|stdin|none}     specify input\n");
        printf("-h                      print this help message\n");
        printf("\n");
}
/**
 * Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
 void init_sensor(){
   if(rc_i2c_get_lock(BH1750_BUS)){
        	printf("WARNING: i2c bus claimed by another thread\n");
        	printf("Continuing anyway.\n");
        }
        
        // initialize the bus
        if(rc_i2c_init(BH1750_BUS, BH1750_HADD)<0){
        	printf("ERROR: failed to initializmain.ce i2c bus\n");
        	
        }
       if(rc_i2c_init(BH1750_BUS, BH1750_LADD)<0){
        	printf("ERROR: failed to initialize i2c bus\n");
        	
        }  
        rc_i2c_lock_bus(BH1750_BUS);
 }
 
 
 uint16_t read_ligth_sensor(int bus, uint8_t address){
         uint16_t data; 
         rc_i2c_set_device_address(bus, address);
         rc_i2c_read_word(bus, address, &data);
         return data;
 }
 
int main(int argc, char *argv[])
{
        int c;
        if(!ftime(&timer_msec_i)) 
        {
                msec_i = ((long long int) timer_msec_i.time * 1000ll +
                        (long long int) timer_msec_i.millitm);
        }
        else
        {
                msec_i = -1;
        }
        
        time(&begin); 
        time(&start); 
        f = fopen("../log32.csv", "w");
        fprintf(f,"tempo, ref, theta, w, u\n");
        fprintf(f,"0, 1, 2, 3, 4\n");
        //uint32_t initial_time =   timer;
        pthread_t setpoint_thread = 0;
        pthread_t battery_thread = 0;
        pthread_t printf_thread = 0;
        pthread_t light_thread = 0;
        // parse arguments
        opterr = 0;
        // while ((c = getopt(argc, argv, "i:")) != -1){
        //         switch (c){
        //         case 'i': // input option
        //                 if(!strcmp("dsm", optarg)) {
        //                         m_input_mode = DSM;
        //                 } else if(!strcmp("stdin", optarg)) {
        //                         m_input_mode = STDIN;
        //                 } else if(!strcmp("none", optarg)){
        //                         m_input_mode = NONE;
        //                 } else {
        //                         __print_usage();
        //                         return -1;
        //                 }
        //                 break;
        //         case 'h':
        //                 __print_usage();
        //                 return -1;
        //                 break;
        //         default:
        //                 __print_usage();
        //                 return -1;
        //                 break;
        //         }main.c
        // }
        // make sure another instance isn't running
        // if return value is -3 then a background process is running with
        // higher privaledges and we couldn't kill it, in which case we should
        // not continue or there may be hardware conflicts. If it returned -4
        // then there was an invalid argument that needs to be fixed.
        if(rc_kill_existing_process(2.0)<-2) return -1;
        // start signal handler so we can exit cleanly
        if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
        }
        // initialize buttons
        // if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
        //                                         RC_BTN_DEBOUNCE_DEFAULT_US)){
        //         fprintf(stderr,"ERROR: failed to initialize pause button\n");
        //         return -1;
        // }
        // if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
        //                                         RC_BTN_DEBOUNCE_DEFAULT_US)){
        //         fprintf(stderr,"ERROR: failed to initialize mode button\n");
        //         return -1;
        // }
        // Assign functions to be called when button events occur
        // rc_button_set_callbacks(RC_BTN_PIN_PAUSE,__on_pause_press,NULL);
        // rc_button_set_callbacks(RC_BTN_PIN_MODE,NULL,__on_mode_release);
        // initialize enocders
        // if(rc_encoder_eqep_init()==-1){
        //         fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
        //         return -1;
        // }
        // initialize motors
        // if(rc_motor_init()==-1){
        //         fprintf(stderr,"ERROR: failed to initialize motors\n");
        //         return -1;
        // }
        //rc_motor_standby(1); // start with motors in standby
        // start dsm listener
        // if(m_input_mode == DSM){
        //         if(rc_dsm_init()==-1){
        //                 fprintf(stderr,"failed to start initialize DSM\n");
        //                 return -1;
        //         }
        // }
        //initialize adc
        if(rc_adc_init()==-1){
                fprintf(stderr, "failed to initialize adc\n");
        }
        // make PID file to indicate your project is running
        // due to the check made on the call to rc_kill_existing_process() above
        // we can be fairly confident there is no PID file already and we can
        // make our own safely.
        rc_make_pid_file();
        // printf("\nPress and release MODE button to toggle DSM drive mode\n");
        // printf("Press and release PAUSE button to pause/start the motors\n");
        // printf("hold pause button down for 2 seconds to exit\n");
        if(rc_led_set(RC_LED_GREEN, 0)==-1){
                fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
                return -1;
        }
        if(rc_led_set(RC_LED_RED, 1)==-1){
                fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
                return -1;
        }
        // set up mpu configuration
        rc_mpu_config_t mpu_config = rc_mpu_default_config();
        mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
        mpu_config.orient = ORIENTATION_Z_UP;
        //mpu_config.orient = ORIENTATION_Y_UP;
        // if gyro isn't calibrated, run the calibration routine
        if(!rc_mpu_is_gyro_calibrated()){
                printf("Gyro not calibrated, automatically starting calibration routine\n");
                printf("Let your MiP sit still on a firm surface\n");
                rc_mpu_calibrate_gyro_routine(mpu_config);
        }
        // make sure setpoint starts at normal values
        setpoint.arm_state = DISARMED;
        setpoint.drive_mode = NOVICE;
        // set up D1 Theta controller
        // double D1_num[] = D1_NUM;
        // double D1_den[] = D1_DEN;
        // if(rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)){
        //         fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
        //         return -1;
        // }
        // D1.gain = D1_GAIN;
        // rc_filter_enable_saturation(&D1, -1.0, 1.0);
        // rc_filter_enable_soft_start(&D1, SOFT_START_SEC);
        // // set up D2 Phi controller
        // double D2_num[] = D2_NUM;
        // double D2_den[] = D2_DEN;
        // if(rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)){
        //         fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
        //         return -1;
        // }
        // D2.gain = D2_GAIN;
        // rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
        // rc_filter_enable_soft_start(&D2, SOFT_START_SEC);
        // printf("Inner Loop controller D1:\n");
        // rc_filter_print(D1);
        // printf("\nOuter Loop controller D2:\n");
        // rc_filter_print(D2);
        // // set up D3 gamma (steering) controller
        // if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
        //         fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
        //         return -1;
        // }
        // rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);
        // // start a thread to slowly sample battery
        if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
                fprintf(stderr, "failemain.cd to start battery thread\n");
                return -1;
        }
        // wait for the battery thread to make the first readmeasure time in c in minuts
        
        
        
        while(cstate.vBatt<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);
        // start printfmain.c_thread if running from a terminal
        // if it was started as a background process then don't bother
        if(isatty(fileno(stdout))){
                if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
                        fprintf(stderr, "failed to start battery thread\n");
                        return -1;
                }
        }
        
        
        //start light control
        rc_pthread_create(&light_thread, __light_loop, (void*) NULL, SCHED_OTHER, 0);
        // start mpu
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }
        // start balance stack to control setpoints
        // if(rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
        //         fprintf(stderr, "failed to start battery thread\n");
        //         return -1;
        // }
        

        if(rc_pthread_create(&setpoint_thread, __by_hand_pwm, (void*) NULL, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start battery thread\n");
                return -1;
        }

        // this should be the last step in initialization
        // to make sure other setup functions don't interfere
        rc_mpu_set_dmp_callback(&__balance_controller);
        // start in the RUNNING state, pressing the pause button will swap to
        // the PAUSED state then back again.
        //printf("\nHold your MIP upright to begin balancing\n");
        rc_set_state(RUNNING);
        // chill until something exits the program
        rc_set_state(RUNNING);
        while(rc_get_state()!=EXITING){
                rc_usleep(200000);
        }
        // join threads
        rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
        rc_pthread_timed_join(battery_thread, NULL, 1.5);
        rc_pthread_timed_join(printf_thread, NULL, 1.5);
        // cleanup
        rc_filter_free(&D1);
        rc_filter_free(&D2);
        rc_filter_free(&D3);
        rc_mpu_power_off();
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_encoder_eqep_cleanup();
        rc_button_cleanup();    // stop button handlers
        rc_remove_pid_file();   // remove pid file LAST
        
        
        rc_i2c_unlock_bus(BH1750_BUS);
        
        fclose(f);
        return 0;
}
/**
 * This thread is in charge of adjusting the controller setpoint based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(__attribute__ ((unused)) void* ptr)
{
        double drive_stick, turn_stick; // input sticks
        int i, ch, chan, stdin_timeout = 0; // for stdin input
        char in_str[11];
        // wait for mpu to settle
        __disarm_controller();
        rc_usleep(2500000);
        rc_set_state(RUNNING);
        // rc_led_set(RC_LED_RED,0);
        // rc_led_set(RC_LED_GREEN,1);
        while(rc_get_state()!=EXITING){
                // clear out input of old data before waiting for new data
                if(m_input_mode == STDIN) fseek(stdin,0,SEEK_END);
                // sleep at beginning of loop so we can use the 'continue' statement
                rc_usleep(1000000/SETPOINT_MANAGER_HZ);
                // nothing to do if paused, go back to beginning of loop
                if(rc_get_state() != RUNNING || m_input_mode == NONE) continue;
                // if we got here the state is RUNNING, but controller is not
                // necessarily armed. If DISARMED, wait for the user to pick MIP up
                // which will we detected by wait_for_starting_condition()
                if(setpoint.arm_state == DISARMED){
                        if(__wait_for_starting_condition()==0){
                                __zero_out_controller();
                                __arm_controller();
                        }
                        else continue;
                }
                // if dsm is active, update the setpoint rates
                switch(m_input_mode){
                case NONE:
                        continue;
                case DSM:
                        if(rc_dsm_is_new_data()){
                                // Read normalized (+-1) inputs from RC radio stick and multiply by
                                // polarity setting so positive stick means positive setpoint
                                turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
                                drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;
                                // saturate the inputs to avoid possible erratic behavior
                                rc_saturate_double(&drive_stick,-1,1);
                                rc_saturate_double(&turn_stick,-1,1);
                                // use a small deadzone to prevent slow drifts in position
                                if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
                                if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;
                                // translate normalized user input to real setpoint values
                                switch(setpoint.drive_mode){
                                case NOVICE:
                                        setpoint.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
                                        setpoint.gamma_dot =  TURN_RATE_NOVICE * turn_stick;
                                        break;
                                case ADVANCED:
                                        setpoint.phi_dot   = DRIVE_RATE_ADVANCED * drive_stick;
                                        setpoint.gamma_dot = TURN_RATE_ADVANCED  * turn_stick;
                                        break;
                                default: break;
                                }
                        }
                        // if dsm had timed out, put setpoint rates back to 0
                        else if(rc_dsm_is_connection_active()==0){
                                setpoint.theta = 0;
                                setpoint.phi_dot = 0;
                                setpoint.gamma_dot = 0;
                                continue;
                        }
                        break;
                case STDIN:
                        i = 0;
                        while ((ch = getchar()) != EOF && i < 10){
                                stdin_timeout = 0;
                                if(ch == 'n' || ch == '\n'){
                                        if(i > 2){
                                                if(chan == DSM_TURN_CH){
                                                        turn_stick = strtof(in_str, NULL)* DSM_TURN_POL;
                                                        setpoint.phi_dot = drive_stick;
                                                }
                                                else if(chan == DSM_TURN_CH){
                                                        drive_stick = strtof(in_str, NULL)* DSM_DRIVE_POL;
                                                        setpoint.gamma_dot = turn_stick;
                                                }
                                        }
                                        if(ch == 'n') i = 1;
                                        else i = 0;
                                }
                                else if(i == 1){
                                        chan = ch - 0x30;
                                        i = 2;
                                }
                                else{
                                        in_str[i-2] = ch;
                                }
                        }
                        // if it has been more than 1 second since getting data
                        if(stdin_timeout >= SETPOINT_MANAGER_HZ){
                                setpoint.theta = 0;
                                setpoint.phi_dot = 0;
                                setpoint.gamma_dot = 0;
                        }
                        else{
                                stdin_timeout++;
                        }
                        continue;
                        break;
                default:
                        fprintf(stderr,"ERROR in setpoint manager, invalid input mode\n");
                        break;
                }
        }
        // if state becomes EXITING the above loop exists and we disarm here
        __disarm_controller();
        return NULL;
}
/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
 
 static uint16_t __array_argmax(){
        uint16_t max_indx=0;
        uint16_t max=0;
        for(uint16_t i=0; i<360; i++){
                if(max<light_read[i]){
                        max = light_read[i];
                        max_indx = i;
                }
        }
        return max_indx;
}
 
 
 static float __angdiff(float diff){
        return (float)((int)(diff + 180) % 360 - 180);
 }
 
 
static void __balance_controller(void)
{
        static float errw, old_errw,ierrw,derrw;
        /*
        FILE* f = fopen("../log.csv", "a");
        */
        //GETTING TIME IN MILISECONDS
        struct timeb timer_msec;
        long long int  msec;
        if(!ftime(&timer_msec)) 
        {
                msec = ((long long int) timer_msec.time * 1000ll +
                        (long long int) timer_msec.millitm);
        }
        else
        {
                msec = -1;
        }
        
        
        msec-=msec_i;
        /*
        if (((int) (end - start))>15){
            ref = 90.0;
        }
        if (((int) (end - start))>30){
            ref = 0.0;
            time(&start); 
        }
        time(&end); 
        */
        ref =90;
        theta = mpu_data.dmp_TaitBryan[2]*RAD_TO_DEG;
        err = ref-theta;
        //ref = (float)__array_argmax()-180;
        rc_mpu_read_gyro(&mpu_data);
        omega = mpu_data.gyro[2];
        //fprintf(f, "%f, ", omega);
        //fprintf(f, "%f, ", ref);
        //fprintf(f, "%lld \n", msec);
        
        old_err = err;
        old_errw = errw;
        errw = refw - omega;
        //err = __angdiff(ref - theta);
        //err = __angdiff(ref - theta);
        //err = ref-theta;
        //if (fabs(err)<45)
        //        err = (lo_data - hi_data)/65536.0 * 90.0+ theta;
        
        if(fabs(err)<5)
               // err = 0.0;
        if(DEBUG) printf("%f\n",omega);
        
       
        if( ( (err>0)&&(pot<100) ) || ( (err<0)&&(pot>-100) ) )
        {
                ierr += (err +old_err) *DT/2;
        }
        /*
        derr = (err-old_err)/DT;
        
        
        if( ( (errw>0)&&(pot<100) ) || ( (errw<0)&&(pot>-100) ) )
        {
                ierrw += errw*DT;
        }
        
        derrw = (errw-old_errw)/DT;
        
        //pot =  err*KP/30 + derr*KD/5 + ierr*KI/50 ;
        pot = 0.2*err;
        */
        //pot = -0.0683*theta -0.0797*omega + 0.0683*ref;
        pot = -0.0867*theta -0.0757*omega + 0.0258*ierr; //+ 0.0309*ref;
        fprintf(f,"%f, %f, %f, %f, %f\n", (double)(msec/1000.0), ref, theta, omega, pot);
        printf("\rtheta: %9.4f, omega:%9.4f, u: %9.4f err: %9.4f ref: %9.4f", theta, omega, pot, err, ref);
        if(pot>0){
                dir = 0;
                duty_cicle = pot;
        }
        else {
                dir = 1;
                duty_cicle = -pot;
        }
        
        if(duty_cicle > 100) {
                duty_cicle = 100;
        }

        
        return;
}
/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{
        rc_filter_reset(&D1);
        rc_filter_reset(&D2);
        rc_filter_reset(&D3);
        setpoint.theta = 0.0;
        setpoint.phi   = 0.0;
        setpoint.gamma = 0.0;
        rc_motor_set(0,0.0);
        return 0;
}
/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
        rc_motor_standby(1);
        rc_motor_free_spin(0);
        setpoint.arm_state = DISARMED;
        return 0;
}
/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
        __zero_out_controller();
        rc_encoder_eqep_write(ENCODER_CHANNEL_L,0);
        rc_encoder_eqep_write(ENCODER_CHANNEL_R,0);
        // prefill_filter_inputs(&D1,cstate.theta);
        rc_motor_standby(0);
        setpoint.arm_state = ARMED;
        return 0;
}
/**
 * Wait for MiP to be held upright long enough to begin. Returns
 *
 * @return     0 if successful, -1 if the wait process was interrupted by pause
 *             button or shutdown signal.
 */
static int __wait_for_starting_condition(void)
{
        int checks = 0;
        const int check_hz = 20;        // check 20 times per second
        int checks_needed = round(START_DELAY*check_hz);
        int wait_us = 1000000/check_hz;
        // wait for MiP to be tipped back or forward first
        // exit if state becomes paused or exiting
        while(rc_get_state()==RUNNING){
                // if within range, start counting
                if(fabs(cstate.theta) > START_ANGLE) checks++;
                // fell out of range, restart counter
                else checks = 0;
                // waited long enough, return
                if(checks >= checks_needed) break;
                rc_usleep(wait_us);
        }
        // now wait for MiP to be upright
        checks = 0;
        // exit if state becomes paused or exiting
        while(rc_get_state()==RUNNING){
                // if within range, start counting
                if(fabs(cstate.theta) < START_ANGLE) checks++;
                // fell out of range, restart counter
                else checks = 0;
                // waited long enough, return
                if(checks >= checks_needed) return 0;
                rc_usleep(wait_us);
        }
        return -1;
}
/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
        double new_v;
        while(rc_get_state()!=EXITING){
                new_v = rc_adc_batt();
                // if the value doesn't make sense, use nominal voltage
                if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
                cstate.vBatt = new_v;
                rc_usleep(1000000 / BATTERY_CHECK_HZ);
        }
        return NULL;
}

static void* __by_hand_pwm(__attribute__ ((unused)) void* ptr)
{
        rc_state_t last_rc_state, new_rc_state; // keep track of last state
        last_rc_state = rc_get_state();
        while(rc_get_state()!=EXITING){
               if(dir) {
                        rc_led_set(RC_LED_GREEN,1);
                }
                else {
                        rc_led_set(RC_LED_GREEN,0);
                }
                
                
                //liga
                if(duty_cicle)
                {
                        rc_led_set(RC_LED_RED,1);
                        rc_usleep((PERILDO*duty_cicle)/100);
                }
             
                //desliga
                rc_led_set(RC_LED_RED,0);
                rc_usleep((PERILDO*(100-duty_cicle))/100); 
               
               //if (DEBUG) printf("\tpwm\n");
        }
        return NULL;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
        rc_state_t last_rc_state, new_rc_state; // keep track of last state
        last_rc_state = rc_get_state();
        while(rc_get_state()!=EXITING){
                new_rc_state = rc_get_state();
                // check if this is the first time since being paused
                // if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
                //         printf("\nRUNNING: Hold upright to balance.\n");
                //         printf("    θ    |");
                //         printf("  θ_ref  |");
                //         printf("    φ    |");
                //         printf("  φ_ref  |");
                //         printf("    γ    |");
                //         printf("  D1_u   |");
                //         printf("  D3_u   |");
                //         printf("  vBatt  |");
                //         printf("arm_state|");
                //         printf("\n");
                // }
                // else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
                //         printf("\nPAUSED: press pause again to start.\n");
                // }
                // last_rc_state = new_rc_state;
                // // decide what to print or exit
                // if(new_rc_state == RUNNING){
                //         printf("\r");
                //         printf("%7.3f  |", cstate.theta);
                //         printf("%7.3f  |", setpoint.theta);
                //         printf("%7.3f  |", cstate.phi);
                //         printf("%7.3f  |", setpoint.phi);
                //         printf("%7.3f  |", cstate.gamma);
                //         printf("%7.3f  |", cstate.d1_u);
                //         printf("%7.3f  |", cstate.d3_u);
                //         printf("%7.3f  |", cstate.vBatt);
                //         if(setpoint.arm_state == ARMED) printf("  ARMED  |");
                //         else printf("DISARMED |");
                //         fflush(stdout);
                // }
                rc_usleep(1000000 / PRINTF_HZ);
        }
        return NULL;
}

#define EULER           2.718282
static float __mult_abs_val (float a, float b)
{
        if (a == 0.0) return b;
        else if (b == 0.0) return a;
        else 
        {
                return (pow(EULER, fabs(log(a/b)))); 
        }
}





#define KP_LIGHT        10
static void* __light_loop(__attribute__ ((unused)) void* ptr)
{
        init_sensor();
        float err_light = 0.0;
        float dist = 0.0;
        rc_state_t last_rc_state, new_rc_state; // keep track of last state
        last_rc_state = rc_get_state();
        while(rc_get_state()!=EXITING){
                new_rc_state = rc_get_state();
                //hi_data = read_ligth_sensor(1, BH1750_HADD);
                //lo_data = read_ligth_sensor(1, BH1750_LADD);
                //printf("%u, %u, ", hi_data, lo_data);
                dist = __mult_abs_val(hi_data/1.2, lo_data*1.4);
                //printf("%.2f, ", dist);
        	err_light = dist <= 2 ? 0.0 : ((lo_data*1.4)-(hi_data/1.2));
                refw = err_light*KP_LIGHT;
                //ref = (30.0 - theta);
                //printf("%.2f\n", ref);
                light_read[(int)theta+180] = (hi_data+lo_data) >>1;
                rc_usleep(16000);
        }
        return NULL;
}
/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
static void __on_pause_press(void)
{
        int i=0;
        const int samples = 100;        // check for release 100 times in this period
        const int us_wait = 2000000; // 2 seconds
        switch(rc_get_state()){
        // pause if running
        case EXITING:
                return;
        case RUNNING:
                rc_set_state(PAUSED);
                __disarm_controller();
                rc_led_set(RC_LED_RED,1);
                rc_led_set(RC_LED_GREEN,0);
                break;
        case PAUSED:
                rc_set_state(RUNNING);
                __disarm_controller();
                rc_led_set(RC_LED_GREEN,1);
                rc_led_set(RC_LED_RED,0);
                break;
        default:
                break;
        }
        // now wait to see if the user want to shut down the program
        while(i<samples){
                rc_usleep(us_wait/samples);
                if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
                        return; //user let go before time-out
                }
                i++;
        }
        printf("long press detected, shutting down\n");
        //user held the button down long enough, blink and exit cleanly
        rc_led_blink(RC_LED_RED,5,2);
        rc_set_state(EXITING);
        return;
}
/**
 * toggle between position and angle modes if MiP is paused
 */
static void __on_mode_release(void)
{
        // toggle between position and angle modes
        if(setpoint.drive_mode == NOVICE){
                setpoint.drive_mode = ADVANCED;
                printf("using drive_mode = ADVANCED\n");
        }
        else {
                setpoint.drive_mode = NOVICE;
                printf("using drive_mode = NOVICE\n");
        }
        rc_led_blink(RC_LED_GREEN,5,1);
        return;
}