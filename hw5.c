/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <math.h>
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

/** ** ** ** ** ** ** ** ** ** ** ** ** Begin HOMEWORK CODEBLOCK  ** ** ** ** ** ** ** ** **/
#define FREQ 200.0
#define fcutoff 20.0 // in rad/s
#define RC (1/fcutoff)

// function declarations
void on_pause_press();
void on_pause_release();
double aRC = (double)(RC / (RC + (1/FREQ)));

static void imu_interupt_function(void);         ///< mpu interrupt routine
static void* printerThread(void* ptr);     ///< background thread

rc_mpu_data_t mpu_data;
double accelAngle, gyroAngle, synthAngle, MPUAngle; // the result vars to be displayed

 /** ** ** ** ** ** ** ** ** ** ** ** ** END HOMEWORK CODEBLOCK  ** ** ** ** ** ** ** ** **/
int main()
{
	pthread_t thread1 = 0;

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

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	// start balance stack to control setpoints
        if(rc_pthread_create(&thread1,printerThread, (void*) NULL, SCHED_OTHER, 0)){
                fprintf(stderr, "failed to start thread1\n");
                return -1;
        }


	 // set up mpu configuration
        rc_mpu_config_t mpu_config = rc_mpu_default_config();
				mpu_config.dmp_fetch_accel_gyro=1;
        mpu_config.dmp_sample_rate = FREQ; //hz to execute imu interupt
        mpu_config.orient = ORIENTATION_Y_UP;
	  // start mpu
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }
				// use defaults for now, except also enable magnetometer.


	printf("\nPress and release pause button to turn green LED on and off\n");
	printf("hold pause button down for 2 seconds to exit\n");

	 // this should be the last step in initialization
        // to make sure other setup functions don't interfere
        rc_mpu_set_dmp_callback(&imu_interupt_function);
	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
		}
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// always sleep at some point
		rc_usleep(1000000);
	}

	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);

	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}

void imu_interupt_function(){
	double Y,Z, accel;
	static double gyro;
	static double accelLastOut,gyroLastIn,gyroLastOut; // the filter computation vars
	double accelFilt, gyroFilt;
	//printf("\n imu interupt executing every 0.2 seconds\n");
	if(rc_mpu_read_accel(&mpu_data)<0){
          printf("read accel data failed\n");
  }
  if(rc_mpu_read_gyro(&mpu_data)<0){
          printf("read gyro data failed\n");
	}
	Y = mpu_data.accel[1];
	Z = mpu_data.accel[2];
	accelAngle  = atan2(Y,Z)*180*M_1_PI-90;
	gyroAngle += mpu_data.gyro[0]/FREQ;
	MPUAngle=mpu_data.dmp_TaitBryan[TB_PITCH_X]*180*M_1_PI;
	// filters
  //y[i] := y[i-1] + α * (x[i] - y[i-1])
	accelFilt= accelLastOut + aRC * (accelAngle - accelLastOut); // LPF
	//y[i] := α * (y[i-1] + x[i] - x[i-1])
	gyroFilt = aRC * (gyroLastOut + gyroAngle - gyroLastIn);  // HPF
	//printf("LPF yn: %.4f LPF yn-1: %.4f LPF xn: %.4f \n",accelFilt, accelLastOut, accelAngle);
	//printf("aRC:%.4f HPF yn: %.4f HPF yn-1: %.4f HPF xn: %.4f HPF xn-1: %.4f\n",aRC,gyroFilt, gyroLastOut, gyroAngle, gyroLastIn);

	synthAngle=accelFilt+gyroFilt;
	// update for next cycle
	accelLastOut=accelFilt;
	gyroLastIn = gyroAngle;
	gyroLastOut = gyroFilt;
}

static void* printerThread(__attribute__ ((unused)) void* ptr){
	while(rc_get_state()!=EXITING){
		//printf(ANSI_COLOR_CYAN "\nthread 1 executing every .05 second, 20Hz\n" ANSI_COLOR_RESET);
		double diff= MPUAngle-synthAngle;
	//	printf(ANSI_COLOR_CYAN "Accel: %.4f Gyro: %.4f Synth: %.4f DMP: %.4f DIFF: %.4f\n",accelAngle, gyroAngle, synthAngle, MPUAngle, diff);
		printf(ANSI_COLOR_CYAN "%.4f , %.4f , %.4f , %.4f \n",accelAngle, gyroAngle, synthAngle, MPUAngle);

		rc_usleep(50000);
	}
	return NULL;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** END HOMEWORK CODEBLOCK  ** ** ** ** ** ** ** ** **/







/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
