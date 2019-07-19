#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI
#include <robotcontrol.h>
#include "joystick.hh"
#include <unistd.h>

#define MOTOR_CHANNEL_L 2
#define MOTOR_CHANNEL_R 3

#define ENCRAD	338 	//by running from rc_test_encoders_eqep
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

/** ** ** ** ** ** ** ** ** ** ** ** ** Begin HOMEWORK CODEBLOCK  ** ** ** ** ** ** ** ** **/
#define LOOP_FREQ 200 // 100hz
#define FILTFREQ 200.0
#define fcutoff 20.0 // in rad/s
#define RC (1/fcutoff)

// function declarations
void on_pause_press();
void on_pause_release();
double aRC = (double)(RC / (RC + (1/FILTFREQ)));

static void imu_interupt_function(void);         ///< mpu interrupt routine

rc_mpu_data_t mpu_data;
double accelAngle, gyroAngle, synthAngle, MPUAngle; // the result vars to be displayed

double input[]={0,0,0};
double output[]={0,0,0};
double input2[]={0,0,0};
double output2[]={0,0,0};
double setpoint,setpoint2=0,current,current2L,current2R;
double duty=0.0; // from -1 to 1,
double tempout,tempout2;
double inner[]={-1.8127,0.8127,-3.9246,7.0740,-3.1827};
double tf[]={-1.0478,0.2696,0.0441,0.0016,-0.0425};
double stickGain=2,steerGain=0.5;
int joyval,steerval;
static void follow_controller(void);
static void* print_thread(void* ptr);
static void* outer_loop(void* ptr);
double comp,steer_set;
Joystick joystick("/dev/input/js0");

int main(int argc, char *argv[])
{
        tf[0] = atof(argv[1]);
        tf[1] = atof(argv[2]);
        tf[2] = atof(argv[3]);
        tf[3] = atof(argv[4]);
        tf[4] = atof(argv[5]);
        stickGain = atof(argv[6]);
        steerGain = atof(argv[7]);
        printf("TF: %.4f %.4f %.4f %.4f %.4f\n",tf[0],tf[1],tf[2],tf[3],tf[4]);

        // Ensure that it was found and that we can use it
        if (!joystick.isFound())
        {
          printf("open failed.\n");
          exit(1);
        }

        int in;
        int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;

        // set up D1 Theta controller
        pthread_t thread1 = 0;
        if(rc_pthread_create(&thread1,print_thread, (void*) NULL, SCHED_OTHER, 0)){
                      fprintf(stderr, "failed to start thread1\n");
                      return -1;
              }
        pthread_t thread2 = 0;
        if(rc_pthread_create(&thread2,outer_loop, (void*) NULL, SCHED_OTHER, 0)){
                      fprintf(stderr, "failed to start thread2n");
                      return -1;
              }
        //pthread_t thread2 = 0;

        rc_mpu_config_t mpu_config = rc_mpu_default_config();
        mpu_config.dmp_sample_rate = LOOP_FREQ;
        mpu_config.orient=ORIENTATION_Y_UP;

        if(rc_encoder_eqep_init()){
                fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
                return -1;
        }
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }
        if(rc_motor_init()) return -1; // way fater than 100hz loop

        rc_mpu_set_dmp_callback(&follow_controller);
        // initialize hardware first
        // decide what to do

        // wait untill the user exits
        rc_set_state(RUNNING);
        while(rc_get_state()!=EXITING){
                rc_usleep(200000);
        }
        // final cleanup
        printf("\ncalling cleanup()\n");
        rc_motor_cleanup();
        rc_encoder_eqep_cleanup();
        return 0;
}
void calculate_angle(){
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
	gyroAngle += mpu_data.gyro[0]/FILTFREQ;
	MPUAngle=mpu_data.dmp_TaitBryan[TB_PITCH_X];
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
static void follow_controller(void)
{
        calculate_angle();
        //setpoint=-0.30; // 90 degrees
        current=synthAngle;
        //shift input outputs
        output[2]=output[1];
        output[1]=output[0];
        input[2]=input[1];
        input[1]=input[0];
        input[0]=(setpoint-current); // input the error

        if(rc_get_state()==EXITING){
                rc_motor_set(0,0.0);
                return;
        }

        tempout=(double) -inner[0]*output[1] -inner[1]*output[2] + inner[2]*input[0] + inner[3]*input[1] + inner[4]*input[2];
        if (tempout>=1)
          output[0]=1;
        else if (tempout<=-1)
          output[0]=-1;
        else
          output[0]=tempout;

        // diffrence eq
        duty=output[0]; // scale by the max nomonal voltage
        rc_motor_set(MOTOR_CHANNEL_L, -(duty-comp));
        rc_motor_set(MOTOR_CHANNEL_R, (duty+comp));

        JoystickEvent event;
        if (joystick.sample(&event))
        {
          if (event.isAxis())
          {	if(event.number==1)
              joyval=event.value;
            if(event.number==2)
              steerval=-event.value;

          }
        }
        return;

}
static void* outer_loop(void* ptr){
  while(1){
    //setpoint2=0; // 90 degrees
    current2L=(double)-rc_encoder_eqep_read(2)/ENCRAD;
    current2R=(double)rc_encoder_eqep_read(3)/ENCRAD;
    output2[2]=output2[1];
    output2[1]=output2[0];
    input2[2]=input2[1];
    input2[1]=input2[0];
    input2[0]=(setpoint2-(current2L+current2R)/2); // input the error

    tempout2=(double) -tf[0]*output2[1] -tf[1]*output2[2] + tf[2]*input2[0] + tf[3]*input2[1] + tf[4]*input2[2] ;
    setpoint=tempout2-0.3;
    output2[0]=tempout2;
    rc_usleep(50000);
  }
}


static void* print_thread(void* ptr){
  while(1){
    printf("\r comp %.4f duty:%.4f joy%i set_p:%.4f set_a:%.4f mpu:%.4f encoder:%.4f temp:%.4f",comp,duty,joyval,setpoint2,setpoint,current,(current2L+current2R),tempout);
    fflush(stdin);
    rc_usleep(100000);

    steer_set+=((double)steerval/32768)*steerGain;
    comp=(current2L-current2R-steer_set)/10;
    setpoint2+=((double)-joyval/32768)*stickGain;
  }
}
