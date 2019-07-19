
#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI
#include <robotcontrol.h>

#define MOTOR_CHANNEL_L 2
#define LOOP_FREQ 100 // 100hz
#define ENCRAD	338 	//by running from rc_test_encoders_eqep

rc_mpu_data_t mpu_data;

double input[]={0,0,0};
double output[]={0,0,0};
double setpoint,current;
static void follow_controller(void);

int main(int argc, char *argv[])
{
        double duty = 0.0;
        int c, in;
        int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;

        // set up D1 Theta controller


        rc_mpu_config_t mpu_config = rc_mpu_default_config();
        mpu_config.dmp_sample_rate = LOOP_FREQ;

        if(rc_encoder_eqep_init()){
                fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
                return -1;
        }
        if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
                fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
                rc_led_blink(RC_LED_RED, 5, 5);
                return -1;
        }
        if(rc_motor_init_freq(500)) return -1; // way fater than 100hz loop

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

static void follow_controller(void)
{
        setpoint=(double)-rc_encoder_eqep_read(3)/ENCRAD;
        current=(double)rc_encoder_eqep_read(2)/ENCRAD;

        //shift input outputs
        output[2]=output[1];
        output[1]=output[0];
        input[2]=input[1];
        input[1]=input[0];
        input[0]=setpoint-current; // input the error

        double dutyL; // from -1 to 1,

        if(rc_get_state()==EXITING){
                rc_motor_set(0,0.0);
                return;
        }
//iter1
//  0.5313 z^2 - 0.3351 z - 0.1639
//  ------------------------------
//     z^2 - 0.3511 z - 0.6487
//iter2
 // 1.562 z^2 - 1.021 z - 0.4936
 // ----------------------------
 //   z^2 - 0.4081 z - 0.5722
 //iter3
//  0.05 z^2 - 0.03267 z - 0.01579
// ------------------------------
//    z^2 - 0.4081 z - 0.5722
// 0.2092 z^2 - 0.1599 z - 0.04535
// -------------------------------
//     z^2 - 0.3947 z - 0.5854
// 0.2858 z^2 - 0.1587 z - 0.1207
// ------------------------------
//    z^2 - 0.3947 z - 0.5854
// 0.7144 z^2 - 0.3967 z - 0.3018
// ------------------------------
//    z^2 - 0.3947 z - 0.5854
        output[0]=(double)0.3947*output[1] + 0.5854*output[2] + 0.7144*input[0] -0.3967*input[1] - 0.3018*input[2];
        // diffrence eq
        dutyL=output[0]; // scale by the max nomonal voltage

        printf("\rduty:%.4f set:%.4f follow:%.4f",dutyL,setpoint,current);
        fflush(stdout);

        rc_motor_set(MOTOR_CHANNEL_L, dutyL);
        return;
}
