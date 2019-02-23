#include <unistd.h>
#include <cstdio>
#include <sys/time.h>
#include <Navio2/RCInput_Navio2.h>
#include <Common/Util.h>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include <memory>
#include <stdio.h>
#include <stdint.h>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Common/Util.h>
#include "AHRS.hpp"
#include <Navio2/Led_Navio2.h>

#define READ_FAILED -1
#define FRMOTOR 0
#define FLMOTOR 2
#define BRMOTOR 3
#define BLMOTOR 1

#define PREPARE 0
#define DISARMED 1
#define ARMED 2

#define RATE_CTL 0
#define ANGLE_CTL 1

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define KP 0
#define KI 1
#define KD 2


float pid(float err, int id, int axis);

float Olderr[2][3]={0.0};
float Sum[2][3]={0.0};

//Feedback gain
float Pid_gain[2][3][3]={
    {//rate contorl
        //Kp, Ki, Kd
        {0.01, 0.0, 0.0},//Roll
        {0.01, 0.0, 0.0},//Pitch
        {0.01, 0.0, 0.0} //Yaw
    },
    {//angle control
        //Kp, Ki, Kd
        {0.01, 0.0, 0.0},//Roll
        {0.01, 0.0, 0.0},//Pitch
        {0.01, 0.0, 0.0} //Yaw
        
    }
};


Led_Navio2 led;

std::unique_ptr <RCInput> get_rcin()
{
    auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
    return ptr;
}
    
std::unique_ptr <RCOutput> get_rcout()
{
    auto ptr = std::unique_ptr <RCOutput>{ new RCOutput_Navio2() };
    return ptr;
}

float pid(float err, int id, int axis){

    float u,sum,derr;

    derr = err-Olderr[id][axis];
    Sum[id][axis] = Sum[id][axis]+err;
    sum = Sum[id][axis];
    Olderr[id][axis] = err;

    u += Pid_gain[id][axis][KP] * err;  //P
    u += Pid_gain[id][axis][KI] * sum;  //I
    u += Pid_gain[id][axis][KD] * derr; //D  

    return u ;
}

//============================== Main loop ====================================

void imuLoop(AHRS* ahrs, RCInput* rcin, RCOutput* pwm)
{
    // Orientation data

    float phi, theta, psi;
    float p, q, r;
    struct timeval tv;
    float dt;

    // Timing data

    static float maxdt;
    static float mindt = 0.01;
    static float dtsumm = 0;
    static unsigned long previoustime, currenttime;

    //State Valiable
    
    static int isFirst = 1;
    static int count = 1500;
    static int mode=PREPARE; //mode: 0:not prepare 1:raedy disarmed 2:fligt armed 

    //----------------------- Calculate delta time ----------------------------
    gettimeofday(&tv,NULL);
    previoustime = currenttime;
    currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    dt = (currenttime - previoustime) / 1000000.0;
    if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
    gettimeofday(&tv,NULL);
    currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------
    ahrs->updateIMU(dt);

    //------------------------ Read Euler angles ------------------------------
    ahrs->getEuler(&theta, &phi, &psi);
    ahrs->getGyro(&q, &p, &r);
    r=-r;

    //-------- Get RCInput 
    int Rudder   = rcin->read(0);
    int Elevator = rcin->read(1);
    int Throttle = rcin->read(2);
    int Aileron  = rcin->read(3);
    float Roll, Pitch, Yaw, Thrust;

    //------------------- Discard the time of the first cycle -----------------
    if (!isFirst)
    {
      if (dt > maxdt) maxdt = dt;
      if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //------------- Console and network output with a lowered rate ------------
    dtsumm += dt;
    if(dtsumm > 0.005)
    {
        // Console output
        if(0){
            printf(
                "ROLL: %+7.2f PITCH: %+7.2f YAW: %+7.2f "
                "P: %+7.2f Q: %+7.2f R: %+7.2f "
                "AL: %04d  EL: %04d RD: %04d TH: %04d" 
                "PERIOD %.4fs RATE %dHz \n", 
                phi, theta, psi, 
                p, q, r, 
                Aileron, Elevator, Rudder, Throttle,
                dt, int(1/dt)
            );
        }
        else{
            printf(
                "%+7.2f, %+7.2f, %+7.2f, "
                "%+7.2f, %+7.2f, %+7.2f, "
                "%04d, %04d ,%04d ,%04d, " 
                "%.4f, %d\n", 
                phi, theta, psi, 
                p, q, r, 
                Aileron, Elevator, Rudder, Throttle,
                dt, int(1/dt)
            );
    }

    //Control
    
    float ThMin =1081.0;
    float ThMax =1937.0;
    float AilMin=1079.0;
    float AilMax=1938.0;
    float EleMin=1096.0;
    float EleMax=1962.0;
    float RudMin=1056.0;
    float RudMax=1977.0;
    

    float ThrustCom = ((float)Throttle - ThMin)/(ThMax-ThMin);
    float RollCom   = ((float)Aileron  - ( AilMin + AilMax ) / 2 ) * 2 / ( AilMax - AilMin ) ;
    float PitchCom  = ((float)Elevator - ( EleMin + EleMax ) / 2 ) * 2 / ( EleMax - EleMin ) ; 
    float YawCom    = ((float)Rudder   - ( RudMin + RudMax ) / 2 ) * 2 / ( RudMax - RudMin ) ;

    if (ThrustCom<0.0) ThrustCom=0.0;
    else if (ThrustCom>1.0) ThrustCom=1.0;

    if (RollCom<-1.0) RollCom=-1.0;
    else if (RollCom>1.0) ThrustCom=1.0;

    if (PitchCom<-1.0) PitchCom=-1.0;
    else if (PitchCom>1.0) PitchCom=1.0;

    if (YawCom<-1.0) YawCom=-1.0;
    else if (YawCom>1.0) YawCom=1.0;

    RollCom = RollCom * 45.0;
    PitchCom = PitchCom * 45.0;

    //float ThrustErr = ThrustCom;
    float RollErr   = RollCom  - phi;
    float PitchErr  = PitchCom - theta;    
    float YawErr    = 0.0;//YawCom   - psi;

    //Rate Control PID (Outer Loop)
    float pCom = pid(RollErr,  RATE_CTL, AXIS_X);
    float qCom = pid(PitchErr, RATE_CTL, AXIS_Y);
    float rCom = pid(YawErr,   RATE_CTL, AXIS_Z);
        
    float pErr = pCom - p;
    float qErr = qCom - q;
    float rErr = rCom - r;
         
    //Angle Control  PID (Inner Loop)
    float Roll  = pid(pErr, ANGLE_CTL, AXIS_X);
    float Pitch = pid(qErr, ANGLE_CTL, AXIS_Y);
    float Yaw   = pid(rErr, ANGLE_CTL, AXIS_Z);

    Thrust = ThrustCom;

    float mixing[]={800.0, 80.0, 80.0, 40.0 };
    int FRmot=(int)(Thrust*mixing[0] - Roll*mixing[1] + Pitch*mixing[2] + Yaw*mixing[3]) + 1000;
    int FLmot=(int)(Thrust*mixing[0] + Roll*mixing[1] + Pitch*mixing[2] - Yaw*mixing[3]) + 1000;
    int BRmot=(int)(Thrust*mixing[0] - Roll*mixing[1] - Pitch*mixing[2] - Yaw*mixing[3]) + 1000;
    int BLmot=(int)(Thrust*mixing[0] + Roll*mixing[1] - Pitch*mixing[2] + Yaw*mixing[3]) + 1000;

        if (FRmot<1000) FRmot=1000;
        else if (FRmot>2000) FRmot=2000;
        if (FLmot<1000) FLmot=1000;
        else if (FLmot>2000) FLmot=2000;
        if (BRmot<1000) BRmot=1000;
        else if (BRmot>2000) BRmot=2000;
        if (BLmot<1000) BLmot=1000;
        else if (BLmot>2000) BLmot=2000;


        if ( mode==ARMED ){

	    led.setColor(Colors::Red);  
            pwm->set_duty_cycle(FRMOTOR, FRmot);
            pwm->set_duty_cycle(FLMOTOR, FLmot);
            pwm->set_duty_cycle(BRMOTOR, BRmot);
            pwm->set_duty_cycle(BLMOTOR, BLmot);
	
	}
	if (mode==DISARMED){

	    led.setColor(Colors::Blue);
            pwm->set_duty_cycle(FRMOTOR, 1000);
            pwm->set_duty_cycle(FLMOTOR, 1000);
            pwm->set_duty_cycle(BRMOTOR, 1000);
            pwm->set_duty_cycle(BLMOTOR, 1000);

        }
	if (mode==PREPARE){

	    led.setColor(Colors::Yellow);  
            pwm->set_duty_cycle(FRMOTOR, 1000);
            pwm->set_duty_cycle(FLMOTOR, 1000);
            pwm->set_duty_cycle(BRMOTOR, 1000);
            pwm->set_duty_cycle(BLMOTOR, 1000);

	}
	
        

	
	//Mode Change

	if (count>0)count--;
	
	if (count>0){
		mode = PREPARE;
	}
	else{
		if (mode == PREPARE)
	    		led.setColor(Colors::Blue);
		if      ( Throttle < (int)(ThMin+20.0) && Rudder > (int)(RudMax-20.0) )
			mode = ARMED;
		else if ( Throttle < (int)(ThMin+20.0) && Rudder < (int)(RudMin+20.0) )
			mode = DISARMED;
	}	

	//printf("FR %d, FL %d, BR %d, BL %d, Throttle %d\n",FRmot, FLmot, BRmot, BLmot, Throttle);
	//printf("TH %d, X %d, Y %d, Z %d\n",Throttle, Aileron, Elevator, Rudder);
	//printf("TH %d, X %f, Y %f, Z %f\n",Throttle, RollCom, PitchCom, YawCom);
	//printf("TH %d, X %f, Y %f, Z %f\n",Throttle, RollErr, PitchErr, YawErr);
	//printf("TH %d, X %f, Y %f, Z %f\n",Throttle, phi, theta, psi);


        dtsumm = 0;
    }
}


//=============================================================================
int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }
    std::string sensor_name = "mpu";
    if (sensor_name.empty())
        return EXIT_FAILURE;

    if (!led.initialize())
	    printf("LED Init failure.");


    auto imu = get_inertial_sensor(sensor_name);

#if 0
    if (!imu) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }
#endif
    if (!imu->probe()) {
        printf("Sensor not enable\n");
        return EXIT_FAILURE;
    }


    auto ahrs = std::unique_ptr <AHRS>{new AHRS(move(imu)) };

    //-------------------- Setup gyroscope offset -----------------------------

    ahrs->setGyroOffset();

    auto rcin = get_rcin();
    auto pwm = get_rcout();

    if (getuid()) {
        fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    }

    //------------------- Radio Control setup --------------------------------
    rcin->initialize();
    pwm->initialize(FRMOTOR);
    pwm->initialize(FLMOTOR);
    pwm->initialize(BRMOTOR);
    pwm->initialize(BLMOTOR);

    pwm->set_frequency(FRMOTOR, 300);
    pwm->set_frequency(FLMOTOR, 300);
    pwm->set_frequency(BRMOTOR, 300);
    pwm->set_frequency(BLMOTOR, 300);

    if ( !(pwm->enable(FRMOTOR)) ) {
        return 1;
    }
    if ( !(pwm->enable(FLMOTOR)) ) {
        return 1;
    }
    if ( !(pwm->enable(BRMOTOR)) ) {
        return 1;
    }
    if ( !(pwm->enable(BLMOTOR)) ) {
        return 1;
    }

    if (rcin->read(2)>1900){
        led.setColor(Colors::Yellow);
	printf("Start Motor Calibration\n");

	while( rcin->read(2) > 1100 ){
            pwm->set_duty_cycle(FRMOTOR, 2000);
            pwm->set_duty_cycle(FLMOTOR, 2000);
            pwm->set_duty_cycle(BRMOTOR, 2000);
            pwm->set_duty_cycle(BLMOTOR, 2000);
	}
	//while( rcin->read(2) > 1100 );

    }

    for (int i=0;i<8000;i++){
        pwm->set_duty_cycle(FRMOTOR, 1000);
        pwm->set_duty_cycle(FLMOTOR, 1000);
        pwm->set_duty_cycle(BRMOTOR, 1000);
        pwm->set_duty_cycle(BLMOTOR, 1000);
	usleep(1000);

    }

    led.setColor(Colors::Blue);

    printf("Let's go Fly ! \n");

    //pwm->set_duty_cycle(FRMOTOR, 1000);
    //pwm->set_duty_cycle(FLMOTOR, 1000);
    //pwm->set_duty_cycle(BRMOTOR, 1000);
    //pwm->set_duty_cycle(BLMOTOR, 1000);

    while (true)
    {
        //int FRmot = rcin->read(2);
        //if (FRmot == READ_FAILED) return EXIT_FAILURE;
        //printf("%d\n", FRmot);
        //pwm->set_duty_cycle(FRMOTOR, FRmot);
        usleep(1900);
        imuLoop(ahrs.get(), rcin.get(), pwm.get());
    }
    return 0;
}
