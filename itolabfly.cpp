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
#include <sys/resource.h>


#define READ_FAILED -1
#define FRMOTOR 0
#define FLMOTOR 2
#define BRMOTOR 3
#define BLMOTOR 1
#define MOTSTOP 1000

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

#define FREQ 200.0

float pid(float err, int id, int axis);

float Olderr[2][3]={0.0};
float Sum[2][3]={0.0};

//Feedback gain
float Pid_gain[2][3][3]={
    {//rate contorl
        //Kp, Ki, Kd
        {0.025, 0.0002, 0.04},//Roll
        {0.025, 0.0002, 0.04},//Pitch
        {0.02, 0.0001, 0.00} //Yaw
    },
    {//angle control
        //Kp, Ki, Kd
        {2.5, 0.0, 0.0},//Roll
        {2.5, 0.0, 0.0},//Pitch
        {1.5, 0.0, 0.0} //Yaw
        
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
    if (Sum[id][axis]>500000.0) Sum[id][axis]=500000.0;
    else if (Sum[id][axis]<-500000.0) Sum[id][axis]=-500000.0;
    sum = Sum[id][axis];
    Olderr[id][axis] = err;

    u = 0.0;
    u += Pid_gain[id][axis][KP] * err;  //P
    u += Pid_gain[id][axis][KI] * sum;  //I
    u += Pid_gain[id][axis][KD] * derr; //D  

    return u ;
}

void resetSum(void){

    for (int i=0; i<2; i++){
        for (int j=0; j<3; j++) {
            Sum[i][j]=0.0;
        }
    }
}

void resetErr(void){
    for (int i=0; i<2; i++){
        for (int j=0; j<3; j++) {
            Olderr[i][j]=0.0;
        }
    }
}

float Olddata[3]={0.0};

float filter(int axis, float dt, float data){
    float y,T,f;
    f=20.0;//[Hz]
    T=1.0/f/2/3.14159;
    y=(T * Olddata[axis] + dt * data)/(T + dt);
    Olddata[axis]=y;
    return y;
}
void motor_control(RCOutput* pwm, int rr, int rl, int br, int bl){
    pwm->set_duty_cycle(FRMOTOR, rr);
    pwm->set_duty_cycle(FLMOTOR, rl);
    pwm->set_duty_cycle(BRMOTOR, br);
    pwm->set_duty_cycle(BLMOTOR, bl);
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
    static float dtsumm = 0.0;
    static unsigned long previoustime, currenttime;
    static float total_time=0.0;

    //State Valiable
    
    static int isFirst = 1;
    static int count = 1500;
    static int mode=PREPARE; //mode: 0:PREPARE 1:Disarmed 2:Armed 

    //----------------------- Calculate delta time ----------------------------
    gettimeofday(&tv,NULL);
    previoustime = currenttime;
    currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    dt = (currenttime - previoustime) / 1000000.0;
    //if ( dt < 0.001 )  usleep((0.001-dt)*1000000);
    //gettimeofday(&tv,NULL);
    //currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
    //dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS -------------
    ahrs->updateIMU(dt);

    //------------------------ Read Euler angles ------------------------------
    ahrs->getEuler(&theta, &phi, &psi);
    ahrs->getGyro(&q, &p, &r);
    r=-r;
    
    //filtering
    p=filter(AXIS_X, dt, p);
    q=filter(AXIS_Y, dt, q);
    r=filter(AXIS_Z, dt, r);

    //-------- Get RCInput
     
    int Rudder   = 0;
    int Elevator = 0;
    int Throttle = 0;
    int Aileron  = 0;
    Rudder   = rcin->read(0);
    Elevator = rcin->read(1);
    Throttle = rcin->read(2);
    Aileron  = rcin->read(3);
    float Roll, Pitch, Yaw, Thrust;

    //------------------- Discard the time of the first cycle -----------------
    if (!isFirst)
    {
      if (dt > maxdt) maxdt = dt;
      if (dt < mindt) mindt = dt;
    }
    else{
        dt =0.0;
        isFirst = 0;
        printf(
            "time, "
            "Roll, Pitch, Yaw, "
            "p, q, r, "
            "RollCom, PitchCom, YawCom, "
            "pCom, qCom, rCom, "
            "Ail, Ele, Rud, Thro, "
            "FRmot, FLmot, BRmot, BLmot, "
            "dtsum, freq"
            "\n"
        );
        return;
    }

    //------------- Console and network output with a lowered rate ------------
    dtsumm = dt;
    if(1)//dtsumm > 1.0/FREQ )
    {
    

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
        YawCom *=45.0;

        //float ThrustErr = ThrustCom;
        float RollErr   = RollCom  - phi;
        float PitchErr  = PitchCom - theta;    
        float YawErr    = YawCom;

        //Angle Control PID (Outer Loop)
        float pCom = pid(RollErr,  ANGLE_CTL, AXIS_X);
        float qCom = pid(PitchErr, ANGLE_CTL, AXIS_Y);
        float rCom = pid(YawErr,   ANGLE_CTL, AXIS_Z);
        
        float pErr = pCom - p;
        float qErr = qCom - q;
        float rErr = rCom - r;
         
        //Rate Control  PID (Inner Loop)
        float Roll  = pid(pErr, RATE_CTL, AXIS_X);
        float Pitch = pid(qErr, RATE_CTL, AXIS_Y);
        float Yaw   = pid(rErr, RATE_CTL, AXIS_Z);

        Thrust = ThrustCom;

        float mixing[]={800.0, 80.0, 80.0, 40.0 };
        int FRmot=(int)(Thrust*mixing[0] - Roll*mixing[1] + Pitch*mixing[2] + Yaw*mixing[3]) + 1000;
        int FLmot=(int)(Thrust*mixing[0] + Roll*mixing[1] + Pitch*mixing[2] - Yaw*mixing[3]) + 1000;
        int BRmot=(int)(Thrust*mixing[0] - Roll*mixing[1] - Pitch*mixing[2] - Yaw*mixing[3]) + 1000;
        int BLmot=(int)(Thrust*mixing[0] + Roll*mixing[1] - Pitch*mixing[2] + Yaw*mixing[3]) + 1000;

        if (FRmot<1000) FRmot=1000;
        else if (FRmot>1900) FRmot=1900;
        if (FLmot<1000) FLmot=1000;
        else if (FLmot>1900) FLmot=1900;
        if (BRmot<1000) BRmot=1000;
        else if (BRmot>1900) BRmot=1900;
        if (BLmot<1000) BLmot=1000;
        else if (BLmot>1900) BLmot=1900;

#if 1
        if ( mode==ARMED ){

            led.setColor(Colors::Red);
            if(Throttle < ThMin+50){
                resetSum();
                resetErr();
                motor_control(pwm, MOTSTOP, MOTSTOP, MOTSTOP, MOTSTOP);
            }
            else{
                motor_control(pwm, FRmot, FLmot, BRmot, BLmot);
            }
        }
        if (mode==DISARMED){

            led.setColor(Colors::Blue);
            motor_control(pwm, MOTSTOP, MOTSTOP, MOTSTOP, MOTSTOP);
            resetSum();
            resetErr();
        }
        
        if (mode==PREPARE){

            led.setColor(Colors::Yellow);  
            motor_control(pwm, MOTSTOP, MOTSTOP, MOTSTOP, MOTSTOP);
            resetSum();
            resetSum();
            resetErr();
        }
#endif        
        //Mode Change

        if (count>0)count--;
    
        if (count>0){
            mode = PREPARE;
        }
        else{

            if (mode == PREPARE)
                led.setColor(Colors::Blue);
            if ( Throttle < (int)(ThMin+20.0) && Rudder > (int)(RudMax-20.0) )
                mode = ARMED;
            else if ( Throttle < (int)(ThMin+20.0) && Rudder < (int)(RudMin+20.0) )
                mode = DISARMED;
        }

        // Console output
#if 1
        
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
                "%.4f, "
                "%+7.3f, %+7.3f, %+7.3f, "
                "%+7.3f, %+7.3f, %+7.3f, "
                "%+7.3f, %+7.3f, %+7.3f, "
                "%+7.3f, %+7.3f, %+7.3f, "
                "%04d, %04d ,%04d ,%04d, " 
                "%04d, %04d ,%04d ,%04d, " 
                "%.4f, %d\n", 
                total_time, 
                phi, theta, psi, 
                p, q, r,
                RollCom, PitchCom, YawCom,
                pCom, qCom, rCom, 
                Aileron, Elevator, Rudder, Throttle,
                FRmot, FLmot, BRmot, BLmot,
                dtsumm, int(1/dtsumm)
            );
        }
        //printf("FR %d, FL %d, BR %d, BL %d, Throttle %d\n",FRmot, FLmot, BRmot, BLmot, Throttle);
        //printf("TH %d, X %d, Y %d, Z %d\n",Throttle, Aileron, Elevator, Rudder);
        //printf("TH %d, X %f, Y %f, Z %f\n",Throttle, RollCom, PitchCom, YawCom);
        //printf("TH %d, X %f, Y %f, Z %f\n",Throttle, RollErr, PitchErr, YawErr);
        //printf("TH %d, X %f, Y %f, Z %f\n",Throttle, phi, theta, psi);
#endif
        //printf("%f,%f,%f,%f,%f,%f,%f\n",total_time,p,np,q,nq,r,nr);

        total_time+=dtsumm;
        dtsumm = 0.0;
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

    pwm->set_frequency(FRMOTOR, (int)FREQ );
    pwm->set_frequency(FLMOTOR, (int)FREQ );
    pwm->set_frequency(BRMOTOR, (int)FREQ );
    pwm->set_frequency(BLMOTOR, (int)FREQ );

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
            motor_control(pwm.get() ,2000, 2000, 2000, 2000);
        }

    }

    for (int i=0;i<8000;i++){
        motor_control(pwm.get(), MOTSTOP, MOTSTOP, MOTSTOP, MOTSTOP);
        usleep(1000);
    }

    led.setColor(Colors::Blue);
    printf("Let's go Fly ! \n");

    //Main Loop
    setpriority(PRIO_PROCESS,0,-20);

    while (true)
    {
        imuLoop(ahrs.get(), rcin.get(), pwm.get());
        usleep((int)((1.0/FREQ-0.0012)*1000000));
    }
    return 0;
}
