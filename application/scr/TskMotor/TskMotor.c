#include "TskMotor.h"
#include "dbgUart.h"
#include "timer.h"
#include "imu.h"
#include "spi.h"
#include "FastMath_table.h"
#include "Queue/Queue.h"

#include "embARC.h"
#include "embARC_debug.h"

#include <math.h>

/* 
 * motorTask Priority & stackSize 
 */
const int motorTskPrio = 6;
const int motorTskStkSize = 512;

/*
 * multi-task communication
 */
QueueHandle_t motMbCmd;

/** function control  */
bool AngEnabled = false;
bool AcqZerosEnabled = false;
bool MotorEnabled = false;
bool SeqSetEnabled = false;
bool PutStrEnabled = false;

/** for debug */
bool DEBUG_LQR = false;
bool PRINT_TIME = false;
bool PRINT_SPDQ = false;
bool PRINT_KALMAN = false;
bool PRINT_IMU = false;
bool PRINT_PWM = false;
bool PRINT_LQR = false;

/* 
 * print period, T = PRINT_PERIOD * 2ms
 * dynamically adjusted according to actual demand
 */
int PRINT_PERIOD = 128;

/* 
 * Accl 1-order Lowpass Filter
 * y(n) = (1-a) * y(n-1) + a * x
 *           float         q1.7
 * set a  -1~0.99219 -> -128 ~ 127 
 * 
 * factor 128
 */
int AcclLPFParam = (int)(0.04f * 128);
int SpdLPFParam = (int)(0.02f * 128);
int AngLPFParam = (int)(0.4f * 128);
int GyroLPFParam = (int)(1.f * 128);

/*
 * vertical status starting and termination threshold 
 * 
 * unit:rad factor 65536
 */
int ANGLEDIFF = (int)(0.08f * 65536);
int ANGLIMIT = (int)(0.4f * 65536);

/* kalman params, factor 65536 */
Kalman1Var AngleKal = {
    .Q = 40,            /** 0.0006 * 65536   */
    .R = 6487,          /** 0.099 * 65536    */
    .xpos = 0,
    .Ppos = 0
};

/** lqr params, factor 1*/
lqr_q lqr = {
    .k1 = -6,
    .k2 = -30,
    .k3 = 150,
    .k4 = 3
};

/* 
 * setpoint:
 * 			Velocity --> linear speed
 * 			Timeout  --> period of Velocity
 * 			Acc      --> linear acceleration
 * 			Omega 	 --> (y axial)pitch angular velocity
 * 			ThetaZ	 --> (z axial)yaw angle
 * 			ThetaY 	 --> (y axial)pitch angle
 * Factor:  65536
 * 
 * 			|<-  timeout  ->|
 * 		    -----------------
 * 		   /|				 \
 * 		  /	|Velocity	  	  \-----------> desire.Acc
 *	0m/s / 	|				   \  0m/s
 * -----/  -|-					\------   	 
 * 
 */
VelOmega desire = {
    .Velocity = 0,
    .Timeout = 0,
    .Acc = 32768,
    .Omega = 0,
    .ThetaZ = 0,
    .ThetaY = -29818        /** -0.455 * 65536 */
};

/** pid params, factor 1024, avoid overflow*/
Pid angPid = {
    .p = 1024, 
    .i = 0, 
    .d = 0, 
    .n = 1024,
    .accI = 0,
    .accD = 0       
};

/*
 * for atan LUT, index = SAT16((AcclX / AcclZ) << 7 + 192) (-1.5~0.45-> 0~255)
 *               accl2Ang = Table[index]
 *    ATTENTION: accl2Ang = real_angle << 10  
 */
extern int16_t accl2AngleIndex(ImuValues *imuVals, int16_t *AcclX, int16_t *AcclZ, int16_t *LPFParam);

extern void kalmanPredict(Kalman1Var *kalVal, int32_t u);
extern int32_t kalmanCorrect(Kalman1Var *kalVal, int32_t z);
extern int32_t lqrCalc(lqr_q *lqr, int32_t pos, int32_t vel, int32_t ang, int32_t av);
extern int32_t pidTick(Pid *pid, int32_t diff);

void motorTask(void *pvParameters)
{
    BaseType_t rtn;
    MsgType msg;
    int32_t ercd;
    uint64_t start_us = 0, end_us = 0;

    /** velocity timeout, default value: 65535 * 2ms, about 2 minutes*/
    int timeCnt = 0, timeout = 65535;

    /** angle status check, if less than starting threshold 1s, enter upright mode*/
    int16_t status = 0;

    /*
     * -------------------------|---------------------------
     * 			gyro 			|			accl
     * -------------------------|---------------------------
     * unit: m/s^2 factor: 115	|	unit: m/s^2 factor: 115
     * -------------------------|---------------------------
     * 	angular Velocity 		|	linear acceleration		
     * 		 |	 				|			|
     *    	 |AVG Filter		|			|a_X/a_Y ratio
     *    	 |					|			|
     *  gyro bias				|	pitch angle tan value
     *    	 |					|			|
     *     	 |					|			|kalman Filter
     *     	 |					|			|
     *  gyro relative value	----|--->pitch angle
     *  ------------------------|---------------------------
     */
    ImuValues imuVals;
    int16_t staticCnt = 0, staticCntCyced = 0;
    int16_t GyroZ = 0, GyroY = 0, AcclX = 0, AcclZ = 0;
    int16_t GyroZZero = 0, GyroYZero = 0;
    int16_t accl2Ang = 0, acclIndex = 0;
    int32_t gyroZZeroAcc = 0, gyroYZeroAcc = 0;
    int32_t angle1 = 0, angle = 0, lqrGyroY = 0;

    /*
     * speed and pos related values
     */
    Queue spd;
    Queue ang;
    int16_t qei[2] = {0, 0}, pwm[2] = {0, 0};
    int32_t pwmDeal[2] = {0, 0};
    int16_t encVel = 0;
    int 	angZout = 0, speed = 0, pos = 0, posDes = 0;
    int 	angTmp = 0, spdTmp = 0;
    int32_t lqrOut = 0, pidOut = 0;

    putStr("MOT TIME:%s, %s\r\n",  __DATE__, __TIME__);

    /** initialize spi */
    ercd = cpld_spi_init();    
    configASSERT(ercd == E_OK);

    /** ensure motor stationary*/
    pwm[0] = pwm[1] = 0;
    ercd = spi_write_pwm(pwm);
    configASSERT(ercd == E_OK);

    /** initialize imu */
    ercd = imuInit();
    configASSERT(ercd == E_OK);

    //TODO: add yaw sequence
    /** accelerated sequence, to avoid mutation*/
    q_init_int(&spd, SPDSEQLENMAX);   

    /** yaw sequence, 2rad/s */
    q_init_int(&ang, ANGSEQLENMAX);

    /** lauch 2Ms tick interrupt */
    base_timer_init(); 

    vTaskDelay(10);

    while(1)
    {

    	/** semphore keep 2ms-loop */
     	rtn = xSemaphoreTake(SemMotTick, 3);
        configASSERT(rtn == pdPASS);
        
        if(PRINT_TIME)
            start_us = board_get_cur_us();

        /** read encder */
        ercd = spi_read_qei(qei);
        configASSERT(ercd == E_OK);

        /* 
         * -qei[0] -- R 
         * qei[1]  -- L
         */
        encVel = (-qei[0] + qei[1]) / 2;
        
        /** read imu type:int16_t */
        ercd = imuGetValues(&imuVals);
        configASSERT(ercd == E_OK);

        /** relative value = initial value - bias */
        GyroY = -imuVals.angvY + GyroYZero;
        GyroZ = imuVals.angvZ - GyroZZero;

        /** imu adj zeros, auto adj zeros if static 1.28s */
        if(AcqZerosEnabled)
        {
            staticCnt++;
            staticCntCyced = (staticCnt & 0x0FFF);  // cyced per 8.192s(@Ts=2ms)

            if(staticCntCyced == 127)               // desire static 254ms(@Ts=2ms), adj zero start
            {
                gyroYZeroAcc = 0;
                gyroZZeroAcc = 0;
            }
            else if(staticCntCyced >= 128 && staticCntCyced < 640)
            {
                gyroZZeroAcc += imuVals.angvZ;
                gyroYZeroAcc += imuVals.angvY;
            }
            else if(staticCntCyced == 640)   // adj zero finish
            {
                GyroZZero = gyroZZeroAcc >> 9;
                GyroYZero = gyroYZeroAcc >> 9;
                putStr("Y%d, Z%d\r\n",
                        GyroYZero, GyroZZero);
            }
        }
        else 
            staticCnt = 0;

        if(timeout <= 0)
        {

            timeout = 65535;

            /** decelerated sequence */
            if(desire.Velocity != 0)
            {
                SeqSetEnabled = true;
                desire.Velocity = 0.f;  
            }
        }

        if(SeqSetEnabled)
        {
            SeqSetEnabled = false; 

        	/*
        	 * based on current value, recalculate relative sequence
        	 * update timeout
        	 */
            q_clr(&spd);
            q_clr(&ang);

        	ercd = calcSeq(&spd, spdTmp, desire.Velocity, desire.Acc);
       		configASSERT(ercd < SPDSEQLENMAX);
        
			ercd = calcSeq(&ang, angTmp, desire.ThetaZ, 2 << 16);
			configASSERT(ercd < ANGSEQLENMAX);

            if(desire.Timeout > 0)  timeout = desire.Timeout * TS_RECI >> 16;
        }

        if(AngEnabled)
        {
        	/** calculate angle */
            // acclIndex = accl2AngleIndex(&imuVals, &AcclX, &AcclZ, (int16_t *)&AcclLPFParam);
            // accl2Ang = atan_tables[acclIndex];
            AcclX = ((LPF_FACTOR - AcclLPFParam) * AcclX - AcclLPFParam * imuVals.acclX) / LPF_FACTOR;
            AcclZ = ((LPF_FACTOR - AcclLPFParam) * AcclZ - AcclLPFParam * imuVals.acclZ) / LPF_FACTOR;
            accl2Ang = (int)(atan2f(AcclX, AcclZ) * 1024);
            angle1 = ((LPF_FACTOR - AngLPFParam) * (angle1 + GyroY * Ts) + AngLPFParam * accl2Ang) / LPF_FACTOR;
            kalmanPredict(&AngleKal, GyroY);
            angle = kalmanCorrect(&AngleKal, angle1);
        }

        if(MotorEnabled)
        {
            if(status < 505)
            {   
                /** check start-up status, 0.1rad * 65536 -> 6554 */
                if(((desire.ThetaY - angle) < ANGLEDIFF) && ((angle - desire.ThetaY) < ANGLEDIFF))
                {
                    status++;
                }
                else status = 0;

                /** angle delta value < ANGLEDIFF keep 1s=2ms*500, start controller*/
                if(status == 499)
                {
                    putStr("ON\r\n");
                    AcclX = 0;
                    AcclZ = 0;
                    angle1 = 0;
                    angZout = 0;
                    speed = 0;
                    pos = 0;
                    posDes = 0;
                    spdTmp = 0;
                    angTmp = 0;
                    lqrGyroY = 0;
                    SeqSetEnabled = true; 
                }
            }

            else
            {   
                timeout--;

                /** accumulate yaw angle */
                angZout += GyroZ * GYRO_FACTOR / TS_RECI;
                /** calculate current speed, Low-pass filter*/
                speed = ((LPF_FACTOR - SpdLPFParam) * speed + SpdLPFParam * encVel * ENC_FACTOR) / LPF_FACTOR;
                /** accumulate actual position */
                pos += encVel * ENC_FACTOR / TS_RECI;
                /** dequeue, if queue if empty, keep final value*/
                q_de_int(&spd, &spdTmp);
                q_de_int(&ang, &angTmp);

                /** calcilate setting position */
                posDes += spdTmp / TS_RECI; 

                pidOut = pidTick(&angPid, angTmp - angZout);

                lqrGyroY = ((LPF_FACTOR - GyroLPFParam) * lqrGyroY + GyroLPFParam * GyroY * GYRO_FACTOR) / LPF_FACTOR;
                
                lqrOut = lqrCalc(&lqr,
                                 saturate(posDes - pos, 65000, -65000),
                                 spdTmp - speed,
                                 desire.ThetaY - angle,
                                 desire.Omega - lqrGyroY
                                );

                if(((desire.ThetaY - angle) > ANGLIMIT) || ((angle - desire.ThetaY) > ANGLIMIT))
                {
                    lqrOut = 0;
                    pidOut = 0;
                    /** avoid reset pwm fail */
                    if (status < 510)
                    { 
                        status++;
                    }
                    else{
                        status = 0;
                    }
                }

                /* 
                 * lqrOut / battery voltage   -->   normalization 
                 * pidOut range [-1, 1]
                 * pwm = lqrOut/ Vref + pidOut
                 * saturate pwm to [-65000, 65000], equal to [-0.99, 0.99] << 16
                 * pwm >> 5 to be 12 bits
                 */
                pwmDeal[0] = saturate((lqrOut * 90 >> 10) - pidOut , 65000, -65000);
                pwmDeal[1] = saturate((lqrOut * 90 >> 10) + pidOut , 65000, -65000);
                Asm("asrsr %0, %1, %2" : "=r"(pwm[0]) : "r"(pwmDeal[0]), "r"(5));
                Asm("asrsr %0, %1, %2" : "=r"(pwm[1]) : "r"(pwmDeal[1]), "r"(5));

                ercd = spi_write_pwm(pwm);
                configASSERT(ercd == E_OK);

            }
        }

        if(PutStrEnabled){
            if(timeCnt > PRINT_PERIOD){

                if(PRINT_TIME)
                    putStr("%d\r\n", end_us);

                if(DEBUG_LQR)
                    putStr("%5d, %5d, %5d, %5d\r\n",
                            angle >> 6, lqrOut >> 6, pwmDeal[0], pwm[0]);

                /* approximately 1000 timers large*/
                if(PRINT_LQR)
                    putStr("%5d, %5d, %5d, %5d, %5d, \r\n", 
                            GyroY, angZout / 70 , angle >> 6, pos >> 6, speed >> 6);            

                if(PRINT_PWM){
                    putStr("%5d, %5d, %5d, %5d,\r\n",
                             qei[1], -qei[0], pwm[0], pwm[1]);
                }

                if(PRINT_IMU)
                    putStr("%5d, %5d, %5d, %5d, \r\n", 
                            GyroY, GyroZ, AcclX >> 1, AcclZ >> 1);

                if(PRINT_KALMAN)
                    putStr("%5d, %5d, %5d,\r\n", 
                            GyroY, accl2Ang, angle >> 6);
                                
                if(PRINT_SPDQ){
                    // q_de_int(&spd, &spdTmp);
                    // q_de_int(&ang, &angTmp);
                    // timeout--;
                    putStr("%d %d %d\r\n", 
                            spdTmp >> 6, angTmp >> 6, timeout); 
                }

                timeCnt = 0;
            }
            timeCnt++;

        }

        if(xQueueReceive(motMbCmd, &msg, (TickType_t)0))
        {
            switch(msg & 0xFFFF0000)
            {
            case EnableMotors:
                spdTmp = 0.f;
                angTmp = 0.f;
                MotorEnabled = true;
                break;

            case DisableMotors:
                // write pwm 0
                pwm[0] = 0;
                pwm[1] = 0;
                ercd = spi_write_pwm(pwm);
                configASSERT(ercd == E_OK);

                angPid.accI = 0;
                angPid.accD = 0;

                AngleKal.xpos = 0;
                AngleKal.Ppos = 0;

                status = 0;
                MotorEnabled = false;
                break;

            case EnableAng:
                AngEnabled = true;
                break;
            case DisableAng:
                AngEnabled = false;
                break;
            case EnableAcqZeros:
                AcqZerosEnabled = true;
                break;
            case DisableAcqZeros:
                AcqZerosEnabled = false;
                break;
            case EnableSeqSet:
                SeqSetEnabled = true;
                break;
            case DisableSeqSet:
                SeqSetEnabled = false;
                break;
            case EnablePutStr:
                PutStrEnabled = true;
                break;
            case DisablePutStr:
                PutStrEnabled = false;
                break;
            default:
                break;
            }
        }

        if(PRINT_TIME)
            end_us = board_get_cur_us() - start_us; 
    }

}

void motorInit()
{

	BaseType_t rtn;

    motMbCmd = xQueueCreate(4, sizeof(MsgType));
    configASSERT(motMbCmd);

    // Create tasks
    rtn=xTaskCreate(motorTask, "motorTask", motorTskStkSize, NULL, motorTskPrio, NULL);
    configASSERT(rtn==pdPASS);

}