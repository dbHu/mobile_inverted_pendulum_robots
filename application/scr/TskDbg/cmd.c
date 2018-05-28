#include "cmd.h"
#include "dbgUart.h"
#include "TskMotor.h"

#include "embARC.h"
#include "embARC_debug.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

VarAndName varname[varnum] = {
	/** origin value */
	{&lqr.k1,					"ka"},
	{&lqr.k2,					"kb"},
	{&lqr.k3,					"kc"},
	{&lqr.k4,					"kd"},
	/** enlarge 128 */
	{&AcclLPFParam,				"lpfa"},
	{&SpdLPFParam,				"lpfv"},
	{&AngLPFParam,				"lpfang"},
	{&AngLPFParam,				"lpfg"},
	{&PRINT_PERIOD,				"putTs"},
	/** enlarge 1024 */
	{&angPid.p,					"angp"},
	{&angPid.i,					"angi"},
	{&angPid.d,					"angd"},
	{&angPid.n,					"angn"},
	/** enlarge 65536 */
	{&ANGLEDIFF,				"angmin"},
	{&ANGLIMIT,					"angmax"},
	{&AngleKal.Q,				"kalq"},
	{&AngleKal.R,				"kalr"},
	{&desire.Velocity,			"lv"},
	{&desire.Timeout,			"ts"},
	{&desire.Acc,				"acc"},
	{&desire.Omega,				"av"},
	{&desire.ThetaZ,			"angz"},
	{&desire.ThetaY,			"angy"}
};

char input[120];
char *cmd;

void paramCorr(void)
{
	/** *d = "space", for string decomposition */
	const char *d = " ";
	bool exit_flag = 0;
	float temp;
	int temp_int;
	MsgType msg;

	putStr("start:  start motor\r\n");
	vTaskDelay(100);
	putStr("stop:   stop motor\r\n");
	vTaskDelay(100);
	putStr("angle:  Enable angle\r\n");
	vTaskDelay(100);
	putStr("stop:   stop motor\r\n");
	vTaskDelay(100);
	putStr("put:    print data\r\n");
	vTaskDelay(100);	
	putStr("time,spd,kal,imu,pwm,lqr\r\n");
	vTaskDelay(100);
	putStr("q:    stop data print\r\n");
	vTaskDelay(100);
	putStr("Format:	str num\r\n");
	vTaskDelay(100);
	putStr("angpid kal lqr desire\r\n");
	vTaskDelay(100);
	putStr("lpfParam, angle range \r\n");
	vTaskDelay(100);
	putStr("print period \r\n");
	vTaskDelay(100);

	while(1){

        putStr("input:");
        vTaskDelay(50);
        
        /** wait for cmd input */
		UartGetLine(input);

		/** string decomposition, obtain command */
		cmd = strtok(input, d);

	    for(int i = 0; i < varnum; i++){
	    	if(!strcmp(cmd, "start")) {
				putStr("start acqZeros\r\n");
	        	vTaskDelay(50);
	        	msg = EnableAcqZeros;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	putStr("please don't touch within 2s\r\n");
	        	vTaskDelay(2000);

	        	putStr("Enable Motor\r\n");
	        	vTaskDelay(50);
	        	msg = DisableAcqZeros;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	msg = EnableMotors;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	msg = EnableAng;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	msg = EnableSeqSet;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	break;
        	}

        	else if(!strcmp(cmd, "stop")) {
				putStr("disable motor\r\n");
	        	vTaskDelay(50);
				msg = DisableMotors;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	break;
        	}

        	else if(!strcmp(cmd, "zero")){
				putStr("Enable acqZeros\r\n");
	        	vTaskDelay(50);
				msg = EnableAcqZeros;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	break;
        	}  

        	else if(!strcmp(cmd, "angle")){
				putStr("Enable angle\r\n");
	        	vTaskDelay(50);
				msg = EnableAng;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	break;
        	}  

        	else if(!strcmp(cmd, "put")){
				putStr("print data\r\n");
	        	vTaskDelay(50);
				msg = EnablePutStr;
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	        	break;
        	}

        	else if(!strcmp(cmd, "dbg")){
				putStr("print lqr param\r\n");
	        	vTaskDelay(50);
				DEBUG_LQR = true;
	        	break;
        	}

        	else if(!strcmp(cmd, "time")){
				putStr("print time\r\n");
	        	vTaskDelay(50);
				PRINT_TIME = true;
	        	break;
        	}

        	else if(!strcmp(cmd, "seq")){
				putStr("print Seq\r\n");
	        	vTaskDelay(50);
				PRINT_SPDQ = true;
	        	break;
        	}

        	else if(!strcmp(cmd, "kal")){
				putStr("print kalman data\r\n");
	        	vTaskDelay(50);
				PRINT_KALMAN = true;
	        	break;
        	}           	

        	else if(!strcmp(cmd, "imu")){
				putStr("print imu data\r\n");
	        	vTaskDelay(50);
				PRINT_IMU = true;
	        	break;
        	}           	

        	else if(!strcmp(cmd, "pwm")){
				putStr("print pwm qei data\r\n");
	        	vTaskDelay(50);
				PRINT_PWM = true;
	        	break;
        	} 

        	else if(!strcmp(cmd, "lqr")){
				putStr("print lqr data\r\n");
	        	vTaskDelay(50);
				PRINT_LQR = true;
	        	break;
        	} 

        	else if(!strcmp(cmd, "q")){
				msg = DisablePutStr;	
	        	xQueueSend(motMbCmd, &msg, portMAX_DELAY);
				putStr("stop data print\r\n");
				PRINT_TIME = false;
				PRINT_SPDQ = false;
				PRINT_KALMAN = false;
				PRINT_IMU = false;
				PRINT_PWM = false;
				PRINT_LQR = false;
				DEBUG_LQR = false;
	        	vTaskDelay(50);
	        	break;
        	}

	    	else if(!strcmp(cmd, varname[i].name)) {  		
	    		vTaskDelay(50);
	    		/** string decomposition, obtain params variation */
	        	cmd = strtok(NULL, d);
	        	temp = atof(cmd);

	        	if(isnan(temp))
	            	temp = *varname[i].value;
	            switch(i){
	            	case 0:
	            	case 1:
	            	case 2:
	            	case 3:
	            		temp_int = (int)temp;
	            		break;
	            	case 4:
	            	case 5:
	            	case 6:
	            	case 7:
	            	case 8:
	            		temp_int = (int16_t)(temp * 128);
	            		break;
	            	case 9:
	            	case 10:
	            	case 11:
	            	case 12:
	            		temp_int = (int)(temp * 1024);
	            		break;
	            	default:
	            		temp_int = (int)(temp * 65536);
	            		break;
	            }

	        	putStr("\t %s = %d (was %d)\r\n", varname[i].name,
	                *varname[i].value + temp_int, *varname[i].value );
	        	vTaskDelay(50);

	        	*varname[i].value += temp_int;
	        	
	        	if(i < 17) putStr("data change after input stop\r\n");
	    		else {
	    			putStr("data change right now\r\n");
	    			msg = EnableSeqSet;	
	        		xQueueSend(motMbCmd, &msg, portMAX_DELAY);
	    		}
	    	}

       		else if(!strcmp(cmd, "exit")) {
            	exit_flag = 1;
            	break;
        	}
        }

        if(exit_flag){
            break;
        }
	}
}         