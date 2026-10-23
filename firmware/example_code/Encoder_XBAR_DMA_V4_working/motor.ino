#define STBY 16
#define M1_A1 14
#define M1_A2 15
#define M2_A1 17
#define M2_A2 18
#define M1_PWM 9
#define M2_PWM  19
#define PWM_RESOLUTION   12  // 0 - 4095
#define PWM_MAX        4095  // 12 bit
#define PWM_FREQUENCY 30000  // max for 12 bit 36621

void motor_init()
{
        pinMode(M1_A1, OUTPUT);
        pinMode(M1_A2 ,OUTPUT);
        pinMode(M2_A1, OUTPUT);
        pinMode(M2_A2 ,OUTPUT);
        pinMode(STBY, OUTPUT);
        
        digitalWrite(M1_A1,0); // fwd
        digitalWrite(M1_A2,1); // fwd
        digitalWrite(M2_A1,1); // fwd
        digitalWrite(M2_A2,0); // fwd
        digitalWrite(STBY,1);  // enable
        analogWrite(M1_PWM,0); // zero velocity
        analogWrite(M2_PWM,0); // zero velocity

        analogWriteFrequency(M1_PWM, PWM_FREQUENCY); // for 12 bit ideal freq = 36621.09 
        analogWriteFrequency(M2_PWM, PWM_FREQUENCY); // 
        analogWriteResolution(PWM_RESOLUTION);   // 0- 4095
}

void motor_left(int16_t PWM)
{ 
        if(PWM>PWM_MAX)PWM=PWM_MAX;
        if(PWM<-PWM_MAX)PWM=-PWM_MAX;
        
         if (PWM>0)
        {
          digitalWriteFast(M2_A1,1);
          digitalWriteFast(M2_A2,0);
        }
        else
        { PWM = -PWM;
          digitalWriteFast(M2_A1,0);
          digitalWriteFast(M2_A2,1);
        }
         analogWrite(M2_PWM, PWM);
}


void motor_right(int16_t PWM)
{
        if(PWM>PWM_MAX)PWM=PWM_MAX;
        if(PWM<-PWM_MAX)PWM=-PWM_MAX;
     
        if (PWM>0)
        {
          digitalWriteFast(M1_A1,0);
          digitalWriteFast(M1_A2,1);
        }
        else
        { PWM = -PWM;
          digitalWriteFast(M1_A1,1);
          digitalWriteFast(M1_A2,0);
        }
         analogWrite(M1_PWM, PWM);
}
