#include <DMAChannel.h>

#define tick_us 0.0000004266 // measured
#define encoder_slots 164.0f
#define wheel_circum_mm 67.50f

float mm_per_slot = wheel_circum_mm/encoder_slots;

static DMAChannel dma_M1_A;
static DMAChannel dma_M1_B;
static DMAChannel dma_M2_A;
static DMAChannel dma_M2_B;
uint16_t dma_buffer_M1_A[200];
uint16_t dma_buffer_M1_B[200];
uint16_t dma_buffer_M2_A[200];
uint16_t dma_buffer_M2_B[200];

IMXRT_TMR_t * TMR1 = (IMXRT_TMR_t *)&IMXRT_TMR1;
IMXRT_TMR_t * TMR2 = (IMXRT_TMR_t *)&IMXRT_TMR2;
IMXRT_TMR_t * TMR3 = (IMXRT_TMR_t *)&IMXRT_TMR3;
IMXRT_TMR_t * TMR4 = (IMXRT_TMR_t *)&IMXRT_TMR4;

uint16_t buffer_capture_cnt =100;
uint16_t buffer_byte_cnt = 200;

void dma_setup() {
  
        dma_M1_A.TCD->SADDR =&TMR1_CAPT0 ;// source addr  tmr0_ch0
        dma_M1_A.TCD->ATTR_SRC =1; // 16 bit transfer
        dma_M1_A.TCD->SOFF =0;// source inc 0
        dma_M1_A.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(2) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M1_A.TCD->SLAST =0 ;// no source adj at end
        dma_M1_A.TCD->DADDR =  &dma_buffer_M1_A;         // M1_phase A
        dma_M1_A.TCD->DOFF = 2;  // dest addr inc bytes
        dma_M1_A.TCD->ATTR_DST = 1; // 16 bit dest write
        dma_M1_A.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M1_A.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M1_A.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M1_A.TCD->CSR =0;
        dma_M1_A.triggerAtHardwareEvent(48); //48 = dma mux for timer 1 ch0 capture event
        dma_M1_A.enable();

        dma_M1_B.TCD->SADDR =&TMR2_CAPT0 ;// source addr  tmr0_ch1
        dma_M1_B.TCD->ATTR_SRC =1; // 16 bit transfer
        dma_M1_B.TCD->SOFF =0;// source inc 0
        dma_M1_B.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(2) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M1_B.TCD->SLAST =0 ;// no source adj at end
        dma_M1_B.TCD->DADDR =  &dma_buffer_M1_B;          // M1_phase B
        dma_M1_B.TCD->DOFF = 2;  // dest addr inc bytes
        dma_M1_B.TCD->ATTR_DST = 1; // 16 bit dest write
        dma_M1_B.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M1_B.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M1_B.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M1_B.TCD->CSR =0;
        dma_M1_B.triggerAtHardwareEvent(112); //112 = dma mux for timer 2 capture event 
        dma_M1_B.enable();

        
    //    dma_M2_A.TCD->SADDR =&TMR3_CAPT0 ;// source addr  tmr0_ch0
        dma_M2_A.TCD->SADDR =&TMR3_CAPT1 ;// source addr  tmr0_ch0
        dma_M2_A.TCD->ATTR_SRC =1; // 16 bit transfer
        dma_M2_A.TCD->SOFF =0;// source inc 0
        dma_M2_A.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(2) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M2_A.TCD->SLAST =0 ;// no source adj at end
        dma_M2_A.TCD->DADDR =  &dma_buffer_M2_A;         // M2_phase A
        dma_M2_A.TCD->DOFF = 2;  // dest addr inc bytes
        dma_M2_A.TCD->ATTR_DST = 1; // 16 bit dest write
        dma_M2_A.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M2_A.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M2_A.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M2_A.TCD->CSR =0;
       // dma_M2_A.triggerAtHardwareEvent(56); //56 = dma mux for timer 3 ch0 capture event
        dma_M2_A.triggerAtHardwareEvent(57); //57 = dma mux for timer 3 ch1 capture event
        dma_M2_A.enable();

        dma_M2_B.TCD->SADDR =&TMR4_CAPT0 ;// source addr  tmr0_ch1
        dma_M2_B.TCD->ATTR_SRC =1; // 16 bit transfer
        dma_M2_B.TCD->SOFF =0;// source inc 0
        dma_M2_B.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(2) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M2_B.TCD->SLAST =0 ;// no source adj at end
        dma_M2_B.TCD->DADDR =  &dma_buffer_M2_B;          // M2_phase B
        dma_M2_B.TCD->DOFF = 2;  // dest addr inc bytes
        dma_M2_B.TCD->ATTR_DST = 1; // 16 bit dest write
        dma_M2_B.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M2_B.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M2_B.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M2_B.TCD->CSR =0;
        dma_M2_B.triggerAtHardwareEvent(120); //120 = dma mux for timer 4capture event 
        dma_M2_B.enable();
}

void XBAR_setup()
{
    // xbar and pin alt mux for pin0 and 1 for quad encoder and capture
     IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 1 ; // alt1 pin0
     IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 1 ; // alt1 pin1
     IOMUXC_XBAR1_IN17_SELECT_INPUT =1;         // xbar17 select for ad_B0_03_alt1
     IOMUXC_XBAR1_IN16_SELECT_INPUT =0;        // xbar16 select for ad_B0_02_alt1

     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_17); // inputphA
     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_16); // inputPhB
     IOMUXC_GPR_GPR6 |= (IOMUXC_GPR_GPR6_QTIMER1_TRM0_INPUT_SEL); // output tmr0_0
     IOMUXC_GPR_GPR6 |= (IOMUXC_GPR_GPR6_QTIMER2_TRM0_INPUT_SEL); // output tmr2_0
     
     XBARA1_SEL43 = 16;   //in 16  out to tmr1_ch0 (xbar86) 
     XBARA1_SEL45 = 17;   //in 17  out to tmr2_ch0 (xbar90) 
     XBARA1_SEL33 = 16 |(17<<8);  // in 16 ,17  out to phaseA phaseBquad 1 (xbar66 and 67)

     // xbar and pin alt mux for pin2 and 3 for quad encoder and capture
     IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 3 ; // alt3 pin2
     IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 3 ; // alt3 pin3
    // IOMUXC_XBAR1_IN6_SELECT_INPUT =0;         // xbar17 select for ad_B0_03_alt1
    // IOMUXC_XBAR1_IN7_SELECT_INPUT =0;        // xbar16 select for ad_B0_02_alt1

     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_6); // inputphA
     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_7); // inputPhB
     IOMUXC_GPR_GPR6 |= (IOMUXC_GPR_GPR6_QTIMER3_TRM0_INPUT_SEL); // output tmr0_0
     IOMUXC_GPR_GPR6 |= (IOMUXC_GPR_GPR6_QTIMER4_TRM0_INPUT_SEL); // output tmr2_0
     
     XBARA1_SEL47 = 6;   //in 6  out to tmr3_ch0 (xbar86) 
     XBARA1_SEL49 = 7;   //in 7  out to tmr3_ch0 (xbar90) 
     XBARA1_SEL35 = 6<<8;  //  phaseA quad 2(xbar71)
     XBARA1_SEL36 = 7;     //  phaseB quad 2(xbar72)
}

void clock_setup()
{
     CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);  // xbar clock
     CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);  // timer1 clock
     CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);  // timer2 clock
     CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);  // timer3 clock
     CCM_CCGR6 |= CCM_CCGR6_QTIMER4(CCM_CCGR_ON);  // timer4 clock
     CCM_CCGR4 |= CCM_CCGR4_ENC1(CCM_CCGR_ON);  // encoder clock
}

void timer_setup()
{
     // timer0_ch0 for M1_phaseA capture
     TMR1->CH[0].CTRL =0;   // stop counter
     TMR1->CH[0].CNTR =0; // counter
     TMR1->CH[0].COMP1 = 0xFFFF; // compare
     TMR1->CH[0].LOAD =0; // reload
     // 0xE clk/64
     TMR1->CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xD)  | TMR_CTRL_SCS(0);
     TMR1->CH[0].SCTRL =   TMR_SCTRL_CAPTURE_MODE(3) | TMR_SCTRL_IEF ; 
     TMR1->CH[0].CSCTRL |= TMR_CSCTRL_ROC ;  // reset count on capture
     TMR1->CH[0].DMA |= TMR_DMA_IEFDE;  // dma out trigger from timer

     // timer2_ch0 for M1_phaseB capture
     TMR2->CH[0].CTRL =0;   // stop counter
     TMR2->CH[0].CNTR =0; // counter
     TMR2->CH[0].COMP1 = 0xFFFF; // compare
     TMR2->CH[0].LOAD =0; // reload
     // 0xE clk/64
     TMR2->CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xD)  | TMR_CTRL_SCS(0);
     TMR2->CH[0].SCTRL =   TMR_SCTRL_CAPTURE_MODE(3) | TMR_SCTRL_IEF ; 
     TMR2->CH[0].CSCTRL |= TMR_CSCTRL_ROC ;  // reset count on capture
     TMR2->CH[0].DMA |= TMR_DMA_IEFDE;  // dma out trigger from timer

       // timer3_ch0 for M2_phaseA capture
     TMR3->CH[1].CTRL =0;   // stop counter
     TMR3->CH[1].CNTR =0; // counter
     TMR3->CH[1].COMP1 = 0xFFFF; // compare
     TMR3->CH[1].LOAD =0; // reload
     // 0xE clk/64
     TMR3->CH[1].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xD)  | TMR_CTRL_SCS(0);
     TMR3->CH[1].SCTRL =   TMR_SCTRL_CAPTURE_MODE(3) | TMR_SCTRL_IEF ; 
     TMR3->CH[1].CSCTRL |= TMR_CSCTRL_ROC ;  // reset count on capture
     TMR3->CH[1].DMA |= TMR_DMA_IEFDE;  // dma out trigger from timer

      // timer0_ch1 for M1_phaseB capture
     TMR4->CH[0].CTRL =0;   // stop counter
     TMR4->CH[0].CNTR =0; // counter
     TMR4->CH[0].COMP1 = 0xFFFF; // compare
     TMR4->CH[0].LOAD =0; // reload
     // 0xE clk/64
     TMR4->CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xD)  | TMR_CTRL_SCS(0);
     TMR4->CH[0].SCTRL =   TMR_SCTRL_CAPTURE_MODE(3) | TMR_SCTRL_IEF ; 
     TMR4->CH[0].CSCTRL |= TMR_CSCTRL_ROC ;  // reset count on capture
     TMR4->CH[0].DMA |= TMR_DMA_IEFDE;  // dma out trigger from timer
  
}


void test()
{
    motor_left(300);
    delay(1000);
    motor_left(0);
    delay(1000);
    while(1);
  
}




void setup()
{
  Serial.begin(115200);

   
     clock_setup();
     XBAR_setup();
     timer_setup();
     dma_setup();
     motor_init();

   //  test();
    
}


void loop()
{
   
    int16_t speed_pwm =0;
    float dma_vel_M1_A =0;
    float dma_vel_M1_B =0;
     float dma_vel_M2_A =0;
    float dma_vel_M2_B =0;
    float enc_vel =0;
    uint16_t enc_count_new =0;
    uint16_t enc_count_old =0;
    float sum_M1_A=0;
    float sum_M1_B=0;
    float sum_M2_A=0;
    float sum_M2_B=0;
    long loopTime_old = micros();
    long loopTime =2000;
    int16_t speed_inc = 4;

    
      while(1)
      {         
               while(micros() < (loopTime_old+loopTime)){;}
               loopTime_old=micros();
              motor_left(speed_pwm);
              motor_right(speed_pwm);
              
                  speed_pwm += speed_inc;
                  if(speed_pwm > 500)speed_inc=-speed_inc;
                   if(speed_pwm < -500)speed_inc=-speed_inc;

               if(Serial.available())
               {char inp=Serial.read();
               if(inp=='a')speed_pwm +=100;
               if(inp=='z')speed_pwm -=100;
                
               }
                              
               dma_M1_A.disable();
               dma_M1_B.disable();
               dma_M2_A.disable();
               dma_M2_B.disable();
               uint16_t buffSize_M1_A = buffer_capture_cnt - dma_M1_A.TCD->CITER;
               uint16_t buffSize_M1_B = buffer_capture_cnt - dma_M1_B.TCD->CITER;
               uint16_t buffSize_M2_A = buffer_capture_cnt - dma_M2_A.TCD->CITER;
               uint16_t buffSize_M2_B = buffer_capture_cnt - dma_M2_B.TCD->CITER;
        
               if(buffSize_M1_A!=0)
               {
                     sum_M1_A = 0;
                     for(int x=0;x<buffSize_M1_A;x++)
                           {
                            sum_M1_A += dma_buffer_M1_A[x];
                           }
                     dma_vel_M1_A = mm_per_slot/((sum_M1_A/buffSize_M1_A)*tick_us);
               }
               
               if(buffSize_M1_B!=0)
               {
                     sum_M1_B = 0;
                     for(int x=0;x<buffSize_M1_B;x++)
                           {
                            sum_M1_B += dma_buffer_M1_B[x];
                           }
                     dma_vel_M1_B = mm_per_slot/((sum_M1_B/buffSize_M1_B)*tick_us);
               }

                 
               if(buffSize_M2_A!=0)
               {
                     sum_M2_A = 0;
                     for(int x=0;x<buffSize_M2_A;x++)
                           {
                            sum_M2_A += dma_buffer_M2_A[x];
                           }
                     dma_vel_M2_A = mm_per_slot/((sum_M2_A/buffSize_M2_A)*tick_us);
               }
               
               if(buffSize_M2_B!=0)
               {
                     sum_M2_B = 0;
                     for(int x=0;x<buffSize_M2_B;x++)
                           {
                            sum_M2_B += dma_buffer_M2_B[x];
                           }
                     dma_vel_M2_B = mm_per_slot/((sum_M2_B/buffSize_M2_B)*tick_us);
               }
                 
                  
                    
                     enc_count_old=enc_count_new;
                     enc_count_new = ENC1_LPOS/2;
                     enc_vel = (mm_per_slot*(enc_count_new-enc_count_old))/0.002;
                    
                     
                     int32_t enc1_u_read =  ENC1_UPOS;
                     enc1_u_read = (enc1_u_read<<16) | ENC1_LPOSH;
                     float distance = mm_per_slot *( enc1_u_read);

                     Serial.printf("%0.1f\t",dma_vel_M1_A);
                     Serial.printf("%0.1f\t",dma_vel_M1_B);
                     Serial.printf("%0.1f\t",dma_vel_M2_A);
                     Serial.printf("%0.1f\t",dma_vel_M2_B);
                    
                   
                     Serial.println("");
                     dma_M1_A.TCD->CITER = buffer_capture_cnt;
                     dma_M1_B.TCD->CITER = buffer_capture_cnt;
                     dma_M2_A.TCD->CITER = buffer_capture_cnt;
                     dma_M2_B.TCD->CITER = buffer_capture_cnt;
                     dma_M1_A.TCD->DADDR = &dma_buffer_M1_A;
                     dma_M1_B.TCD->DADDR = &dma_buffer_M1_B;
                     dma_M2_A.TCD->DADDR = &dma_buffer_M2_A;
                     dma_M2_B.TCD->DADDR = &dma_buffer_M2_B;
                     dma_M1_A.enable();
                     dma_M1_B.enable();
                     dma_M2_A.enable();
                     dma_M2_B.enable();
      }


  
}



     
     
