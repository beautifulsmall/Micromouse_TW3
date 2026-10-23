#include <DMAChannel.h>

//#define tick_us 0.0000004266 // measured
#define tick_us 0.000000125f // 8MHZ 24/3   


#define encoder_slots 164.0f
#define wheel_circum_mm 67.50f

float mm_per_slot = wheel_circum_mm/encoder_slots;

static DMAChannel dma_M1_A;
static DMAChannel dma_M1_B;
static DMAChannel dma_M2_A;
static DMAChannel dma_M2_B;
uint32_t dma_buffer_M1_A[200];
uint32_t dma_buffer_M1_B[200];
uint32_t dma_buffer_M2_A[200];
uint32_t dma_buffer_M2_B[200];

uint16_t buffer_capture_cnt =200;
uint16_t buffer_byte_cnt = 4*buffer_capture_cnt;

void dma_setup() {
  
        dma_M1_A.TCD->SADDR = &GPT1_CNT ;// source addr  gpt1 32bit
        dma_M1_A.TCD->ATTR_SRC =2; // 32 bit transfer
        dma_M1_A.TCD->SOFF =0;// source inc 0
        dma_M1_A.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M1_A.TCD->SLAST =0 ;// no source adj at end
        dma_M1_A.TCD->DADDR =  &dma_buffer_M1_A;         // M1_phase A
        dma_M1_A.TCD->DOFF = 4;  // dest addr inc bytes
        dma_M1_A.TCD->ATTR_DST = 2; // 16 bit dest write
        dma_M1_A.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M1_A.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M1_A.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M1_A.TCD->CSR =0;
        dma_M1_A.triggerAtHardwareEvent(30); //48 = dma mux for timer 1 ch0 capture event
        dma_M1_A.enable();

        dma_M1_B.TCD->SADDR = &GPT1_CNT ;// source addr  gpt1 32bit ;// source addr  tmr0_ch1
        dma_M1_B.TCD->ATTR_SRC =2; // 32 bit transfer
        dma_M1_B.TCD->SOFF =0;// source inc 0
        dma_M1_B.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M1_B.TCD->SLAST =0 ;// no source adj at end
        dma_M1_B.TCD->DADDR =  &dma_buffer_M1_B;          // M1_phase B
        dma_M1_B.TCD->DOFF = 4;  // dest addr inc bytes
        dma_M1_B.TCD->ATTR_DST = 2; // 16 bit dest write
        dma_M1_B.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M1_B.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M1_B.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M1_B.TCD->CSR =0;
        dma_M1_B.triggerAtHardwareEvent(31); //112 = dma mux for timer 2 capture event 
        dma_M1_B.enable();

        
    //    dma_M2_A.TCD->SADDR =&TMR3_CAPT0 ;// source addr  tmr0_ch0
        dma_M2_A.TCD->SADDR = &GPT1_CNT ;// source addr  gpt1 32bit ;// source addr  tmr0_ch0
        dma_M2_A.TCD->ATTR_SRC =2; // 32 bit transfer
        dma_M2_A.TCD->SOFF =0;// source inc 0
        dma_M2_A.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M2_A.TCD->SLAST =0 ;// no source adj at end
        dma_M2_A.TCD->DADDR =  &dma_buffer_M2_A;         // M2_phase A
        dma_M2_A.TCD->DOFF = 4;  // dest addr inc bytes
        dma_M2_A.TCD->ATTR_DST = 2; // 16 bit dest write
        dma_M2_A.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M2_A.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M2_A.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M2_A.TCD->CSR =0;
       // dma_M2_A.triggerAtHardwareEvent(56); //56 = dma mux for timer 3 ch0 capture event
        dma_M2_A.triggerAtHardwareEvent(94); //57 = dma mux for timer 3 ch1 capture event
        dma_M2_A.enable();

        dma_M2_B.TCD->SADDR = &GPT1_CNT ;// source addr  gpt1 32bit 
        dma_M2_B.TCD->ATTR_SRC =2; // 32 bit transfer
        dma_M2_B.TCD->SOFF =0;// source inc 0
        dma_M2_B.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |
                  DMA_TCD_NBYTES_MLOFFYES_MLOFF(0);
        dma_M2_B.TCD->SLAST =0 ;// no source adj at end
        dma_M2_B.TCD->DADDR =  &dma_buffer_M2_B;          // M2_phase B
        dma_M2_B.TCD->DOFF = 4;  // dest addr inc bytes
        dma_M2_B.TCD->ATTR_DST = 2; // 16 bit dest write
        dma_M2_B.TCD->BITER = buffer_capture_cnt;  // 16 bit buff
        dma_M2_B.TCD->CITER = buffer_capture_cnt;
        // ring buffer re-start buffer if capture cnt =0
        dma_M2_B.TCD->DLASTSGA = -buffer_byte_cnt;  
        dma_M2_B.TCD->CSR =0;
        dma_M2_B.triggerAtHardwareEvent(95); //120 = dma mux for timer 4capture event 
        dma_M2_B.enable();
}

void XBAR_setup()
{
    // xbar and pin alt mux for pin0 and 1 for quad encoder and capture
     IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03 = 1 ; // alt1 pin0
     IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 = 1 ; // alt1 pin1
     IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 3 ; // alt1 pin2
     IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 3 ; // alt1 pin3
    
     
     IOMUXC_XBAR1_IN17_SELECT_INPUT =1;         // xbar17 select for ad_B0_03_alt1
     IOMUXC_XBAR1_IN16_SELECT_INPUT =0;        // xbar16 select for ad_B0_02_alt1

     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_17); // inputphA
     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_16); // inputPhB
     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_6); // inputphA
     IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_7); // inputPhB
     
     
     XBARA1_SEL0 = 16 |(17<<8);   //in 16,17  out to xbar 0 , 1
     XBARA1_SEL1 = 6 |(7<<8);     //in 6,7    out to xbar 2 , 3
     XBARA1_SEL33 = 16 |(17<<8);  // in 16 ,17  out to phaseA phaseB quad1 (xbar66 and 67)
     XBARA1_SEL35 = 6<<8;  //  phaseA quad2 2(xbar71)
     XBARA1_SEL36 = 7;     //  phaseB quad2 2(xbar72)

    
      // dma trigger enable
      XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(3) | XBARA_CTRL_DEN0; 
      XBARA1_CTRL0 |= XBARA_CTRL_STS1 | XBARA_CTRL_EDGE1(3) | XBARA_CTRL_DEN1;  
      XBARA1_CTRL1 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(3) | XBARA_CTRL_DEN0; 
      XBARA1_CTRL1 |= XBARA_CTRL_STS1 | XBARA_CTRL_EDGE1(3) | XBARA_CTRL_DEN1;  
    

    
}

void clock_setup()
{
     CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);  // xbar clock
     CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON); // gpt1 cock
     CCM_CCGR4 |= CCM_CCGR4_ENC1(CCM_CCGR_ON);  // encoder clock
     CCM_CCGR4 |= CCM_CCGR4_ENC2(CCM_CCGR_ON);  // encoder clock
}

void timer_setup()
{
  GPT1_CR = 0;
    // 3. Clear status flags (write 1 to clear)
  GPT1_SR = 0x3F;
   // 4. Set prescaler (PR register is divide by N+1)
  // Prescale 24 MHz clock by 3 => 8 MHz
  GPT1_PR = 3 - 1;
   // 5. Configure Control Register (CR)
      GPT1_CR = GPT_CR_EN /* Enable timer */ 

          | GPT_CR_CLKSRC(1) /* 24 MHz peripheral clock */
          | GPT_CR_FRR /* Free-Run mode (do not reset at compare) */;
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
            int32_t enc_count_new =0;
            int32_t enc_count_old =0;
            float sum_M1_A=0;
            float sum_M1_B=0;
            float sum_M2_A=0;
            float sum_M2_B=0;
            long loopTime_old = micros();
            long loopTime =2000;
            int16_t speed_inc = 4;


       uint16_t buff_M1_A_index =  0;
       uint16_t buff_M1_B_index =  0;
       uint16_t buff_M2_A_index =  0;
       uint16_t buff_M2_B_index =  0;
       uint16_t buff_M1_A_index_old =  0;
       uint16_t buff_M1_B_index_old =  0;
       uint16_t buff_M2_A_index_old =  0;
       uint16_t buff_M2_B_index_old =  0;
       uint32_t buffStart =0;
       uint32_t buffEnd =0;
       uint32_t time_new_M1_A =0;
       uint32_t time_old_M1_A =0;
       uint32_t time_new_M1_B =0;
       uint32_t time_old_M1_B =0;

    
      while(1)
      {         
               while(micros() < (loopTime_old+loopTime)){;}
               loopTime_old=micros();
               motor_left(speed_pwm);
              //motor_right(speed_pwm);
              
                  speed_pwm += speed_inc;
                  if(speed_pwm > 2000)speed_inc=-speed_inc;
                  if(speed_pwm < 0)speed_inc=-speed_inc;

               if(Serial.available())
               {
                  char inp=Serial.read();
                  if(inp=='a')speed_pwm +=100;
                  if(inp=='z')speed_pwm -=100;
               }
                    
               // index counts down from buffer_capture_cnt          
               buff_M1_A_index_old =  buff_M1_A_index;
               buff_M1_B_index_old =  buff_M1_B_index;
               buff_M2_A_index_old =  buff_M2_A_index;
               buff_M2_B_index_old =  buff_M2_B_index;
               
               buff_M1_A_index =  dma_M1_A.TCD->CITER;
               buff_M1_B_index =  dma_M1_B.TCD->CITER;
               buff_M2_A_index =  dma_M2_A.TCD->CITER;
               buff_M2_B_index =  dma_M2_B.TCD->CITER;
        
     ///////////////////////////////// buffer M1A  ///////////////////////////////////////////        

             if(buff_M1_A_index < buff_M1_A_index_old)  // counts down from max to 1
               {  
                  sum_M1_A = 0;
                  buffStart = buffer_capture_cnt - buff_M1_A_index_old;
                  buffEnd =   buffer_capture_cnt - buff_M1_A_index;
              //    Serial.print("M1_PHA ");

                  for(int x = buffStart; x < buffEnd; x++)
                        {     time_old_M1_A=time_new_M1_A;
                              time_new_M1_A=dma_buffer_M1_A[x];
                              sum_M1_A += time_new_M1_A- time_old_M1_A;
                              
                        }

                  dma_vel_M1_A = mm_per_slot/((sum_M1_A/(buffEnd-buffStart))*tick_us);
                //  Serial.print(dma_vel_M1_A );
                  
               }

             if(buff_M1_A_index > buff_M1_A_index_old)  // roll over
               {
                  sum_M1_A = 0;
                  buffStart = buffer_capture_cnt - buff_M1_A_index_old;
                  buffEnd =   buffer_capture_cnt;
                  uint16_t    rollover_count = buffEnd - buffStart;
               //   Serial.print("M1_PHA ");
                  for(int x = buffStart; x < buffEnd; x++)
                        {     time_old_M1_A=time_new_M1_A;
                              time_new_M1_A=dma_buffer_M1_A[x];
                              sum_M1_A += time_new_M1_A- time_old_M1_A;                             
                        }
                     

                  buffStart = 0;
                  buffEnd =   buffer_capture_cnt-buff_M1_A_index;
                  rollover_count += (buffEnd - buffStart);
                  for(int x = buffStart; x < buffEnd; x++)
                        {     time_old_M1_A=time_new_M1_A;
                              time_new_M1_A=dma_buffer_M1_A[x];
                               sum_M1_A += time_new_M1_A- time_old_M1_A;   
                             
                        }
                         dma_vel_M1_A = mm_per_slot/((sum_M1_A/rollover_count)*tick_us);
                    //     Serial.print(dma_vel_M1_A );
                              
                  
                }

    ///////////////////////////////// buffer M1B  ///////////////////////////////////////////      
            if(buff_M1_B_index < buff_M1_B_index_old)  // counts down from max to 1
               {
                  sum_M1_B = 0;
                  buffStart = buffer_capture_cnt - buff_M1_B_index_old;
                  buffEnd =   buffer_capture_cnt - buff_M1_B_index;
              //    Serial.print("M1_PHA ");
                  for(int x = buffStart; x < buffEnd; x++)
                        {     time_old_M1_B=time_new_M1_B;
                              time_new_M1_B=dma_buffer_M1_B[x];
                              sum_M1_B += time_new_M1_B- time_old_M1_B;  
                             
                        }
                         dma_vel_M1_B = mm_per_slot/(sum_M1_B/(buffEnd-buffStart)*tick_us);
                    //     Serial.print(" , ");
                    //     Serial.print(dma_vel_M1_B );
                 
               }

            if(buff_M1_B_index > buff_M1_B_index_old)  // roll over
               {
                  sum_M1_B = 0;
                  buffStart = buffer_capture_cnt - buff_M1_B_index_old;
                  buffEnd =   buffer_capture_cnt;
                  uint16_t    rollover_count = buffEnd - buffStart;
               //   Serial.print("M1_PHA ");
                  for(int x = buffStart; x < buffEnd; x++)
                        {     time_old_M1_B=time_new_M1_B;
                              time_new_M1_B=dma_buffer_M1_B[x];
                              sum_M1_B += time_new_M1_B- time_old_M1_B;                             
                        }
                     

                  buffStart = 0;
                  buffEnd =   buffer_capture_cnt-buff_M1_B_index;
                  rollover_count += (buffEnd - buffStart);
                  for(int x = buffStart; x < buffEnd; x++)
                        {     time_old_M1_B=time_new_M1_B;
                              time_new_M1_B=dma_buffer_M1_B[x];
                               sum_M1_B += time_new_M1_B- time_old_M1_B;   
                             
                        }
                         dma_vel_M1_B = mm_per_slot/((sum_M1_B/rollover_count)*tick_us);
                    //     Serial.print(" , ");
                    //     Serial.print(dma_vel_M1_B );
                              
                  
                }
  
             
                    // dma_vel_M1_A = mm_per_slot/((sum_M1_A/buffSize_M1_A)*tick_us);
                        
                    
                   
                      int32_t enc1_u_read =  ENC1_UPOS;
                     enc1_u_read = (enc1_u_read<<16) | ENC1_LPOSH;
                     enc_count_old=enc_count_new;
                     enc_count_new = enc1_u_read;
                     if(enc_count_old != enc_count_new)
                     {
                        enc_vel = (mm_per_slot*(enc_count_new-enc_count_old)/2)/0.002;
                    
                     
                    
                    
                     float distance = mm_per_slot *( enc1_u_read);

                     Serial.print((dma_vel_M1_A + dma_vel_M1_B) / 2 );

                       Serial.printf(",%0.2f",enc_vel);
                 //    Serial.printf("%0.1f\t",dma_vel_M1_A);
               //      Serial.printf("%0.1f\t",dma_vel_M1_B);
              //       Serial.printf("%0.1f\t",dma_vel_M2_A);
              //       Serial.printf("%0.1f\t",dma_vel_M2_B);
                       Serial.println("");
                     }
                    
                    
                   
                    
                     
      }


  
}



     
     
