//---------------------------------------------------------------------------------------------------------------//
//                                                                                                               //
//              This program written for SOURCE and HSID board of IONICAXX to                                    //
//              control the HV and Temperature of HSID and Source                                                //
//              MehrdadT@ionics.ca         Date : Aug 12th, 2008       				    						 //	
//              Version : 2.0 based on dsPIC33  							    								 //	
//              comment: this version has a PID control parameter to control temperature with seperate           //
//                       thermocouple                                                                            //
//              latest update for Source May12 was regarding immidiate heater shut down after zero set point     //
//              latest update for HV+ and HV- seperate setting                                                   // 
//              HSID heat controller fixed   July30th 2009                                                       //        
//              Calibration fixed Aug5th 2009                                                                    //
//              Fuzzy control start Aug 26th and finished successfull on Aug 28th                                //
//              Probe id detection added, too.  Sep 2nd 2009                                                     //  
//              Added new commands to poll temperature and HV without setting them on Dec3rd                     //
//              Change and mod for HV monitor and Temperature reading also temp control on Jan13,2010            //
//              Open loop temp control Jan13,2010                                                                //   
//              The SPI will be reset if the instruction in serial communication isn't right(1,2,3,11,12)        // 
//              SPI polling modified to Interrupt and it works perfect Feb21,2010                                // 
//              The HV pwm Carrier FQ modified to get max HV Mar24                                               //                                                  
//              PID tune up for Source heaters May 20th,2010                                                     // 
//              Changed termo limit to 700 and pwm to 60000                                                      //  
//              Changed PWM's limit to no limit to get 500C on probe1 and probe2 July 15,2011                    //
//              Adding Soft Start to resolving CE and TUV certification issue (ramp up current slow!) Sep6th,2011//                                                                                      
//              Add limitation for HV PWM in order to protect HV power suplly                                    //
//              Change power limitation from 45000 to 60000 on heat control March 2013                           //
//              change for APPI mode
//              Added Probe ID and ESI voltage control for MicroSpray-Sept 2014 (VT) 
//---------------------------------------------------------------------------------------------------------------//
#define __dsPIC33F__

#define P_Gain 2 //13// 
#define D_Gain 400 //621//
#define I_Gain 15 //13// 


#include "p33Fxxxx.h"

#include <spi.h>
#include <outcompare.h>
#include <timer.h>

_FGS(GSS_OFF & GWRP_OFF); //code protect off,write protect disabled
_FOSCSEL(FNOSC_PRIPLL & IESO_OFF);
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF & POSCMD_HS);
_FWDT(FWDTEN_OFF);//watch dog disabled

// configuration settings//
int SPIFlag =0 ;
int rxdData1;
unsigned short CommandCounter1 = 0;
int txdData1;
int TempAverageOld = 0;
int AverageCounter = 0;
unsigned int ControlledBand = 15; 
unsigned int IntegralCounter = 0;
unsigned int IntegralCounterMax = 34000;// 27021;//60021
unsigned int IntegralCounterDec = 320000;//1921; 
unsigned int Integral=0 ;
int BoostCounter =0;
int IntegralBoost = 1;
int LoopCounter = 0;
unsigned int RampTime =0;
unsigned short MyResult = 0;
// Functions //
unsigned short Calculate ( unsigned short TempSetpoint, unsigned short Temperature);


//---------------------------- Main loop-------------------------------------------//
int main(void)
{

int Instruction1; // 1,2,3
int DisplayCounter =0;
long int TempDisplay =1;
long int TempAverage = 1;
long int TempAverageCopy = 1;



unsigned short Result;
unsigned short ResultHV; 

unsigned short ResultTemperature;
unsigned short ResultTemperatureCopy;
unsigned short Temperature;
unsigned short TempSetpoint = 0;
unsigned short HVSetpoint1 = 0;
unsigned short HVSetpoint2 = 0;
unsigned short TimeConstant = 50;
unsigned short HeatPower =0;

unsigned char TemperatureLow=0;
unsigned char TemperatureHi=0;
unsigned char TempSetpointLow=0;
unsigned char TempSetpointHi=0;
unsigned char TemperatureLowDisplay=0;
unsigned char TemperatureHiDisplay=0;
unsigned char VoltageMonitorLow1=0;
unsigned char VoltageMonitorHi1=0;
unsigned char HVSetpointLow1=0;
unsigned char HVSetpointHi1=0;
unsigned char APCIRealTime=0;

unsigned char ESIRealTime=0;

unsigned char ProbeId=0x0;



// init the SPI 1 and SPI2

                	    // enable slave, mode8, cke=1, ckp = 0, smp = 0
    SPI2CON1 = 0x8080;	//0x8080 enables the spi
    SPI2STAT = 0x8000;
 
TRISG = 0x00;
//init PWM for HV1 and HV2 , fc 90KHZ //
CloseOC1();
ConfigIntOC1(OC_INT_OFF );
OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 316, 318);
CloseOC2();
ConfigIntOC2(OC_INT_OFF );
OpenOC2(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 316, 318);


//init timer2, 0x175 for 90khz//
    ConfigIntTimer2(T2_INT_OFF);
    WriteTimer2(0);
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_1 ,  318);
    T2CON = 0x8000;

//init PWM for Temp1 and Temp2 , fc 60 HZ //
CloseOC3();
ConfigIntOC3(OC_INT_OFF );
OpenOC3(OC_IDLE_CON & OC_TIMER3_SRC & OC_CONTINUE_PULSE, 0xfffd, 0xffff);
SetPulseOC3(0x0, 0xfffd);
 
//init timer3, 0xffff for 60Hz//
    ConfigIntTimer3(T3_INT_OFF);
    WriteTimer3(0);
    OpenTimer3(T3_ON & T3_GATE_OFF & T3_PS_1_1 ,  0xfffe);
    T3CON = 0x8010;

//init ADC channels for AN0 and AN2 //
	AD1CON1 = 0x00e0;
	AD1CON2 = 0x0000;
 	AD1CON3 = 0x1f02;
    AD1PCFGL = 0xfffa; //AN0 and AN2 ADC enabled 
    AD1PCFGH = 0xbfff; //needed for probe ID reading and an30 enabled//
    AD1CSSH = 0x0000;     
	AD1CSSL = 0x0000;  //ano and an2 scan 			
	AD1CON1bits.ADON =1; // ADC1 on

/* Configure SPI2 interrupt */

    ConfigIntSPI2(SPI_INT_EN &  SPI_INT_PRI_6);
// start of main loop !!!! //

while(1)

 {

if (( MyResult < 50) & (PORTEbits.RE7 == 0))
{
ESIRealTime = 0 ;
APCIRealTime = 0;
 ProbeId = 0 ; //no probe//
}   
if (( MyResult > 200) & (PORTEbits.RE7 == 1))
{
ESIRealTime = 1 ;
APCIRealTime = 0;
 ProbeId = 3 ; //APPI probe//
}   
if (( MyResult > 200) & (PORTEbits.RE7 == 0))
{
ESIRealTime = 1 ;
APCIRealTime = 0;
 ProbeId = 1 ; //ESI probe//
}   
if (( MyResult < 50) & ((PORTEbits.RE7) == 1))
{
ESIRealTime = 0 ;
APCIRealTime = 1;
 ProbeId = 2 ; //APCI probe//
}
   
if (( MyResult > 100)&( MyResult < 200))
{
ESIRealTime = 1 ;
APCIRealTime = 0;
 ProbeId = 4 ; //Micro spray//
}   






		 //Check the APCI and ESI logic for protection //

if  ( APCIRealTime == 1) //(ESIRealTime == 0)// 
{
HVSetpointHi1 = 0;    // reset HV //
HVSetpointLow1 = 0;   // reset HV // 
}		
if  (ESIRealTime == 0) 
{
HVSetpointHi1 = 0;    // reset HV //
HVSetpointLow1 = 0;   // reset HV // 
}		
         //time to check spi flag//
 if ( SPIFlag == 1  )
  {              
	          SPIFlag =0;
              PORTGbits.RG15 = 0 ;
   				//What to do wirh the first byte ?//

				if (CommandCounter1==4)//is it first byte?//
					{
					 CommandCounter1 = 0;
                     txdData1 = 18 ;// Instruction1 + 0x80 ;latest  version number
		             if (Instruction1 == 1)
                              {
						      TempSetpointLow = rxdData1;
                              }
                           
					 if  (Instruction1 == 2)
                         	 {
                             HVSetpointLow1 = rxdData1;
                             }
					 } 
	           
			   else if  (CommandCounter1==1)//is it second byte?//
					{
                       Instruction1 = rxdData1;
				    	 if ((Instruction1 == 1) ^ (Instruction1 == 11))
                            {
                             txdData1=TemperatureLowDisplay;
                             }
					      if  ((Instruction1 == 2) ^ (Instruction1 == 12))
                            {
                          txdData1=VoltageMonitorLow1;
                            }
  						  if  (Instruction1 == 3)
                         	 {                             
                             txdData1=ProbeId;
                             }			
                     }
                else if   (CommandCounter1==2)//is it third byte?//
					      {
					   
                           if ((Instruction1 == 1)^(Instruction1 == 11))
                              {  	
                              txdData1=TemperatureHiDisplay;
                              
							  }	
					       
                           if ((Instruction1 == 2)^(Instruction1 == 12))
                             {
                             txdData1=0 ;//VoltageMonitorHi1; 
                           
                             }	
					      if  (Instruction1 == 3)
                         	 {                   
                             txdData1=0;                            
                             }	
                         }
                else if   (CommandCounter1==3)//is it fourth byte?//
					      {  
                             txdData1=0;//0x55;			                 
                          
                          if (Instruction1 == 1)
                              { 
						      TempSetpointHi = rxdData1;
                              }
				          if  (Instruction1 == 2)
                         	 {
                             HVSetpointHi1 = rxdData1;
                             }
                            }
     }


		//Now its Time to set HV1 and HV2//

HVSetpoint1 = HVSetpointHi1;  //HV + setting
HVSetpoint2 = HVSetpointLow1; //HV - setting
        // loading the pwm buffer for HV1 and HV2// 

SetPulseOC1(0x0 , HVSetpoint1 +2 );
SetPulseOC2(0x0 , HVSetpoint2 +2 );
		//Now its Time to set TempSetpoint1 and TempSetpoint2//

TempSetpoint = TempSetpointHi;
TempSetpoint = TempSetpointLow + (TempSetpoint * 0xff) ;
                       

if (TempSetpoint > 700) TempSetpoint =700;    //limit for TempSetpoint     
		//Now its time to read ADCs //

AD1CHS0 = 0x0000;//select AN0 //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
ResultHV = ADC1BUF0;
ResultHV >>= 2; //adjust adc from 10 bit to 8 bit value//
VoltageMonitorLow1 = ResultHV ;

AD1CHS0 = 0x001e;//select AN30 //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
MyResult = ADC1BUF0;
MyResult >>= 2; //adjust adc from 10 bit to 8 bit value//


   
   if (LoopCounter++ == 21)    //21
    {
      LoopCounter =0;  	

   //  control the Heater based on PID algorithm //
 
         Temperature = TempAverageCopy;
       //  HeatPower = Calculate(TempSetpoint , Temperature);

//-------------------------------------------------------------------------------------//
//                                Calculate                                            //
//-------------------------------------------------------------------------------------//

unsigned int HeatPower;

  if (TempSetpoint >= Temperature )
  {
   if ( Temperature < (TempSetpoint - ControlledBand) )
    {
     //HeatPower =65532; 
      if (HeatPower < 60000)    //45000 on rev 6
         { 
         RampTime++;  
         if (RampTime == 7)
             {      
             HeatPower = HeatPower +1;
             RampTime =0;    
             } 
         }
      if (HeatPower == 60000)
         {
          HeatPower = 65533; 
         } 
     if (BoostCounter++ > 13000)   
       { 
         if ( IntegralBoost < 6 ) IntegralBoost++;
         BoostCounter = 0; 
         Integral  = IntegralBoost * TempSetpoint ;    
       } 
    } 
   else  
    {
     
       if ( IntegralCounter++ > (IntegralCounterMax /  ((TempSetpoint - Temperature)+1) ))
         {
         IntegralCounter = 0;
         if (Temperature <=  TempSetpoint  )
            {
             Integral++;
            } 
         if ( Integral > 4400 )  Integral = 4400 ;
         }           
          HeatPower = ( (((TempSetpoint+1)/(Temperature+1)) * P_Gain)  + (Integral * I_Gain )  + ((TempSetpoint - Temperature)* D_Gain) ); 
                    
    }
  }
  else if   (TempSetpoint < Temperature )
  { 
   
   if (( Temperature   > TempSetpoint) & ((Temperature - 3)< TempSetpoint)) 
       {
        HeatPower =  ( (((TempSetpoint+1)/(Temperature+1)) * P_Gain)  + ((Integral/1.21) * I_Gain )  - ((  Temperature - TempSetpoint)* D_Gain) ); 
   
         if (IntegralCounter++ > IntegralCounterDec )
           {
             IntegralCounter = 0;
             if (Integral > 0) Integral--;
            }   
       }
   else if ( (Temperature - 3 )  > TempSetpoint)
      {
         HeatPower = TempSetpoint * 25; //cut off the heat if its 10 degree more than setpoint 
      }          
    
   }  

if (HeatPower > 65532) HeatPower = 65532; 
//if (Temperature > 600) HeatPower = 0;
SetPulseOC3(0x0, (65533 -  HeatPower) );   

//---------------------------------------------------------------------------------------------------------------------
	
AD1CHS0 = 0x0002;  //select AN2 //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
ResultTemperature = ADC1BUF0;
Temperature = (ResultTemperature * 43 )/50 ; //consider termocouple posiotion in source//

  //Now its time to calculate average of Temperature for display //
	TempAverage = Temperature + TempAverage;
	AverageCounter++;

   if (AverageCounter == 3000) //if we have 2 samples, control heatthers //
     {
         	TempAverage = TempAverage /  3000;    // this is real average of temp and calibration //
 
         //clear all the buffers//	
         AverageCounter = 0;
		 TempAverageCopy = TempAverage;         
         TempAverage=0; 	

         //now digital filtering for display for noise reduction//
         TempDisplay = TempAverageCopy + TempDisplay;
         DisplayCounter++;
     if (DisplayCounter ==30)
        {
          DisplayCounter =0;
          TempDisplay = TempDisplay/30;
 
			ResultTemperatureCopy = TempDisplay;
			TemperatureLowDisplay = ResultTemperatureCopy;
			ResultTemperatureCopy >>= 8;
			TemperatureHiDisplay = ResultTemperatureCopy; 	
            TempDisplay =0;     
		}//display filter end//
       }

	 }   //end of main loop for thermo controller//
                    
    }//for while
   return 0;

}//for main

//*************************************************************************************//
//Time to check SPI2 for any new data//
void __attribute__((__interrupt__)) _SPI2Interrupt(void) 

{    
    
    IFS2bits.SPI2IF = 0;
    SPI2STATbits.SPIROV = 0;  // Clear SPI1 receive overflow flag if set //

PORTGbits.RG15 = 1 ;
rxdData1 = ReadSPI2();


 SPIFlag =1;
CommandCounter1 = CommandCounter1 +1 ;
if (!SPI2STATbits.SPITBF)
								{
								WriteSPI2(txdData1);//if txd buffer is rdy send data//
							    }

}


