//---------------------------------------------------------------------------------------------------------------//
//                                                                                                               //
//              This program written for SOURCE and HSID board of IONICAXX to                                    //
//              control the HV and Temperature of HSID and Source                                                //
//              MehrdadT@ionics.ca         Date : Aug 12th, 2008       				    						 //	
//              Version : 2.0 based on dsPIC33  							    								 //	
//              comment: this version has a PID control parameter to control temperature with seperate           //                                                                                                 //
//              latest update for Source May12 was regarding immidiate heater shut down after zero set point     //
//              latest update for HV+ and HV- seperate setting                                                   // 
//              HSID heat controller fixed   July30th 2009                                                       // 
//              Calibration fixed Aug5th 2009                                                                    //
//              Fuzzy control start Aug 26th and finished successfull on Aug 28th                                //  
//              Change and mod for HV monitor and Temperature reading also temp control on Jan13,2010            //
//              Open loop temp control Jan13,2010                                                                //   
//              The SPI will be reset if the instruction in serial communication isn't right(1,2,3,11,12)        //  
//              The SPI communication modified for fast respond to polling FEB15th,2010                          //   
//              SPI polling modified to Interrupt and works perfect Feb21,2010                                   //
//              CurtainCap logic changed (never disable curtaincap on APCI or ESI) Mar23                         //   
//              New PID temperature control May 20th 2010                                                        //                        
//              New PID optimizing Feb 17th, 2011                                                                // 
//---------------------------------------------------------------------------------------------------------------//
#define __dsPIC33F__

#define P_Gain 2 //13// 
#define D_Gain 621//621
#define I_Gain 13 //18//changed to 18 on Feb17th,2011 to adapt to the new HW changes 

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
int TempAverageCopy = 0;
int AverageCounter = 0;
unsigned int ControlledBand = 5;  //15 
unsigned int IntegralCounter = 0;
unsigned int IntegralCounterMax = 14400;//3400;//55021;//49021//
unsigned int IntegralCounterDec = 12200;//021; //1021
unsigned int Integral=0 ;
int BoostCounter =0;
int IntegralBoost = 1;
int LoopCounter = 0;
// Functions //
unsigned int Calculate ( unsigned short TempSetpoint, unsigned short Temperature);


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
unsigned int HeatPower =0;

unsigned char TemperatureLow=0;
unsigned char TemperatureHi=0;
unsigned char TempSetpointLow=0;
unsigned char TempSetpointHi=0;
unsigned char VoltageMonitorLow1=0;
unsigned char VoltageMonitorHi1=0;
unsigned char TemperatureLowDisplay=0;
unsigned char TemperatureHiDisplay=0;
unsigned char HVSetpointLow1=0;
unsigned char HVSetpointHi1=0;
unsigned char APCIRealTime=0;

unsigned char ESIRealTime=0;

unsigned char ProbeId=0x0;



// init the SPI 1 and SPI2

                	    // enable slave, mode8, cke=1, ckp = 0, smp = 0
    SPI2CON1 = 0x8080;	// enables the spi
    SPI2STAT = 0x8000;
 
TRISG = 0x00;
//init PWM for HV1 and HV2 , fc 90KHZ //
CloseOC1();
ConfigIntOC1(OC_INT_OFF );
OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 0x157, 0x159);
CloseOC2();
ConfigIntOC2(OC_INT_OFF );
OpenOC2(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 0x157, 0x159);


//init timer2, 0x175 for 90khz//
    ConfigIntTimer2(T2_INT_OFF);
    WriteTimer2(0);
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_1 ,  0x159);
    T2CON = 0x8000;

//init PWM for Temp1 and Temp2 , fc 60 HZ //
CloseOC3();
ConfigIntOC3(OC_INT_OFF );
OpenOC3(OC_IDLE_CON & OC_TIMER3_SRC & OC_CONTINUE_PULSE, 0xfffd, 0xffff); //0xffff,0xfffd for 65hz
SetPulseOC3(0x0, 0xfffd);

//init timer3, 0xffff for 60Hz//
    ConfigIntTimer3(T3_INT_OFF);
    WriteTimer3(0);
    OpenTimer3(T3_ON & T3_GATE_OFF & T3_PS_1_1 ,  0xfffe);
    T3CON = 0x8000;

//init ADC channels for AN0 and AN2 //
	AD1CON1 = 0x00e0;
	AD1CON2 = 0x0000;
 	AD1CON3 = 0x1f02;
    AD1PCFGL = 0xfff0; //AN0 and AN2 and AN3 ADCs enabled 
    AD1PCFGH = 0xffff; //needed for probe ID reading //
    AD1CSSH = 0x0000;     
	AD1CSSL = 0x0000;  //ano and an2 scan 			
	AD1CON1bits.ADON =1; // ADC1 on
/* Configure SPI2 interrupt */

    ConfigIntSPI2(SPI_INT_EN &  SPI_INT_PRI_6);

// start of main loop !!!! //

while(1)
  {

PORTGbits.RG15 = 0 ;
PORTGbits.RG15 = 1 ;    //just for loop speed test //

 //Now its the time to read APCI and ESI inputs//
 
 if (( PORTEbits.RE4 == 0 ) & ( PORTEbits.RE3 == 0 ))   //reading ESI-1 and ESI-2//
  {
   ESIRealTime = 0 ;
  } 
 else
  {
  ESIRealTime = 1 ; 
  }

 if (( PORTEbits.RE6 == 0 ) & ( PORTEbits.RE7 == 0 ))   //reading APCI-1 and APCI-2//
  {
   APCIRealTime = 0 ;
  }   
  else
  {
  APCIRealTime = 1 ;
  }
 //Time to check SPI2 for any new data//
           //time to check spi flag//
 if ( SPIFlag == 1  )
  {              
	          SPIFlag =0;
              PORTGbits.RG15 = 0 ;
   				//What to do wirh the first byte ?//

				if (CommandCounter1==4)//is it first byte?//
					{
					 CommandCounter1 = 0;
                     txdData1 =9;         // Version;
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
                             txdData1=TemperatureLowDisplay;                             }
					      if  ((Instruction1 == 2) ^ (Instruction1 == 12))
                            {
                          txdData1=VoltageMonitorLow1;
                            }
  						  if  (Instruction1 == 3)
                         	 {                             
                             txdData1=0;
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


  		 //Check the APCI and ESI logic for protection //

if ( ( APCIRealTime == 0) &  (ESIRealTime == 0)) 
{
HVSetpointHi1 = 0;    // reset HV //
HVSetpointLow1 = 0;   // reset HV // 
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
if (TempSetpoint > 360) TempSetpoint = 360;    //limit for TempSetpoint           
		//Now its time to read ADCs //

AD1CHS0 = 0x0000;//select AN0 //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
ResultHV = ADC1BUF0;
ResultHV >>= 2; //adjust adc from 10 bit to 8 bit value//
VoltageMonitorLow1 = ResultHV ;
	

         
     if (LoopCounter++ == 15)
    { 
      LoopCounter =0; //reset it for next time //

    	 //  control the Heater based on PID algorithm //
  
         Temperature = TempAverageCopy;  
         HeatPower = Calculate(TempSetpoint , Temperature);       

AD1CHS0 = 0x0003;  //select AN3(REVA)  AN2(old) //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
ResultTemperature = ADC1BUF0;
Temperature = (ResultTemperature * 2 )/3 ;


     //Now its time to calculate average of Temperature for Display //
     TempAverage = Temperature + TempAverage;
     AverageCounter++;	

   if (AverageCounter == 5000) //if we have 2 samples, control heatthers //
     {
    
         TempAverage = TempAverage /  5000      ;    // this is real average of temp and calibration //
    
        
		 //clear all the buffers//	
         AverageCounter = 0;
		 TempAverageCopy = TempAverage;         
         TempAverage=0;

 //now digital filtering for display for noise reduction//

     TempDisplay = TempAverageCopy + TempDisplay; 
     DisplayCounter++;

    if ( DisplayCounter == 30)
     {
         DisplayCounter =0; 
         TempDisplay = TempDisplay /30;      

         ResultTemperatureCopy = TempDisplay;
         TemperatureLowDisplay = ResultTemperatureCopy;
         ResultTemperatureCopy >>= 8;
         TemperatureHiDisplay = ResultTemperatureCopy;
         TempDisplay =0 ;
      }//display filter end// 
	 }   //end of main loop for thermo controller//
     }               
    }//for while
   return 0;




}//for main
//-------------------------------------------------------------------------------------//
//                                Calculate                                            //
//-------------------------------------------------------------------------------------//
unsigned int Calculate ( unsigned short TempSetpoint, unsigned short Temperature)
{
unsigned int HeatPower;


  if (TempSetpoint >= Temperature )
  {
   if ( Temperature < (TempSetpoint - ControlledBand) )
    {
     HeatPower = 65500;  //64500
     if (BoostCounter++ > 1000 ) //25000
       { 
         if ( IntegralBoost < 13 ) IntegralBoost++; //13 @ 65hz
         BoostCounter = 0; 
         Integral  = IntegralBoost * TempSetpoint ;    
       } 
    } 
   else  
    {
     
       if ( IntegralCounter++ > (IntegralCounterMax /  ((TempSetpoint - Temperature)+1) ))
         {
         IntegralCounter = 0;
         if (Temperature <= TempSetpoint )
            {
             Integral++;
            } 
         if ( Integral > 4600 )  Integral = 4600 ;   //4400 @65hz
         }           
          HeatPower = ( (((TempSetpoint+1)/(Temperature+1)) * P_Gain)  + (Integral * I_Gain )  + ((TempSetpoint - Temperature)* D_Gain) ); 
           
          
    }
  }
  else if   (TempSetpoint < Temperature )
  { 
    if ((Temperature - TempSetpoint) > 20 )
         {
         Integral = 0; // Reset integral //
         IntegralBoost =0 ;    
         } 
  else if (( Temperature    > TempSetpoint) & ((Temperature - 6)< TempSetpoint)) 
       {

        if (IntegralCounter++ > IntegralCounterDec )
           {
             IntegralCounter = 0;
             if (Integral > 0) Integral--;
            }          //its more than enough slow it down!

        HeatPower = ( (((TempSetpoint+1)/(Temperature+1)) * P_Gain)  + ((Integral/1.1) * I_Gain )   ); 

       if (HeatPower > 0xfffd) HeatPower = 0xfffd;
         if (IntegralCounter++ > IntegralCounterDec )
           {
             IntegralCounter = 0;
             if (Integral > 0) Integral--;
            }   
       }
   else if ( (Temperature - 6 )  > TempSetpoint)
      {
         HeatPower = TempSetpoint * 50; //cut off the heat if its 10 degree more than setpoint 
      }          
    
   }  

if (HeatPower > 0xfffd) HeatPower = 0xfffd; 
if (Temperature > 700) HeatPower = 0;
SetPulseOC3(0x0, (0xffff -  HeatPower) ); //65533   
return (1);
} 
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
//---------------------------------------------------------------------------------------//