//---------------------------------------------------------------------------------------------------------------//
//                                                                                                               //
//              This program written for SOURCE and HSID board of IONICAXX to                                    //
//              control the HV and Temperature of HSID and Source                                                //
//              MehrdadT@ionics.ca         Date : Aug 12th, 2008       				    						 //	
//              Version : 2.0 based on dsPIC33  							    								 //	
//              comment: this version has a PID control parameter to control temperature with seperate           //
//                       thermocouple                                                                            //
//              latest update for Source May12 was regarding immidiate heater shut down after zero set point     // 
//              in this version I added an option to current control of Crona HV                                 //
//              latest update for HV+ and HV- seperate setting                                                   // 
//              last  update for crona current control july 9th                                                  // 
//              last update for corona control hv+ and hv- july 15th                                             //
//              last updat for corona control hangout problem Nov11th                                            // 
//              Modified probeID logic for HV reset April 6th,2010                                               //                  
//              Modified SPI the same as rest Sep23,2010 also vesrion readable from ionsniffer                   // 
//              Modified for achieve fast switching @ April 28th 2011                                            //                   
//              Modified for acheive 2 APCI probe at the same time @ Oct 2012                                    //  
//              Modified to work with new hardware (current control) @June 2013                                  //
//---------------------------------------------------------------------------------------------------------------//
#define __dsPIC33F__

#include "p33Fxxxx.h"

#include <spi.h>
#include <outcompare.h>
#include <timer.h>

_FGS(GSS_OFF & GWRP_OFF); //code protect off,write protect disabled
_FOSCSEL(FNOSC_PRIPLL & IESO_OFF);
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF & POSCMD_HS);
_FWDT(FWDTEN_OFF);//watch dog disabled

// configuration settings//
int ix;
int SPIFlag =0 ;
int rxdData1;
int txdData1;
int rxdData2;
int txdData2;
int CommandCounter1 = 0;


int main(void)
{
unsigned int channel2;	// conversion result as read from result buffer
unsigned int channel3;	// conversion result as read from result buffer
unsigned int offset;	// buffer offset to point to the base of the idle buffer


const char*pSrc;
char*pDst;
int txferSize;
int fail=0; // success flag
int i;
int kPos=0;
int kNeg=0;
long int sum =0;

int CommandCounter2 = 0;
int Instruction1; // 1,2,3
int Instruction2; // 1,2,3
int isDataAvlbl1;
int isDataAvlbl2;
int CurrentAverage = 0;
int CurrentAverageOld = 0;
int AverageCounter = 0;
int TimeConstantCounter = 0;
int TempDifference=0; 
int ErrorFactor=0; 
int ErrorCurrentPos =0;
int ErrorCurrentNeg =0 ;
int MyCounter = 0;
int MyDelay =0;

unsigned char PortValue=0x0;
unsigned short Result;
unsigned short ResultHV;
unsigned short ResultCurrent;
unsigned short Temperature;
unsigned short TempSetpoint=0;
unsigned short HVSetpoint1=0;
unsigned short HVSetpoint2=0;
unsigned short TimeConstant = 21;
unsigned short CronaCurrent = 0;
unsigned short CronaSettingPos = 0;
unsigned short CronaSettingNeg = 0;

unsigned char TemperatureLow=0;
unsigned char TemperatureHi=0;
unsigned char TempSetpointLow=0;
unsigned char TempSetpointHi=0;
unsigned char VoltageMonitorLow1=0;
unsigned char VoltageMonitorHi1=0;
unsigned char HVSetpointLow1=0;
unsigned char HVSetpointHi1=0;
unsigned char APCIRealTime =0;
unsigned char APCIRealTime2 =0;

unsigned char ProbeId=0x0;

unsigned char version = 16;   //new version for corona//


// init the SPI 1 and SPI2

 SPI2CON1bits.SSEN = 1; // slave select pin enabled//

                	    // enable slave, mode8, cke=1, ckp = 0, smp = 0
    SPI2CON1 = 0x8080;	// enables the spi
    SPI2STAT = 0x8000;
 
TRISG = 0x00;
//init PWM for HV1 and HV2 , fc 90KHZ //
CloseOC1();
ConfigIntOC1(OC_INT_OFF );
OpenOC1(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 298, 300);
CloseOC2();
ConfigIntOC2(OC_INT_OFF );
OpenOC2(OC_IDLE_CON & OC_TIMER2_SRC & OC_CONTINUE_PULSE, 298, 300);


//init timer2, 0x175 for 90khz//
    ConfigIntTimer2(T2_INT_OFF);
    WriteTimer2(0);
    OpenTimer2(T2_ON & T2_GATE_OFF & T2_PS_1_1 ,  300);
    T2CON = 0x8000;
SetPulseOC1(0x0 , 2 );//2
SetPulseOC2(0x0 , 2 );//2

//init PWM for Temp1 and Temp2 , fc 60 HZ //
CloseOC3();
ConfigIntOC3(OC_INT_OFF );
OpenOC3(OC_IDLE_CON & OC_TIMER3_SRC & OC_CONTINUE_PULSE, 0xffff, 0xfffd);
SetPulseOC3(0x0, 0xfffd);

//init timer3, 0xffff for 60Hz//
    ConfigIntTimer3(T3_INT_OFF);
    WriteTimer3(0);
    OpenTimer3(T3_ON & T3_GATE_OFF & T3_PS_1_1 ,  0xffff);
    T3CON = 0x8010;


//init ADC channels for AN0 and AN2 //
	AD1CON1 = 0x00e0;
	AD1CON2 = 0x0000;
 	AD1CON3 = 0x1f02;
    AD1PCFGL = 0xfffa; //AN0 and AN2 ADC enabled 
    AD1PCFGH = 0xffff; //needed for probe ID reading //
    AD1CSSH = 0x0000;     
	AD1CSSL = 0x0000;  //ano and an2 scan 			
	AD1CON1bits.ADON =1; // ADC1 on
/* Configure SPI2 interrupt */

    ConfigIntSPI2(SPI_INT_EN &  SPI_INT_PRI_6);
// start of main loop !!!! //

while(1)
  {

if ( PORTEbits.RE1 == 0 )   //reading POL2 corona HV//
  {
   PORTGbits.RG15 = 0;
  }
else
 {
   PORTGbits.RG15 = 1;
 }

//Now its the time to read APCI and ESI inputs//
 
 if (( PORTEbits.RE6 == 1 ) || ( PORTEbits.RE7 == 1 ))   //reading APCI-1 and APCI-2//
  {
   APCIRealTime = 1 ;
  }   
  else
  {
  APCIRealTime = 0 ;
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
                     txdData1 = version;//Instruction1 + 0x80 ;
		             if (Instruction1 == 1)
                              {
						      TempSetpointLow = rxdData1;
                              }
                           
					 if  ((Instruction1 == 2) ^ (Instruction1 == 12))
                         	 {
                             HVSetpointLow1 = rxdData1;
                             }
					 } 
	           
			   else if  (CommandCounter1==1)//is it second byte?//
					{
                       Instruction1 = rxdData1;
				    	 if ((Instruction1 == 1) ^ (Instruction1 == 11))
                            {
                             txdData1=0;
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
                              txdData1=0;
                              
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

if (  APCIRealTime  == 0 ) 
{
HVSetpointHi1 = 0;    // reset HV //
HVSetpointLow1 = 0;   // reset HV // 
}		
else {}

//Now it's the time to control the Crona HV current based on the setting //	

CronaSettingPos = HVSetpointHi1;  //HV + setting
CronaSettingNeg = HVSetpointLow1;   //HV - setting


		//Now its time to read ADCs //

AD1CHS0 = 0x0000;//select AN0 //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
ResultHV = ADC1BUF0;
ResultHV >>= 2; //adjust adc from 10 bit to 8 bit value//
VoltageMonitorHi1 = ResultHV ;
	

AD1CHS0 = 0x0002;  //select AN2 //
AD1CON1bits.SAMP = 1; // start sampling  ADC1
while (!AD1CON1bits.DONE);
AD1CON1bits.DONE = 0;
CronaCurrent = ADC1BUF0;
ResultCurrent = ADC1BUF0;
   ResultCurrent >>= 2;
VoltageMonitorLow1 = ResultCurrent ;

     //Now its time to calculate average of CronaCurrent //

sum =  sum + CronaCurrent;
MyCounter = MyCounter + 1;	


     if (MyCounter == 500)                      
         { 
         //clear all the buffers//	

         MyCounter = 0; 
         CurrentAverage = (sum / 500) ;    // this is real average of Crona//	
		 CurrentAverageOld = CurrentAverage;
         sum =0;
         CurrentAverage=0; 

        // Time to control the current //  
          MyDelay++;
     if (MyDelay == 1)    //20
       {
  
         MyDelay=0; //reset for next loop// 
        // first, positive voltage : CronaSettingPos //

if  (CronaSettingPos == 0) 
  {
    SetPulseOC2(0x0 , 2 );                       //power shut down to protection // 
  } 
else if ( PORTEbits.RE1 == 0 )   // if its normal position for crona probe and a valid setting for current 0.1ua to 4.0ua equal to (0-200)setting equql to (0 - 570)ADC reading //
       { 
        SetPulseOC2(0x0 , 2 + CronaSettingPos ); 
       }   
              
       

        // second, negative voltage : CronaSettingNeg //

if  (CronaSettingNeg == 0) 
  {
    SetPulseOC1(0x0 , 2 );                       //power shut down to protection // 
  } 
else if ( PORTEbits.RE1 == 1   )  // if its normal position for crona probe and a valid setting for current 0.1ua to 4.0ua equal to (0-200)setting equql to (0 - 570)ADC reading //
       
        {
         SetPulseOC1(0x0 , 2 +CronaSettingNeg );
        }   
       
       
       }

        
            
        	
	    
     
     } //delay loop//
                    
    }//for while
   return 0;

}
//for main

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