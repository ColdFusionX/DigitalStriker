/*
 * File:   Minicode.c
 * Author: ColdFusionX
 *
 * Created on March 6, 2017, 1:50 PM
 */

#include <p18f4550.h>

#define LCD_EN LATCbits.LC7
#define LCD_RS LATCbits.LC6
#define LCDPORT PORTD


void lcd_delay(unsigned long int time)
{
 unsigned int i , j ;

    for(i = 0; i < time; i++)
    {
            for(j=0;j<100;j++);
    }
}

void SendInstruction(unsigned char command)
{
     LCD_RS = 0;				// RS low : Instruction
     lcd_delay(10);
     LCD_EN = 1;				// EN High
     lcd_delay(10);
     LCDPORT = command;         		// Command
     lcd_delay(10);
     LCD_EN = 0;				// EN Low
     lcd_delay(10);
 }

void SendData(unsigned char lcddata)
{
     LCD_RS = 1;				// RS HIGH : DATA
     lcd_delay(10);
     LCD_EN = 1;				// EN High
     lcd_delay(10);
     LCDPORT = lcddata; 			// DATA
     lcd_delay(10);
     LCD_EN = 0;				// EN Low
     lcd_delay(10);
}

void LCDDisplayStr(unsigned char *String)
{
  while(*String)
  {
   SendData(*String);
   String++;
  }
}
void SetLineNumber(unsigned char linenum)
{
    if(linenum == 1)
    {
      SendInstruction( 0x80 );	
    }
    else
    {
      SendInstruction( 0xC5 );		
    }
}

void InitLCD(void)
{

    SendInstruction(0x38);      		//8 bit mode, 2 line,5x7 dots
    lcd_delay(5);
    SendInstruction(0x06);		// entry mode
    lcd_delay(5);    
    SendInstruction(0x0C);		//Display ON cursor OFF
    lcd_delay(5);
    SendInstruction(0x01);      		//Clear display
    lcd_delay(5);
    
}
void lcd_clear(void)
{
SendInstruction(0x01);
}

void ADCInit(void)
{
    TRISEbits.RE2 = 1;                  //ADC channel 7 input

    ADCON1 = 0b00000111; //Ref voltage Vdd & Vss;AN0-AN7 channels
    ADCON2 = 0b10101110; //Right justified; Acquisition time 4T;Conversion clock Fosc/64
}

unsigned short Read_ADC(unsigned char Ch)
{
    ADCON0 = 0b00011101 ;  //ADC on; Select channel 7;
    GODONE = 1;                    //Start Conversion

    while(GO_DONE == 1 );  //Wait till A/D conv. is complete
    return ADRES;             //Return ADC result
}

unsigned long DisplayResult(unsigned short ADCVal)
{
 unsigned short tempv;
 
 int i;
 unsigned long resistance,conductance,force;
 

tempv = ADCVal;
 ADCVal = (5000/1024)*tempv; //Convert binary data to mV; 1bit <=> (5000/1024)mV
 resistance=5000-ADCVal;
 resistance *= 10000; // 10K resistor
 resistance /= ADCVal; // FSR resistance in ohms.
 conductance = 1000000; //We measure in microMhos.
conductance /= resistance; //Conductance in microMhos.
 if(conductance<=1000)
      force = conductance*100 / 80;
 else
   {
      force=conductance-1000;
      force/=30;
    }
 
 
return force;
}
/* for(i=0;i<5;i++)               
}




void main(void)
{
    /*TRISB = 0x00; //set data port as output
    TRISCbits.RC0 = 0; //EN pin
    TRISCbits.RC1 = 0; // RS pin

    InitLCD();
  while(1)                     			//Forever loop
{        SetLineNumber(1);
        LCDDisplayStr("force=");
        SendInstruction(0x86);
        LCDDisplayStr("");
        SendInstruction(0x89);
        LCDDisplayStr("newton");
                
        SetLineNumber(2);
        LCDDisplayStr("pressure=");
        SendInstruction(0xCA);
        LCDDisplayStr("");
        SendInstruction(0xCC);
        LCDDisplayStr("pa");
 
 
}*/
 /*   int i;
        unsigned short Ch_result,Ch_result1;unsigned long resistance,conductance,force1,force2,pressure,temp;

    TRISD = 0x00;                     //PORTD connected to LCD is output
    ADCInit();
    InitLCD();
    TRISCbits.RC6 = 0; //EN pin
    TRISCbits.RC7 = 0; // RS pin

    do
    {    
        Ch_result = Read_ADC(7);
        lcd_delay(20);
        Ch_result1=Read_ADC(7);
//for 1st result         
temp =Ch_result ;
temp = (5000/1024)*temp; //Convert binary data to mV; 1bit <=> (5000/1024)mV
resistance=5000-temp;
resistance *= 10000; // 10K resistor
	    resistance /= temp; // FSR resistance in ohms.
 
		conductance = 1000000; //We measure in microMhos.
	    conductance /= resistance; //Conductance in microMhos.
 force1 = conductance / 80;
//for 2nd result 
temp =Ch_result1 ;
temp = (5000/1024)*temp; //Convert binary data to mV; 1bit <=> (5000/1024)mV
resistance=5000-temp;
resistance *= 10000; // 10K resistor
	    resistance /= temp; // FSR resistance in ohms.
 
		conductance = 1000000; //We measure in microMhos.
	    conductance /= resistance; //Conductance in microMhos.
 force2 = conductance / 80;               



    }  while(force2<force1);
                    Ch_result = Read_ADC(7);
                    DisplayResult(Ch_result);
                    lcd_delay(20);
                    lcd_delay(20);
                    
 
 
    
}
 */
void main()
{
      short fsrReading,fsrVoltage,fsrResistance,fsrForce,fsrReading1,fsrVolt,fsrResistance1,fsrForce1;
    long fsrConductance,fsrConductance1;
    unsigned char text[16],i;
    
       // unsigned short Ch_result,Ch_result1;unsigned long resistance,conductance,force1,force2,pressure,temp;

    TRISD = 0x00;                     //PORTD connected to LCD is output
TRISB = 0x00;
TRISEbits.RE1 = 0;
    TRISCbits.RC6 = 0; //RS pin
    TRISCbits.RC7 = 0; // EN pin
    ADCInit();
    InitLCD();  
  while(1) { 

fsrReading = Read_ADC(7);  
fsrReading1 =fsrReading;
SetLineNumber(1);  
  LCDDisplayStr("Analog reading = ");
  
  
  SetLineNumber(2);
for( i=10 ; i>0 ; i--)            		//Display string on LCD
 {
   text[i]=(fsrReading1%10)+'0';
   fsrReading1/ = 10;
 }

  for(i=0;i<=10;i++)                       		//Display string on LCD
 {
   SendData(text[i]);
 }



lcd_clear();
lcd_delay(2000);
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)

  fsrVoltage =(5500/1024)*fsrReading;
  fsrVolt=fsrVoltage;
  SetLineNumber(1);  
LCDDisplayStr("Voltage in mV");
  SetLineNumber(2);
  
for( i=10 ; i>0 ; i--)            		//Display string on LCD
 {
   text[i]=(fsrVolt%10)+'0';
   fsrVolt/ = 10;
 }

  for(i=0;i<=10;i++)                       		//Display string on LCD
 {
   SendData(text[i]);
 }  
lcd_delay(2000);
 lcd_clear();
  if (fsrVoltage == 0) {
    SetLineNumber(1);  
LCDDisplayStr("No pressure");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    fsrResistance1 =fsrResistance;
    SetLineNumber(1);  
LCDDisplayStr("FSR-(ohms)=");
     SetLineNumber(2);
for( i=10; i>0 ; i--)            		//Display string on LCD
 {
   text[i]=(fsrResistance1%10)+'0';
   fsrResistance1/ = 10;
 }

  for(i=0;i<=10;i++)                       		//Display string on LCD
 {
   SendData(text[i]);
 }  
lcd_clear();
lcd_delay(2000);
 
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;
    fsrConductance1 =fsrConductance ;
    SetLineNumber(1);  
LCDDisplayStr("Conduc.(uMhos)");
     SetLineNumber(2);
for( i=10; i>0 ; i--)            		//Display string on LCD
 {
   text[i]=( fsrConductance1%10)+'0';
    fsrConductance1/ = 10;
 }

  for(i=0;i<=10;i++)                       		//Display string on LCD
 {
   SendData(text[i]);
 }  
lcd_clear();
lcd_delay(2000);
 
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      SetLineNumber(1);  
LCDDisplayStr("Force in N: ");
     SetLineNumber(2);
for( i=10 ; i>0 ; i--)            		//Display string on LCD
 {
   text[i]=( fsrForce%10)+'0';
    fsrForce/ = 10;
 }

  for(i=0;i<=10;i++)                       		//Display string on LCD
 {
   SendData(text[i]);
 }        
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      SetLineNumber(1);  
LCDDisplayStr("Force in N ");
      SetLineNumber(2);  
for( i=10 ; i>0 ; i--)            		//Display string on LCD
 {
   text[i]=( fsrForce%10)+'0';
    fsrForce/ = 10;
 }

  for(i=0;i<=10;i++)                       		//Display string on LCD
 {
   SendData(text[i]);
 }
      lcd_clear();
lcd_delay(2000);
    }
  }
  if (fsrReading < 10) {
      PORTB=0xC0;
  } else if (fsrReading < 200) {
    PORTB=0xE0;
  } else if (fsrReading < 500) {
    PORTB=0xF8;
  } else if (fsrReading < 800) {
    PORTB=0xFC;
  } else {
 PORTEbits.RE1=0;   
      PORTB=0xFE;
  }
  lcd_delay(1000);
}  
 
 SetLineNumber(1);
  
LCDDisplayStr("--------------------");
  lcd_delay(10000);
}
