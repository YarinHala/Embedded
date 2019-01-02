/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:   PIC18 or PIC24 USB Microcontrollers
 Hardware:    The code is natively intended to be used on the following
        hardware platforms: PICDEM™ FS USB Demo Board, 
        PIC18F87J50 FS USB Plug-In Module, or
        Explorer 16 + PIC24 USB PIM.  The firmware may be
        modified for use on other USB platforms by editing the
        HardwareProfile.h file.
 Complier:    Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:   Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/


//	========================	INCLUDES	========================
#ifdef _VISUAL
#include "VisualSpecials.h"
#endif // VISUAL

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"


#include "mtouch.h"

#include "BMA150.h"

#include "oled.h"

#include "soft_start.h"


//	========================	CONFIGURATION	========================

#if defined(PIC18F46J50_PIM)
   //Watchdog Timer Enable bit:
     #pragma config WDTEN = OFF          //WDT disabled (control is placed on SWDTEN bit)
   //PLL Prescaler Selection bits:
     #pragma config PLLDIV = 3           //Divide by 3 (12 MHz oscillator input)
   //Stack Overflow/Underflow Reset Enable bit:
     #pragma config STVREN = ON            //Reset on stack overflow/underflow enabled
   //Extended Instruction Set Enable bit:
     #pragma config XINST = OFF          //Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
   //CPU System Clock Postscaler:
     #pragma config CPUDIV = OSC1        //No CPU system clock divide
   //Code Protection bit:
     #pragma config CP0 = OFF            //Program memory is not code-protected
   //Oscillator Selection bits:
     #pragma config OSC = ECPLL          //HS oscillator, PLL enabled, HSPLL used by USB
   //Secondary Clock Source T1OSCEN Enforcement:
     #pragma config T1DIG = ON           //Secondary Oscillator clock source may be selected
   //Low-Power Timer1 Oscillator Enable bit:
     #pragma config LPT1OSC = OFF        //Timer1 oscillator configured for higher power operation
   //Fail-Safe Clock Monitor Enable bit:
     #pragma config FCMEN = OFF           //Fail-Safe Clock Monitor disabled
   //Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit:
     #pragma config IESO = OFF           //Two-Speed Start-up disabled
   //Watchdog Timer Postscaler Select bits:
     #pragma config WDTPS = 32768        //1:32768
   //DSWDT Reference Clock Select bit:
     #pragma config DSWDTOSC = INTOSCREF //DSWDT uses INTOSC/INTRC as reference clock
   //RTCC Reference Clock Select bit:
     #pragma config RTCOSC = T1OSCREF    //RTCC uses T1OSC/T1CKI as reference clock
   //Deep Sleep BOR Enable bit:
     #pragma config DSBOREN = OFF        //Zero-Power BOR disabled in Deep Sleep (does not affect operation in non-Deep Sleep modes)
   //Deep Sleep Watchdog Timer Enable bit:
     #pragma config DSWDTEN = OFF        //Disabled
   //Deep Sleep Watchdog Timer Postscale Select bits:
     #pragma config DSWDTPS = 8192       //1:8,192 (8.5 seconds)
   //IOLOCK One-Way Set Enable bit:
     #pragma config IOL1WAY = OFF        //The IOLOCK bit (PPSCON<0>) can be set and cleared as needed
   //MSSP address mask:
     #pragma config MSSP7B_EN = MSK7     //7 Bit address masking
   //Write Protect Program Flash Pages:
     #pragma config WPFP = PAGE_1        //Write Protect Program Flash Page 0
   //Write Protection End Page (valid when WPDIS = 0):
     #pragma config WPEND = PAGE_0       //Write/Erase protect Flash Memory pages starting at page 0 and ending with page WPFP[5:0]
   //Write/Erase Protect Last Page In User Flash bit:
     #pragma config WPCFG = OFF          //Write/Erase Protection of last page Disabled
   //Write Protect Disable bit:
     #pragma config WPDIS = OFF          //WPFP[5:0], WPEND, and WPCFG bits ignored
  
#else
    #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif



//	========================	Global VARIABLES	========================
#pragma udata
//You can define Global Data Elements here

//	========================	PRIVATE PROTOTYPES	========================
static void InitializeSystem(void);
static void ProcessIO(void);
static void UserInit(void);
static void YourHighPriorityISRCode();
static void YourLowPriorityISRCode();

BOOL CheckButtonPressed(void);

//	========================	VECTOR REMAPPING	========================
#if defined(__18CXX)
  //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
  //the reset, high priority interrupt, and low priority interrupt
  //vectors.  However, the current Microchip USB bootloader 
  //examples are intended to occupy addresses 0x00-0x7FF or
  //0x00-0xFFF depending on which bootloader is used.  Therefore,
  //the bootloader code remaps these vectors to new locations
  //as indicated below.  This remapping is only necessary if you
  //wish to program the hex file generated from this project with
  //the USB bootloader.  If no bootloader is used, edit the
  //usb_config.h file and comment out the following defines:
  //#define PROGRAMMABLE_WITH_SD_BOOTLOADER
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
    #define REMAPPED_RESET_VECTOR_ADDRESS     0xA000
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0xA008
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0xA018
  #else 
    #define REMAPPED_RESET_VECTOR_ADDRESS     0x00
    #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS  0x08
    #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS 0x18
  #endif
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  extern void _startup (void);        // See c018i.c in your C18 compiler dir
  #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
  void _reset (void)
  {
      _asm goto _startup _endasm
  }
  #endif
  #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
  void Remapped_High_ISR (void)
  {
       _asm goto YourHighPriorityISRCode _endasm
  }
  #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
  void Remapped_Low_ISR (void)
  {
       _asm goto YourLowPriorityISRCode _endasm
  }
  
  #if defined(PROGRAMMABLE_WITH_SD_BOOTLOADER)
  //Note: If this project is built while one of the bootloaders has
  //been defined, but then the output hex file is not programmed with
  //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
  //As a result, if an actual interrupt was enabled and occured, the PC would jump
  //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
  //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
  //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
  //would effective reset the application.
  
  //To fix this situation, we should always deliberately place a 
  //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
  //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
  //hex file of this project is programmed with the bootloader, these sections do not
  //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
  //programmed using the bootloader, then the below goto instructions do get programmed,
  //and the hex file still works like normal.  The below section is only required to fix this
  //scenario.
  #pragma code HIGH_INTERRUPT_VECTOR = 0x08
  void High_ISR (void)
  {
       _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #pragma code LOW_INTERRUPT_VECTOR = 0x18
  void Low_ISR (void)
  {
       _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
  }
  #endif  //end of "#if defined(||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER))"

  #pragma code
  
//	========================	Application Interrupt Service Routines	========================
  //These are your actual interrupt handling routines.
  #pragma interrupt YourHighPriorityISRCode
  void YourHighPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie fast", since this is in a #pragma interrupt section 
  #pragma interruptlow YourLowPriorityISRCode
  void YourLowPriorityISRCode()
  {
    //Check which interrupt flag caused the interrupt.
    //Service the interrupt
    //Clear the interrupt flag
    //Etc.
  
  } //This return will be a "retfie", since this is in a #pragma interruptlow section 
#endif




//	========================	Board Initialization Code	========================
#pragma code
#define ROM_STRING rom unsigned char*

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the application code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{

  /* Initialize the mTouch library */
  mTouchInit();

  /* Call the mTouch callibration function */
  mTouchCalibrate();

  /* Initialize the accelerometer */
  InitBma150(); 

  /* Initialize the oLED Display */
   ResetDevice();  
   FillDisplay(0x00);
   //oledPutROMString((ROM_STRING)" PIC18F Starter Kit  ",0,0);
}//end UserInit


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
	// Soft Start the APP_VDD
	while(!AppPowerReady())
		;

    #if defined(PIC18F46J50_PIM)
  //Configure all I/O pins to use digital input buffers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
       
    UserInit();

}//end InitializeSystem


//	========================	Application Code	========================

BOOL CheckButtonPressed()
{
    static char buttonPressed = FALSE;
    static unsigned long buttonPressCounter = 0;

    if(PORTBbits.RB0 == 0)
    {
		oledWriteCharInPlace(0x24,2,3*6);
        if(buttonPressCounter++ > 7000)
        {		
            buttonPressCounter = 0;
            buttonPressed = TRUE;
        }
    }
    else
    {
		oledWriteCharInPlace(0x5f,2,3*6);
        if(buttonPressed == TRUE)
        {
            if(buttonPressCounter == 0)
            {
                buttonPressed = FALSE;
                return TRUE;
            }
            else
            {	
                buttonPressCounter--;
            }
        }
    }

    return FALSE;
}




/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
void printScreenUI(){
	//Tempature and Button and Potentiometer
	oledPutROMString("P:",0,0);
	oledPutROMString("[",0,7*6);
	oledPutROMString("]",0,98);
	oledPutROMString("T:",1,0);
	oledPutROMString("@",1,5*6);
	oledPutROMString("B:",2,0);
	oledPutROMString("___",2,2*6);
	//Axles Bars 
	oledPutROMString("X:",6,0);
	oledPutROMString("[     ?     ]",6,6*6);
	oledPutROMString("Y:",7,1);
	oledPutROMString("[     ?     ]",7,6*6);
	//Touch Pads 
	oledPutROMString("  L!R  ",4,21);

	//Scroll bar
	oledPutROMString("U",0,20*6);	
	oledPutROMString("_",1,20*6);
	oledPutROMString("|",2,20*6);
	oledPutROMString("!",3,20*6);
	oledPutROMString("|",4,20*6);
	oledPutROMString("~",5,20*6);
	oledPutROMString("D",6,20*6);

}
//arrange the bite's according to shifting
int getValueFromBytesOfAccelerometer(BYTE msb,BYTE lsb){
	
	int temp_read = 0;
	int con = 0;
	temp_read = msb;
	temp_read = temp_read << 8;
	temp_read = temp_read + lsb;
	temp_read = temp_read >> 6;
	con = temp_read & 0x200;
	//making it nagtvie or postive num
	con = con >> 9;
	if(con){
		temp_read = temp_read + 0xFC00;
	}
	return temp_read ;
}



// inline function to swap two numbers
void swap(char *x, char *y) {
	char a;
	char t = *x;
	a = *y;
	*x = *y;
	*y = t;
}

// function to reverse buffer[i..j]
char* reverse(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]);

	return buffer;
}

int abs(int val){
	if(val < 0){		
		val = ~val;	
		val = val +1;		
	}
	return val;
}

// Iterative function to implement itoa() function in C
char* itoa_new(int value, char* buffer,int row,int start){

	// consider absolute value of number
	int counter = 0;
	int n = 0;
	char buff[5] = {' ',' ',' ',' ',' '};
	int i = 0;
	int sign = 0;
	char a;
	n = abs(value);
	

	while (n)
	{
		int r = n % 10;

		if (r >= 10){
			buffer[i] = 65 + (r - 10);
			a = buffer[i];
			buff[i] = 48 + r;
		}
		else{
			buffer[i] = 48 + r;//48
			a = buffer[i];
			buff[i] = 48 + r;
		}
		i++;
		n = n / 10;
	}

	// if number is 0
	if (i == 0){
		buffer[i] = '0';
		a = buffer[i];
		i++;
	}
	
	//clac  the amount of count to graphic bar (18.96 -> rounded to ~20 for 1 pixcell out of +-512);
	n = abs(value);
	counter = n/20;
	if(counter > 0){
		//counter = counter+10; 
	}
	// is preceded with a minus sign (-)
	// With any other base, value is always considered unsigned value < 0
	if (value < 0){
		buffer[i] = '-';
		a = buffer[i];
		buff[i] = '-';
		i++;
		oledWriteGrapicBar(1,counter,row,start);	
	}
	else{
		oledWriteGrapicBar(-1,counter,row,start);
	}

	buffer[i] = '\0'; // null terminate string
	buff[i] = '\0';
	// reverse the string and return it
	return reverse(buffer, 0, i - 1);
}



void main(void)
{
   	static char buff[64];
	static char buff1[64];
	static char* buffer;
	char buffer_zero[5] = {' ',' ',' ',' ','\0'};
	BYTE x_msb,x_lsb,y_msb,y_lsb,z_msb,z_lsb,x = 0;
	int a2d = 0 ,temp = 0,tempature = 0;
	int check_new = 0,result = 0;
	int b_preesed =1;
	int pad_r = 0,pad_l = 0,pad_u = 0,pad_d = 0;
	char q[3] = {'#',' ','\0'};
	int direction = -1,change = -1;
    InitializeSystem();


	printScreenUI();
    while(1)                            //Main is Usualy an Endless Loop
    {
           
//Potentiometer Section
		//ADCON0 = 0x13;
		a2d = ((int)ADRESH << 8) + ADRESL;	
		itoa_new(a2d,buffer,0,48);
		sprintf(buff1,"%4d",a2d);
		oledPutString(buff1,0,12);

//Button Section
		CheckButtonPressed();
	
		//Tempature Section 
		tempature = BMA150_ReadByte(8);
		tempature = (tempature -32)*5/9;
		sprintf(buff,"%3d",tempature);
		oledPutString(buff,1,2*6);

//Accelerometer Section
		//x axle
		x_msb = BMA150_ReadByte(3);
		x_lsb = BMA150_ReadByte(2);
		temp = getValueFromBytesOfAccelerometer(x_msb,x_lsb);
		check_new = x_lsb & 0x0001;
		if(check_new){
			buffer = itoa_new(temp,buffer,6,76);
			oledPutString(buffer_zero,6,2*6);
			oledPutString(buffer,6,2*6);
		}		
		//y axle
		check_new = 0;
		y_msb = BMA150_ReadByte(5);
		y_lsb = BMA150_ReadByte(4);
		temp = getValueFromBytesOfAccelerometer(y_msb,y_lsb);
		check_new = y_lsb & 0x0001;
		if(check_new){
			buffer = itoa_new(temp,buffer,7,76);
			oledPutString(buffer_zero,7,2*6);
			oledPutString(buffer,7,2*6);
		}		
		//z axle
		check_new = 0;
		z_msb = BMA150_ReadByte(7);
		z_lsb = BMA150_ReadByte(6);
		temp = getValueFromBytesOfAccelerometer(z_msb,z_lsb);
		check_new = z_lsb & 0xFFFE;
		if(check_new){
			//buffer = itoa_new(temp,buffer,5);
			//oledPutString(buffer_zero,5,2*6);
			//oledPutString(buffer,5,2*6);
			if(temp < -60 && temp > -70){
				oledPutROMString("[     ?     ]",6,6*6);
				oledPutROMString("[     ?     ]",7,6*6);
			}
		}
		
				
	
//Touch Pads Section
		mTouchCalibrate();
		pad_r = mTouchReadButton(0);//R
		pad_u = mTouchReadButton(1);//U
		pad_d = mTouchReadButton(2);//D
		pad_l = mTouchReadButton(3);//L
		ADCON0 = 0x13;
		//R Pad
		if(pad_r < 900){
			oledWriteCharInPlace(0x23,4,54);	
		}
		else{
			oledWriteCharInPlace(0x20,4,54);
		}

		//L Pad
		if(pad_l < 900){
			oledWriteCharInPlace(0x23,4,25);	
		}
		else{
			oledWriteCharInPlace(0x20,4,25);
		}
		
		//reseting view to defult U = |, D = |
		if(pad_u > 920 && pad_d > 920){
			oledWriteCharInPlace(0x7c,2,20*6);
			oledWriteCharInPlace(0x7c,4,20*6);
		}
		//reset direaction and change indiraction to defult
		if(pad_u > 940 && pad_d > 940){
			change = 0;
			direction = -1;
		}
		//understanding where the user want to go
		//down
		if(pad_d < 940 && change == 0){
			direction = 0;
			change =1;
		}
		//up
		if(pad_u < 940 && change == 0){
			direction = 1;
			change =1;
		}
		
		//casing the event to where the value of up and down where in this secound =>or ***
		if(pad_u - pad_d > 50 ){
			if(pad_u < 920 &&  pad_d < 920 && direction == 0){
				oledWriteCharInPlace(0x61,2,20*6);
				oledWriteCharInPlace(0x7c,4,20*6);
			}
		
		}
		//or in this second -***
		 if(pad_d - pad_u > 50){
			if(pad_u < 920 &&  pad_d < 920 && direction == 1){	
				oledWriteCharInPlace(0x7c,2,20*6);
				oledWriteCharInPlace(0x56,4,20*6);
			}
		}

//Print thoch pad volt value next to each belong
//by dufult it not show just clear the /* */
		
		//puting out LEFT value
		//buffer = itoa_new(pad_l,buffer,-1,0);
		//oledPutString(buffer_zero,4,0);
		//oledPutString(buffer,4,0);

		/*puting out RIGHT value*/
		//buffer = itoa_new(pad_r,buffer,-1,0);
		//oledPutString(buffer_zero,4,67);
		//oledPutString(buffer,4,67);
		
		//Scroll Section
		//puting out UP value
		//buffer = itoa_new(pad_u,buffer,-1,0);
		//oledPutString(buffer_zero,2,6*16);
		//oledPutString(buffer,2,6*16);
		//puting out DOWN value
		//buffer = itoa_new(pad_d,buffer,-1,0);
		//oledPutString(buffer_zero,4,16*6);
		//oledPutString(buffer,4,16*6);	



	}
	
}//end main


/** EOF main.c *************************************************/
//#endif
