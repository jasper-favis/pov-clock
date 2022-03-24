#include <stdint.h>
#include <stdbool.h>
#include "POV_text.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include <cstring>
#include <math.h>

//*****************************************************************************

volatile unsigned long millis  = 0;
volatile unsigned long micros  = 0;
unsigned long Real_time_millis = 0;
unsigned long Previous_real_time_millis = 0;

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long elapsed_loop_counter;
unsigned long counter_1;
unsigned long current_count;

unsigned int last_state = 0;
float one_rot_time      = 0;
float time_per_deg      = 0;

float delayTime = 1;
int min_or_hr   = 1;
int text_ok     = 0;

unsigned int SEC  = 0;
unsigned int MIN  = 45;
unsigned int HOUR = 2;

// HEX
//int zero[]  = {0x3e,0x45,0x49,0x51,0x3e};     // 0 0x30 48
//int one[]   = {0x00,0x40,0x7f,0x42,0x00};     // 1 0x31 49
//int two[]   = {0x46,0x49,0x51,0x61,0x42};     // 2 0x32 50
//int three[] = {0x31,0x4b,0x45,0x41,0x21};     // 3 0x33 51
//int four[]  = {0x10,0x7f,0x12,0x14,0x18};     // 4 0x34 52
//int five[]  = {0x39,0x45,0x45,0x45,0x27};     // 5 0x35 53
//int six[]   = {0x30,0x49,0x49,0x4a,0x3c};     // 6 0x36 54
//int seven[] = {0x03,0x05,0x09,0x71,0x01};     // 7 0x37 55
//int eight[] = {0x36,0x49,0x49,0x49,0x36};     // 8 0x38 56
//int nine[]  = {0x1e,0x29,0x49,0x49,0x06};     // 9 0x39 57

// DECIMAL
int zero[]  = {62,69,73,81,62};    // 0 0x30 48
int one[]   = {0,64,127,66,0};     // 1 0x31 49
int two[]   = {70,73,81,97,66};    // 2 0x32 50
int three[] = {112,75,69,65,33};   // 3 0x33 51
int four[]  = {16,127,18,20,24};   // 4 0x34 52
int five[]  = {57,69,69,69,39};    // 5 0x35 53
int six[]   = {48,73,73,74,60};    // 6 0x36 54
int seven[] = {3,5,9,113,1};       // 7 0x37 55
int eight[] = {54,73,73,73,54};    // 8 0x38 56
int nine[]  = {30,41,73,73,6};     // 9 0x39 57

// 5-bit Numbers
//int zero[]  = {30,35,37,41,30};    // 0 0x30 48
//int one[]   = {0,32,64,32,0};      // 1 0x31 49
//int two[]   = {34,37,41,49,32};    // 2 0x32 50
//int three[] = {112,75,69,65,33};   // 3 0x33 51
//int four[]  = {16,127,18,20,24};   // 4 0x34 52
//int five[]  = {57,69,69,69,39};    // 5 0x35 53
//int six[]   = {48,73,73,74,60};    // 6 0x36 54
//int seven[] = {3,5,9,113,1};       // 7 0x37 55
//int eight[] = {54,73,73,73,54};    // 8 0x38 56
//int nine[]  = {30,41,73,73,6};     // 9 0x39 57

int a[] = {126, 144, 144, 144, 126};
int b[] = {254, 146, 146, 146, 108};
int c[] = {254, 130, 130, 130, 130};
int d[] = {254, 130, 130, 130, 124};
int e[] = {254, 146, 146, 146, 146};
int f[] = {254, 144, 144, 144, 128};
int g[] = {124, 130, 138, 138, 76};
int h[] = {254, 16, 16, 16, 254};
int i[] = {130, 238, 130};
int j[] = {12, 2, 2, 2, 252};
int k[] = {254, 16, 40, 68, 130};
int l[] = {254, 2, 2, 2, 2};
int m[] = {254, 64, 32, 64, 254};
int n[] = {254, 32, 16, 8, 254};
int o[] = {124, 130, 130, 130, 124};
int p[] = {254, 136, 136, 136, 112};
int q[] = {124, 130, 138, 134, 126};
int r[] = {254, 144, 152, 148, 98};
int s[] = {100, 146, 146, 146, 76};
int t[] = {128, 128, 254, 128, 128};
int u[] = {252, 2, 2, 2, 252};
int v[] = {248, 4, 2, 4, 248};
int w[] = {254, 4, 8, 4, 254};
int x[] = {198, 40, 16, 40, 198};
int y[] = {224, 16, 14, 16, 224};
int z[] = {134, 138, 146, 162, 194};

int colon[] = {0,36,0};
int eos[]   = {0, 3, 2,0};
int excl[]  = {0, 254, 0};
int ques[]  = {64, 128, 138, 144, 96};

void PortFunctionInit(void)
{		
	  //////////////////////////////
    // Enable Peripheral Clocks //
	  //////////////////////////////
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	  //////////////////////////
		// Port B [Hall Sensor] //
	  //////////////////////////
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
	
	  /////////////////////////////////////
		// PD0, PB5, Port C, Port E [LEDs] //
	  /////////////////////////////////////
		GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
	  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
	
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
	
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);
		
		//////////////////////////
		// Port A [Transistors] //
		//////////////////////////
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

		///////////////////////////////////////
		// Port F [on-board switches & LEDs] //
		///////////////////////////////////////
		// Unlock GPIO Port F
		GPIO_PORTF_LOCK_R = 0x4C4F434B;   
		GPIO_PORTF_CR_R |= 0x01;           // allow changes to PF0
		
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
		
		//Enable pull-up on PF4 and PF0
		GPIO_PORTF_PUR_R |= 0x11; 
 }


//Globally enable interrupts 
void IntGlobalEnable(void)
{
    __asm("    cpsie   i\n");
}

//Globally disable interrupts 
void IntGlobalDisable(void)
{
    __asm("    cpsid   i\n");
}

void Timer0A_Init(unsigned long period)
{   
	volatile uint32_t ui32Loop; 
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; // activate timer0
  ui32Loop = SYSCTL_RCGC1_R;				     // Do a dummy read to insert a few cycles after enabling the peripheral.
  TIMER0_CTL_R &= ~0x00000001;           // disable timer0A during setup
  TIMER0_CFG_R = 0x00000000;             // configure for 32-bit timer mode
  TIMER0_TAMR_R = 0x00000002;            // configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;             // reload value
	NVIC_PRI4_R &= ~0xE0000000; 	         // configure Timer0A interrupt priority as 0
  NVIC_EN0_R |= 0x00080000;              // enable interrupt 19 in NVIC (Timer0A)
	TIMER0_IMR_R |= 0x00000001;            // arm timeout interrupt
  TIMER0_CTL_R |= 0x00000001;            // enable timer0A
}

void Timer1A_Init(unsigned long period)
{
	volatile uint32_t ui32Loop;
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER1;
	ui32Loop = SYSCTL_RCGC1_R;
	TIMER1_CTL_R &= ~0x00000001;
	TIMER1_CFG_R = 0x00000000;
	TIMER1_TAMR_R = 0x00000002;
	TIMER1_TAILR_R = period - 1;
	NVIC_PRI5_R &= ~0x00E00000;
	NVIC_EN0_R |= 0x00200000;
	TIMER1_IMR_R |= 0x00000001;
	TIMER1_CTL_R |= 0x00000001;
}

void PortB_Interrupt_Init(void)
{
	NVIC_EN0_R |= 0x00000002;  					// enable interrupt 0 in NVIC (GPIOA)
  NVIC_PRI0_R &= ~0x0000E000; 				// configure GPIOA interrupt priority as 0
	GPIO_PORTB_IM_R |= 0x01;   		      // arm interrupt on PA2
	GPIO_PORTB_IS_R &= ~0x01;           // PA2 is edge-sensitive
  GPIO_PORTB_IBE_R |= 0x01;        	  // PA2 both edges trigger 
	//GPIO_PORTF_IEV_R &= ~0x04;        	// PA2 falling edge trigger
}

void PortF_Interrupt_Init(void)
{
  NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= 0x00E00000; 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R |= 0x11;   	// PF0 and PF4 both edges trigger 
  //GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
	IntGlobalEnable();        		// globally enable interrupt
}


///////////////////////////////////////////////////////////////
//                                                           //
//                M I C R O S E C O N D S                    //
//                                                           //
///////////////////////////////////////////////////////////////
void Timer0A_Handler(void)
{
		// acknowledge flag for Timer0A
		TIMER0_ICR_R |= 0x00000001; 
		micros++;
}

///////////////////////////////////////////////////////////////
//                                                           //
//                M I L L I S E C O N D S                    //
//                                                           //
///////////////////////////////////////////////////////////////
void Timer1A_Handler(void)
{
		// acknowledge flag for Timer1A
		TIMER1_ICR_R |= 0x00000001;
		millis++;
}

///////////////////////////////////////////////////////////////
//                                                           //
//                  H A L L   S E N S O R                    //
//                                                           //
///////////////////////////////////////////////////////////////
void GPIOPortB_Handler(void)
{
		GPIO_PORTB_ICR_R |= 0x01;     // acknowledge

		GPIO_PORTF_DATA_R = 0x00;
		current_count = micros;
	
		if((GPIO_PORTB_DATA_R & 0x01) == 0)
		{
			if(last_state == 1)
			{
				last_state = 0;
				one_rot_time = current_count - counter_1;
				time_per_deg = one_rot_time/360.0f;
				previousMillis = micros;
				text_ok = 1;
			}
		}
		else if (last_state == 0)
		{
			last_state = 1;
			counter_1 = current_count;
		}
}

///////////////////////////////////////////////////////////////
//                                                           //
//                     S W I T C H E S                       //
//                                                           //
///////////////////////////////////////////////////////////////
void GPIOPortF_Handler(void)
{
		////////////////////////////////
		//      D E B O U N C E       //
		////////////////////////////////
		NVIC_EN0_R &= ~0x40000000; 
		SysCtlDelay(53333);	// Delay for a while
		NVIC_EN0_R |= 0x40000000; 
	
		if(min_or_hr == 1)
			GPIO_PORTF_DATA_R = 0x08;
		else
			GPIO_PORTF_DATA_R	= 0x04;
		
		////////////////////////////////
		//        SW1 AND !SW2        //
		////////////////////////////////
		if( (GPIO_PORTF_RIS_R&0x10) && ((GPIO_PORTF_RIS_R&0x01) == 0) )
		{
				GPIO_PORTF_ICR_R |= 0x10;   // Acknowledge
				
				// SW1 pressed
				if((GPIO_PORTF_DATA_R&0x10)==0x00)
				{
						GPIO_PORTF_DATA_R |= 0x02;
						if(min_or_hr == 1)
							MIN = (MIN + 1) % 60;
						else
							HOUR = (HOUR + 1) % 24;
				}
		}
		////////////////////////////////
		//        SW2 AND !SW1        //
		////////////////////////////////
		if( (GPIO_PORTF_RIS_R&0x01) && ((GPIO_PORTF_RIS_R&0x10) == 0) )
		{
				GPIO_PORTF_ICR_R |= 0x01;   // Acknowledge
				
				// SW2 pressed
				if((GPIO_PORTF_DATA_R&0x01)==0x00)
				{
						if(min_or_hr == 0)
						{
							 if(MIN == 0)
									MIN = 59;
							 else
									MIN = (MIN - 1) % 60;
						}
				}
				else
				{
						if(min_or_hr == 0)
						{
								if(HOUR == 0)
									HOUR = 23;
								else
									HOUR = (HOUR - 1) % 24;
						}
				}
		}
		////////////////////////////////
		//        SW1 AND SW2         //
		////////////////////////////////
		if( (GPIO_PORTF_RIS_R&0x10) && (GPIO_PORTF_RIS_R&0x01) )
		{
				GPIO_PORTF_ICR_R |= 0x10;   // Acknowledge
				GPIO_PORTF_ICR_R |= 0x01;   // Acknowledge
			
				if( ((GPIO_PORTF_DATA_R&0x10)==0x00) && ((GPIO_PORTF_DATA_R&0x01)==0x00) ) 
				{
						if(min_or_hr == 1)
						{
								min_or_hr = 0;
								GPIO_PORTF_DATA_R = 0x08;
						}
						else
						{
								min_or_hr = 1;
								GPIO_PORTF_DATA_R	= 0x04;
						}
				}
		}
}

								///////////////////////////////////////////////////////////////
								//                                                           //
								//                  D R A W   A   L I N E                    //
								//                                                           //
								///////////////////////////////////////////////////////////////
void draw_a_line(int this_line)
{
		int now_line;
		now_line = this_line;
//		if (now_line>=64) {GPIO_PORTD_DATA_R |= 0x01;  now_line-=64;} else {GPIO_PORTD_DATA_R &= ~0x01;}
//		if (now_line>=32) {GPIO_PORTB_DATA_R |= 0x20;  now_line-=32;} else {GPIO_PORTB_DATA_R &= ~0x20;}
//		if (now_line>=16) {GPIO_PORTC_DATA_R |= 0x10;  now_line-=16;} else {GPIO_PORTC_DATA_R &= ~0x10;}
//		if (now_line>=8)  {GPIO_PORTC_DATA_R |= 0x20;  now_line-=8;}  else {GPIO_PORTC_DATA_R &= ~0x20;}
//		if (now_line>=4)  {GPIO_PORTC_DATA_R |= 0x40;  now_line-=4;}  else {GPIO_PORTC_DATA_R &= ~0x40;}
//		if (now_line>=2)  {GPIO_PORTC_DATA_R |= 0x80;  now_line-=2;}  else {GPIO_PORTC_DATA_R &= ~0x80;}
//		if (now_line>=1)  {GPIO_PORTE_DATA_R |= 0x02;  now_line-=1;}  else {GPIO_PORTE_DATA_R &= ~0x02;}
	
//		if (now_line>=128){GPIO_PORTD_DATA_R |= 0x01;  now_line-=128;} else {GPIO_PORTD_DATA_R &= ~0x01;}
//		if (now_line>=64) {GPIO_PORTB_DATA_R |= 0x20;  now_line-=64;}  else {GPIO_PORTB_DATA_R &= ~0x20;}
//		if (now_line>=32) {GPIO_PORTC_DATA_R |= 0x10;  now_line-=32;}  else {GPIO_PORTC_DATA_R &= ~0x10;}
//		if (now_line>=16) {GPIO_PORTC_DATA_R |= 0x20;  now_line-=16;}  else {GPIO_PORTC_DATA_R &= ~0x20;}
//		if (now_line>=8)  {GPIO_PORTC_DATA_R |= 0x40;  now_line-=8;}   else {GPIO_PORTC_DATA_R &= ~0x40;}
//		if (now_line>=4)  {GPIO_PORTC_DATA_R |= 0x80;  now_line-=4;}   else {GPIO_PORTC_DATA_R &= ~0x80;}
//		if (now_line>=2)  {GPIO_PORTE_DATA_R |= 0x02;  now_line-=2;}   else {GPIO_PORTE_DATA_R &= ~0x02;}
//		if (now_line>=1)  {GPIO_PORTE_DATA_R |= 0x04;  now_line-=1;}   else {GPIO_PORTE_DATA_R &= ~0x04;}	

		if (now_line>=128){GPIO_PORTB_DATA_R |= 0x20;  now_line-=128;} else {GPIO_PORTB_DATA_R &= ~0x20;}
		if (now_line>=64) {GPIO_PORTC_DATA_R |= 0x10;  now_line-=64;}  else {GPIO_PORTC_DATA_R &= ~0x10;}
		if (now_line>=32) {GPIO_PORTC_DATA_R |= 0x20;  now_line-=32;}  else {GPIO_PORTC_DATA_R &= ~0x20;}
		if (now_line>=16) {GPIO_PORTC_DATA_R |= 0x40;  now_line-=16;}  else {GPIO_PORTC_DATA_R &= ~0x40;}
		if (now_line>=8)  {GPIO_PORTC_DATA_R |= 0x80;  now_line-=8;}   else {GPIO_PORTC_DATA_R &= ~0x80;}
		if (now_line>=4)  {GPIO_PORTE_DATA_R |= 0x02;  now_line-=4;}   else {GPIO_PORTE_DATA_R &= ~0x02;}
		if (now_line>=2)  {GPIO_PORTE_DATA_R |= 0x04;  now_line-=2;}   else {GPIO_PORTE_DATA_R &= ~0x04;}
		if (now_line>=1)  {GPIO_PORTE_DATA_R |= 0x08;  now_line-=1;}   else {GPIO_PORTE_DATA_R &= ~0x08;}
}

								///////////////////////////////////////////////////////////////
								//                                                           //
								//            D I S P L A Y   C H A R A C T E R              //
								//                                                           //
								///////////////////////////////////////////////////////////////
void displayChar(char cr, float line_delay)
{
	if (cr == 'a'){for (int i = 0; i <5; i++){draw_a_line(a[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'b'){for (int i = 0; i <5; i++){draw_a_line(b[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'c'){for (int i = 0; i <5; i++){draw_a_line(c[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'd'){for (int i = 0; i <5; i++){draw_a_line(d[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'e'){for (int i = 0; i <5; i++){draw_a_line(e[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'f'){for (int i = 0; i <5; i++){draw_a_line(f[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'g'){for (int i = 0; i <5; i++){draw_a_line(g[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'h'){for (int i = 0; i <5; i++){draw_a_line(h[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'i'){for (int itr = 0; itr <3; itr++){draw_a_line(i[itr]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'j'){for (int i = 0; i <5; i++){draw_a_line(j[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'k'){for (int i = 0; i <5; i++){draw_a_line(k[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'l'){for (int i = 0; i <5; i++){draw_a_line(l[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'm'){for (int i = 0; i <5; i++){draw_a_line(m[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'n'){for (int i = 0; i <5; i++){draw_a_line(n[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'o'){for (int i = 0; i <5; i++){draw_a_line(o[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'p'){for (int i = 0; i <5; i++){draw_a_line(p[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'q'){for (int i = 0; i <5; i++){draw_a_line(q[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'r'){for (int i = 0; i <5; i++){draw_a_line(r[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 's'){for (int i = 0; i <5; i++){draw_a_line(s[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 't'){for (int i = 0; i <5; i++){draw_a_line(t[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'u'){for (int i = 0; i <5; i++){draw_a_line(u[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'v'){for (int i = 0; i <5; i++){draw_a_line(v[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'w'){for (int i = 0; i <5; i++){draw_a_line(w[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'x'){for (int i = 0; i <5; i++){draw_a_line(x[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'y'){for (int i = 0; i <5; i++){draw_a_line(y[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == 'z'){for (int i = 0; i <5; i++){draw_a_line(z[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	
	if (cr == '!'){for (int i = 0; i <3; i++){draw_a_line(excl[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '?'){for (int i = 0; i <5; i++){draw_a_line(ques[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '.'){for (int i = 0; i <4; i++){draw_a_line(eos[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == ':'){for (int i = 0; i <3; i++){draw_a_line(colon[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	
	if (cr == '0'){for (int i = 0; i <5; i++){draw_a_line(zero[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '1'){for (int i = 0; i <5; i++){draw_a_line(one[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '2'){for (int i = 0; i <5; i++){draw_a_line(two[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '3'){for (int i = 0; i <5; i++){draw_a_line(three[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '4'){for (int i = 0; i <5; i++){draw_a_line(four[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '5'){for (int i = 0; i <5; i++){draw_a_line(five[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '6'){for (int i = 0; i <5; i++){draw_a_line(six[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '7'){for (int i = 0; i <5; i++){draw_a_line(seven[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '8'){for (int i = 0; i <5; i++){draw_a_line(eight[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	if (cr == '9'){for (int i = 0; i <5; i++){draw_a_line(nine[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
	SysCtlDelay(line_delay*2);
}

								///////////////////////////////////////////////////////////////
								//                                                           //
								//               D I S P L A Y   S T R I N G                 //
								//                                                           //
								///////////////////////////////////////////////////////////////
void displayString(char* s, float line_delay)
{
		for (int i = 0; i<=strlen(s); i++)
				displayChar(s[i],line_delay);
}


																	/////////////////////////////////////////////////////////////////
																	//                        ____    __  __     __                // 
																	//					||\    /|   ||    |     ||      || \   |           //
////////////////////////////////////					|| \  / |   ||____|     ||      ||  \  |           /////////////////////////////////// 
																	//					||  \/  |   ||    |   __||__    ||   \_|           //
																	/////////////////////////////////////////////////////////////////
int main(void)
{
		PortFunctionInit();
		Timer0A_Init(64);
		Timer1A_Init(16000);	
		PortB_Interrupt_Init();
		PortF_Interrupt_Init();  
	
		int d0;
		int d1;
		int d2;
		int d3;
		int d4;
		int d5;
		char time_display[] = "00:00:00";
		unsigned long prev = 0;
		unsigned long duration = 0;
	
	
		int countdown = 0;
		GPIO_PORTA_DATA_R |= 0x20;
	
		///////////////////////////////////
		//  D I S P L A Y   J A S P E R  //
		///////////////////////////////////
		while(countdown < 200000)
		{
				countdown = countdown + 1;
				currentMillis = micros;
				elapsed_loop_counter = currentMillis - previousMillis;
			
				delayTime = time_per_deg*60.5f;   
				if((elapsed_loop_counter >=  time_per_deg*(216)) && (elapsed_loop_counter <  time_per_deg*(217)) &&  text_ok)
				{ 
						displayString("jasper",delayTime);
						text_ok = 0;
				}
		}

		countdown = 0;
		while(countdown < 95000)
		{
				countdown = countdown + 1;
		}
		countdown = 0;
		
		///////////////////////////////////
		// D I S P L A Y   K A T R I N A //
		///////////////////////////////////
		while(countdown < 200000)
		{
				countdown = countdown + 1;
				currentMillis = micros;
				elapsed_loop_counter = currentMillis - previousMillis;
			
				delayTime = time_per_deg*60.5f;
				if((elapsed_loop_counter >=  time_per_deg*(216)) && (elapsed_loop_counter <  time_per_deg*(217)) &&  text_ok)
				{
						displayString("katrina", delayTime);
						text_ok=0;
				}
		}
		
		countdown = 0;
		while(countdown < 95000)
		{
				countdown = countdown + 1;
		}
	
		GPIO_PORTD_DATA_R |= 0x01; 
		prev = millis;
		while(1)
		{	
				GPIO_PORTA_DATA_R |= 0x20;
				duration = millis - prev;
				/////////////////////////////////////////////////////
				//                                                 //
				//        D I G I T A L   D I S P L A Y            //
				//                                                 //
				/////////////////////////////////////////////////////
				while(duration < 9000)
				{
						Real_time_millis = millis;  
					
						if(Real_time_millis - Previous_real_time_millis >= 1000)
						{
								Previous_real_time_millis += 1000;
								
								SEC = SEC + 1;
								if(SEC > 59)
								{
									SEC = 0;
									MIN = MIN + 1;
								}
								if(MIN > 59)
								{
									MIN = 0;
									HOUR = HOUR + 1;
								}
								if(HOUR > 23)
								{
									HOUR = 0;
								}
						}
						
						d0 = ((int)SEC % 10) + 48;
						d1 = floor((int)SEC/10 + 48);
						d2 = ((int)MIN % 10) + 48;
						d3 = floor((int)MIN/10 + 48);
						d4 = ((int)HOUR % 10) + 48;
						d5 = floor((int)HOUR/10 + 48);
						
						time_display[0] = d0;
						time_display[1] = d1;
						time_display[3] = d2;
						time_display[4] = d3;
						time_display[6] = d4;
						time_display[7] = d5;
				
						currentMillis = micros;
						elapsed_loop_counter = currentMillis - previousMillis;
					
						delayTime = time_per_deg*45.5f; //we want 35.5 degrees for each line of the letters   

						// Print starts at 85 deg
						if((elapsed_loop_counter >=  time_per_deg*(60)) && (elapsed_loop_counter <  time_per_deg*(61)) &&  text_ok)
						{ 
								displayString(time_display,delayTime);
								text_ok = 0;
						}
						
						duration = millis - prev;
				}
				/////////////////////////////////////////////////////
				//          E N D   O F   D I G I T A L            //
				/////////////////////////////////////////////////////
				
				prev += 9000;
				duration = millis - prev;
			
				/////////////////////////////////////////////////////
				//                                                 //
				//         A N A L O G    D I S P L A Y            //
				//                                                 //
				/////////////////////////////////////////////////////
				while(duration < 9000)
				{					
						Real_time_millis = millis;  
						
						if(Real_time_millis - Previous_real_time_millis >= 1000)
						{
								Previous_real_time_millis += 1000;
								
								SEC = SEC + 1;
								if(SEC > 59)
								{
										SEC = 0;
										MIN = MIN + 1;
								}
								if(MIN > 59)
								{
										MIN = 0;
										HOUR = HOUR + 1;
								}
								if(HOUR > 23)
								{
										HOUR = 0;
								}
						}
						
						currentMillis = micros;
						elapsed_loop_counter = currentMillis - previousMillis;
						
						/////////////////////////////////
						//         Second Hand         //
						/////////////////////////////////
						if(elapsed_loop_counter >=  time_per_deg*(SEC*6) &&  elapsed_loop_counter <  time_per_deg*( (SEC*6)+2) ) 
						{
							// (360 degrees)/(60 seconds) = 6 degrees per sec
							GPIO_PORTE_DATA_R |= 0x3E;   // pins 1, 2, 3, 4, 5 
							GPIO_PORTC_DATA_R |= 0xF0;   // pins 4, 5, 6, 7
							
							// Cyan
							GPIO_PORTA_DATA_R |= 0xC0;   // pins 6, 7
						}
						if(elapsed_loop_counter >= time_per_deg*((SEC*6)+2))
						{
							GPIO_PORTE_DATA_R &= ~0x3E;   // pins 1, 2, 3, 4, 5 
							GPIO_PORTC_DATA_R &= ~0xF0;   // pins 4, 5, 6, 7
							
							GPIO_PORTA_DATA_R &= ~0xC0;   // pins 6, 7
						}

						/////////////////////////////////
						//        Minute hand          //
						/////////////////////////////////
						if(elapsed_loop_counter >=  time_per_deg*(MIN*6) &&  elapsed_loop_counter <  time_per_deg*( (MIN*6)+3) )
						{
							// (360 degrees)/(60 minutes) = 6 degrees per min
							GPIO_PORTE_DATA_R |= 0x3E;   // pins 1, 2, 3, 4, 5 
							GPIO_PORTC_DATA_R |= 0xF0;   // pins 4, 5, 6, 7
							
							// Magenta
							GPIO_PORTA_DATA_R |= 0xA0;   // pins 5, 7
						}

						if(elapsed_loop_counter >= time_per_deg*((MIN*6)+1))
						{
							GPIO_PORTE_DATA_R &= ~0x3E;   // pins 1, 2, 3, 4, 5 
							GPIO_PORTC_DATA_R &= ~0xF0;   // pins 4, 5, 6, 7 
							
							GPIO_PORTA_DATA_R &= ~0xA0;   // pins 5, 7
						}

						/////////////////////////////////
						//          Hour Hand          //
						/////////////////////////////////
						if(elapsed_loop_counter >=  time_per_deg*(HOUR*30) &&  elapsed_loop_counter <  time_per_deg*( (HOUR*30)+3) )
						{
							// (360 degrees)/(24 hours) = 30 degrees per hour
							GPIO_PORTE_DATA_R |= 0x3E;   // pins 1, 2, 3, 4, 5 
							
							// Magenta
							GPIO_PORTA_DATA_R |= 0xA0;   // pins 5, 7
						}

						if(elapsed_loop_counter >= time_per_deg*( (HOUR*30)+1) )
						{
							GPIO_PORTE_DATA_R &= ~0x3E;   // pins 1, 2, 3, 4, 5 
							GPIO_PORTA_DATA_R &= ~0xA0;   // pins 5, 7
						} 
							
						duration = millis - prev;
				}
	      /////////////////////////////////////////////////////
				//           E N D   O F   A N A L O G             //
				/////////////////////////////////////////////////////
				prev += 9000;
		}
}


//************************************************************************************************************************//


//int main(void)
//{
//		PortFunctionInit();
//		Timer0A_Init(64);
//		Timer1A_Init(16000);	
//		Interrupt_Init();
//		GPIO_PORTA_DATA_R |= 0x20;
//		
//	
//		//char time_display[] = "00:00";
//	
//		while(1)
//		{
//				currentMillis = micros;
//				elapsed_loop_counter = currentMillis - previousMillis;
//			
//				delayTime = time_per_deg*45.5f; //we want 2 degrees for each line of the letters   

//				//This if here is to make sure I'll start printing at 216 deg so the text will be centered.
//				if((elapsed_loop_counter >=  time_per_deg*(60)) && (elapsed_loop_counter <  time_per_deg*(61)) &&  text_ok)
//				{ 
//					displayString("72:00:21",delayTime);
//						//delayMicroseconds(delayTime*10);
//					text_ok = 0;
//				}
//			
//		}
//}

						///////////////////////////////////////
						//      JASPER KATRINA DISPLAY       //
						///////////////////////////////////////

//		  unsigned long prev = 0;
//		  unsigned long duration = 0;
//			duration = millis - prev;
//			GPIO_PORTA_DATA_R ^= 0xE0;
//			while(duration < 4000)
//			{
//				currentMillis = micros;
//				elapsed_loop_counter = currentMillis - previousMillis;
//			
//				delayTime = time_per_deg*60.5f; //we want 2 degrees for each line of the letters   

//				//This if here is to make sure I'll start printing at 216 deg so the text will be centered.
//				if((elapsed_loop_counter >=  time_per_deg*(216)) && (elapsed_loop_counter <  time_per_deg*(217)) &&  text_ok)
//				{ 
//						displayString("jasper",delayTime);
//						//delayMicroseconds(delayTime*10);
//					text_ok = 0;
//				}
//				duration = millis - prev;
//				
//			}
//			prev +=4000;

//			duration = millis - prev;
//			GPIO_PORTA_DATA_R ^= 0xE0;
//			while(duration < 4000)
//			{
//				currentMillis = micros;
//				elapsed_loop_counter = currentMillis - previousMillis;
//			
//				delayTime = time_per_deg*60.5f;
//				if((elapsed_loop_counter >=  time_per_deg*(216)) && (elapsed_loop_counter <  time_per_deg*(217)) &&  text_ok)
//				{
//					displayString("katrina", delayTime);
//						text_ok=0;
//				}
//								duration = millis - prev;
//			}
//			prev +=4000;



////////////////////////////////////////////////////////////////
//                                                            //
//               O R I G I N A L    C O D E                   //
//                                                            //
////////////////////////////////////////////////////////////////

//#include <stdint.h>
//#include <stdbool.h>
//#include "POV_text.h"
//#include "inc/hw_types.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_gpio.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/gpio.h"
//#include "inc/tm4c123gh6pm.h"
//#include <cstring>
//#include <math.h>

////*****************************************************************************

//volatile unsigned long millis  = 0;
//volatile unsigned long micros  = 0;
//unsigned long Real_time_millis = 0;
//unsigned long Previous_real_time_millis = 0;

//unsigned long currentMillis;
//unsigned long previousMillis;
//unsigned long elapsed_loop_counter;
//unsigned long counter_1;
//unsigned long current_count;

//unsigned int last_state = 0;
//float one_rot_time      = 0;
//float time_per_deg      = 0;

//float delayTime = 1;
//int text_ok     = 0;

//float SEC  = 0;
//float MIN  = 45;
//float HOUR = 2;

//// HEX
////int zero[]  = {0x3e,0x45,0x49,0x51,0x3e};     // 0 0x30 48
////int one[]   = {0x00,0x40,0x7f,0x42,0x00};     // 1 0x31 49
////int two[]   = {0x46,0x49,0x51,0x61,0x42};     // 2 0x32 50
////int three[] = {0x31,0x4b,0x45,0x41,0x21};     // 3 0x33 51
////int four[]  = {0x10,0x7f,0x12,0x14,0x18};     // 4 0x34 52
////int five[]  = {0x39,0x45,0x45,0x45,0x27};     // 5 0x35 53
////int six[]   = {0x30,0x49,0x49,0x4a,0x3c};     // 6 0x36 54
////int seven[] = {0x03,0x05,0x09,0x71,0x01};     // 7 0x37 55
////int eight[] = {0x36,0x49,0x49,0x49,0x36};     // 8 0x38 56
////int nine[]  = {0x1e,0x29,0x49,0x49,0x06};     // 9 0x39 57

//// DECIMAL
//int zero[]  = {62,69,73,81,62};    // 0 0x30 48
//int one[]   = {0,64,127,66,0};     // 1 0x31 49
//int two[]   = {70,73,81,97,66};    // 2 0x32 50
//int three[] = {112,75,69,65,33};   // 3 0x33 51
//int four[]  = {16,127,18,20,24};   // 4 0x34 52
//int five[]  = {57,69,69,69,39};    // 5 0x35 53
//int six[]   = {48,73,73,74,60};    // 6 0x36 54
//int seven[] = {3,5,9,113,1};       // 7 0x37 55
//int eight[] = {54,73,73,73,54};    // 8 0x38 56
//int nine[]  = {30,41,73,73,6};     // 9 0x39 57

//// 5-bit Numbers
////int zero[]  = {30,35,37,41,30};    // 0 0x30 48
////int one[]   = {0,32,64,32,0};      // 1 0x31 49
////int two[]   = {34,37,41,49,32};    // 2 0x32 50
////int three[] = {112,75,69,65,33};   // 3 0x33 51
////int four[]  = {16,127,18,20,24};   // 4 0x34 52
////int five[]  = {57,69,69,69,39};    // 5 0x35 53
////int six[]   = {48,73,73,74,60};    // 6 0x36 54
////int seven[] = {3,5,9,113,1};       // 7 0x37 55
////int eight[] = {54,73,73,73,54};    // 8 0x38 56
////int nine[]  = {30,41,73,73,6};     // 9 0x39 57

//int a[] = {126, 144, 144, 144, 126};
//int b[] = {254, 146, 146, 146, 108};
//int c[] = {254, 130, 130, 130, 130};
//int d[] = {254, 130, 130, 130, 124};
//int e[] = {254, 146, 146, 146, 146};
//int f[] = {254, 144, 144, 144, 128};
//int g[] = {124, 130, 138, 138, 76};
//int h[] = {254, 16, 16, 16, 254};
//int i[] = {130, 238, 130};
//int j[] = {12, 2, 2, 2, 252};
//int k[] = {254, 16, 40, 68, 130};
//int l[] = {254, 2, 2, 2, 2};
//int m[] = {254, 64, 32, 64, 254};
//int n[] = {254, 32, 16, 8, 254};
//int o[] = {124, 130, 130, 130, 124};
//int p[] = {254, 136, 136, 136, 112};
//int q[] = {124, 130, 138, 134, 126};
//int r[] = {254, 144, 152, 148, 98};
//int s[] = {100, 146, 146, 146, 76};
//int t[] = {128, 128, 254, 128, 128};
//int u[] = {252, 2, 2, 2, 252};
//int v[] = {248, 4, 2, 4, 248};
//int w[] = {254, 4, 8, 4, 254};
//int x[] = {198, 40, 16, 40, 198};
//int y[] = {224, 16, 14, 16, 224};
//int z[] = {134, 138, 146, 162, 194};

//int colon[] = {0,36,0};
//int eos[]   = {0, 3, 2,0};
//int excl[]  = {0, 254, 0};
//int ques[]  = {64, 128, 138, 144, 96};

//void PortFunctionInit(void)
//{
//    // Enable Peripheral Clocks //
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	
//		// Port B (Hall Sensor) //
//    MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
//	
//		// PD0, PB5, Port C, Port E (LEDs) //
//		GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
//	  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
//	
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
//		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5);
//		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
//		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);
//	
//		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
//		MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3);
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);
//		
//		// Port A (Transistors) //
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

//}


////Globally enable interrupts 
//void IntGlobalEnable(void)
//{
//    __asm("    cpsie   i\n");
//}

////Globally disable interrupts 
//void IntGlobalDisable(void)
//{
//    __asm("    cpsid   i\n");
//}

//void Timer0A_Init(unsigned long period)
//{   
//	volatile uint32_t ui32Loop; 
//	
//	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; // activate timer0
//  ui32Loop = SYSCTL_RCGC1_R;				     // Do a dummy read to insert a few cycles after enabling the peripheral.
//  TIMER0_CTL_R &= ~0x00000001;           // disable timer0A during setup
//  TIMER0_CFG_R = 0x00000000;             // configure for 32-bit timer mode
//  TIMER0_TAMR_R = 0x00000002;            // configure for periodic mode, default down-count settings
//  TIMER0_TAILR_R = period-1;             // reload value
//	NVIC_PRI4_R &= ~0xE0000000; 	         // configure Timer0A interrupt priority as 0
//  NVIC_EN0_R |= 0x00080000;              // enable interrupt 19 in NVIC (Timer0A)
//	TIMER0_IMR_R |= 0x00000001;            // arm timeout interrupt
//  TIMER0_CTL_R |= 0x00000001;            // enable timer0A
//}

//void Timer1A_Init(unsigned long period)
//{
//	volatile uint32_t ui32Loop;
//	
//	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER1;
//	ui32Loop = SYSCTL_RCGC1_R;
//	TIMER1_CTL_R &= ~0x00000001;
//	TIMER1_CFG_R = 0x00000000;
//	TIMER1_TAMR_R = 0x00000002;
//	TIMER1_TAILR_R = period - 1;
//	NVIC_PRI5_R &= ~0x00E00000;
//	NVIC_EN0_R |= 0x00200000;
//	TIMER1_IMR_R |= 0x00000001;
//	TIMER1_CTL_R |= 0x00000001;
//}

//void Interrupt_Init(void)
//{
//	NVIC_EN0_R |= 0x00000002;  					// enable interrupt 0 in NVIC (GPIOA)
//  NVIC_PRI0_R &= ~0x0000E000; 				// configure GPIOA interrupt priority as 0
//	GPIO_PORTB_IM_R |= 0x01;   		      // arm interrupt on PA2
//	GPIO_PORTB_IS_R &= ~0x01;           // PA2 is edge-sensitive
//  GPIO_PORTB_IBE_R |= 0x01;        	  // PA2 both edges trigger 
//	//GPIO_PORTF_IEV_R &= ~0x04;        	// PA2 falling edge trigger
//	IntGlobalEnable();       		        // globally enable interrupt
//}


////////////////////////////////////
////     Microsecond counter      //
////////////////////////////////////
//void Timer0A_Handler(void)
//{
//		// acknowledge flag for Timer0A
//		TIMER0_ICR_R |= 0x00000001; 
//		micros++;
//}

////////////////////////////////////
////     Millisecond counter      //
////////////////////////////////////
//void Timer1A_Handler(void)
//{
//		// acknowledge flag for Timer1A
//		TIMER1_ICR_R |= 0x00000001;
//		millis++;
//}

////////////////////////////////////
////      Hall Effect Sensor      //
////////////////////////////////////
//void GPIOPortB_Handler(void)
//{
//		GPIO_PORTB_ICR_R |= 0x01;     // acknowledge

//		current_count = micros;
//	
//		if((GPIO_PORTB_DATA_R & 0x01) == 0)
//		{
//			if(last_state == 1)
//			{
//				last_state = 0;
//				one_rot_time = current_count - counter_1;
//				time_per_deg = one_rot_time/360.0f;
//				previousMillis = micros;
//				text_ok = 1;
//			}
//		}
//		else if (last_state == 0)
//		{
//			last_state = 1;
//			counter_1 = current_count;
//		}
//}

//													//////////////////////////////////
//													//    D R A W   A   L I N E     //
//													//////////////////////////////////
//void draw_a_line(int this_line)
//{
//		int now_line;
//		now_line = this_line;
////		if (now_line>=64){GPIO_PORTD_DATA_R |= 0x01;  now_line-=64;} else {GPIO_PORTD_DATA_R &= ~0x01;}
////		if (now_line>=32) {GPIO_PORTB_DATA_R |= 0x20;  now_line-=32;}  else {GPIO_PORTB_DATA_R &= ~0x20;}
////		if (now_line>=16) {GPIO_PORTC_DATA_R |= 0x10;  now_line-=16;}  else {GPIO_PORTC_DATA_R &= ~0x10;}
////		if (now_line>=8) {GPIO_PORTC_DATA_R |= 0x20;  now_line-=8;}  else {GPIO_PORTC_DATA_R &= ~0x20;}
////		if (now_line>=4)  {GPIO_PORTC_DATA_R |= 0x40;  now_line-=4;}   else {GPIO_PORTC_DATA_R &= ~0x40;}
////		if (now_line>=2)  {GPIO_PORTC_DATA_R |= 0x80;  now_line-=2;}   else {GPIO_PORTC_DATA_R &= ~0x80;}
////		if (now_line>=1)  {GPIO_PORTE_DATA_R |= 0x02;  now_line-=1;}   else {GPIO_PORTE_DATA_R &= ~0x02;}
//	
////		if (now_line>=128){GPIO_PORTD_DATA_R |= 0x01;  now_line-=128;} else {GPIO_PORTD_DATA_R &= ~0x01;}
////		if (now_line>=64) {GPIO_PORTB_DATA_R |= 0x20;  now_line-=64;}  else {GPIO_PORTB_DATA_R &= ~0x20;}
////		if (now_line>=32) {GPIO_PORTC_DATA_R |= 0x10;  now_line-=32;}  else {GPIO_PORTC_DATA_R &= ~0x10;}
////		if (now_line>=16) {GPIO_PORTC_DATA_R |= 0x20;  now_line-=16;}  else {GPIO_PORTC_DATA_R &= ~0x20;}
////		if (now_line>=8)  {GPIO_PORTC_DATA_R |= 0x40;  now_line-=8;}   else {GPIO_PORTC_DATA_R &= ~0x40;}
////		if (now_line>=4)  {GPIO_PORTC_DATA_R |= 0x80;  now_line-=4;}   else {GPIO_PORTC_DATA_R &= ~0x80;}
////		if (now_line>=2)  {GPIO_PORTE_DATA_R |= 0x02;  now_line-=2;}   else {GPIO_PORTE_DATA_R &= ~0x02;}
////		if (now_line>=1)  {GPIO_PORTE_DATA_R |= 0x04;  now_line-=1;}   else {GPIO_PORTE_DATA_R &= ~0x04;}	

//		if (now_line>=128){GPIO_PORTB_DATA_R |= 0x20;  now_line-=128;}  else {GPIO_PORTB_DATA_R &= ~0x20;}
//		if (now_line>=64) {GPIO_PORTC_DATA_R |= 0x10;  now_line-=64;}   else {GPIO_PORTC_DATA_R &= ~0x10;}
//		if (now_line>=32) {GPIO_PORTC_DATA_R |= 0x20;  now_line-=32;}   else {GPIO_PORTC_DATA_R &= ~0x20;}
//		if (now_line>=16) {GPIO_PORTC_DATA_R |= 0x40;  now_line-=16;}   else {GPIO_PORTC_DATA_R &= ~0x40;}
//		if (now_line>=8)  {GPIO_PORTC_DATA_R |= 0x80;  now_line-=8;}    else {GPIO_PORTC_DATA_R &= ~0x80;}
//		if (now_line>=4)  {GPIO_PORTE_DATA_R |= 0x02;  now_line-=4;}    else {GPIO_PORTE_DATA_R &= ~0x02;}
//		if (now_line>=2)  {GPIO_PORTE_DATA_R |= 0x04;  now_line-=2;}    else {GPIO_PORTE_DATA_R &= ~0x04;}
//		if (now_line>=1)  {GPIO_PORTE_DATA_R |= 0x08;  now_line-=1;}    else {GPIO_PORTE_DATA_R &= ~0x08;}
//}

//													///////////////////////////////////////////
//													//   D I S P L A Y   C H A R A C T E R   //
//													///////////////////////////////////////////
//void displayChar(char cr, float line_delay)
//{
//	if (cr == 'a'){for (int i = 0; i <5; i++){draw_a_line(a[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'b'){for (int i = 0; i <5; i++){draw_a_line(b[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'c'){for (int i = 0; i <5; i++){draw_a_line(c[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'd'){for (int i = 0; i <5; i++){draw_a_line(d[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'e'){for (int i = 0; i <5; i++){draw_a_line(e[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'f'){for (int i = 0; i <5; i++){draw_a_line(f[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'g'){for (int i = 0; i <5; i++){draw_a_line(g[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'h'){for (int i = 0; i <5; i++){draw_a_line(h[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'i'){for (int itr = 0; itr <3; itr++){draw_a_line(i[itr]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'j'){for (int i = 0; i <5; i++){draw_a_line(j[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'k'){for (int i = 0; i <5; i++){draw_a_line(k[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'l'){for (int i = 0; i <5; i++){draw_a_line(l[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'm'){for (int i = 0; i <5; i++){draw_a_line(m[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'n'){for (int i = 0; i <5; i++){draw_a_line(n[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'o'){for (int i = 0; i <5; i++){draw_a_line(o[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'p'){for (int i = 0; i <5; i++){draw_a_line(p[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'q'){for (int i = 0; i <5; i++){draw_a_line(q[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'r'){for (int i = 0; i <5; i++){draw_a_line(r[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 's'){for (int i = 0; i <5; i++){draw_a_line(s[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 't'){for (int i = 0; i <5; i++){draw_a_line(t[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'u'){for (int i = 0; i <5; i++){draw_a_line(u[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'v'){for (int i = 0; i <5; i++){draw_a_line(v[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'w'){for (int i = 0; i <5; i++){draw_a_line(w[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'x'){for (int i = 0; i <5; i++){draw_a_line(x[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'y'){for (int i = 0; i <5; i++){draw_a_line(y[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == 'z'){for (int i = 0; i <5; i++){draw_a_line(z[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	
//	if (cr == '!'){for (int i = 0; i <3; i++){draw_a_line(excl[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '?'){for (int i = 0; i <5; i++){draw_a_line(ques[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '.'){for (int i = 0; i <4; i++){draw_a_line(eos[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == ':'){for (int i = 0; i <3; i++){draw_a_line(colon[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	
//	if (cr == '0'){for (int i = 0; i <5; i++){draw_a_line(zero[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '1'){for (int i = 0; i <5; i++){draw_a_line(one[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '2'){for (int i = 0; i <5; i++){draw_a_line(two[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '3'){for (int i = 0; i <5; i++){draw_a_line(three[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '4'){for (int i = 0; i <5; i++){draw_a_line(four[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '5'){for (int i = 0; i <5; i++){draw_a_line(five[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '6'){for (int i = 0; i <5; i++){draw_a_line(six[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '7'){for (int i = 0; i <5; i++){draw_a_line(seven[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '8'){for (int i = 0; i <5; i++){draw_a_line(eight[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	if (cr == '9'){for (int i = 0; i <5; i++){draw_a_line(nine[i]);SysCtlDelay(line_delay);}draw_a_line(0);}
//	SysCtlDelay(line_delay*2);
//}

//													///////////////////////////////////////
//													//   D I S P L A Y   S T R I N G     //
//													///////////////////////////////////////
//void displayString(char* s, float line_delay)
//{
//		for (int i = 0; i<=strlen(s); i++)
//				displayChar(s[i],line_delay);
//}


//int main(void)
//{
//		int d0;
//		int d1;
//		int d2;
//		int d3;
//		int d4;
//		int d5;
//	
//		PortFunctionInit();
//		Timer0A_Init(64);
//		Timer1A_Init(16000);	
//		Interrupt_Init();
//		GPIO_PORTA_DATA_R |= 0x20;
//	
//		char time_display[] = "00:00:00";
//	
//		while(1)
//		{
//				Real_time_millis = millis;  
//			
//				if(Real_time_millis - Previous_real_time_millis >= 1000)
//				{
//						Previous_real_time_millis += 1000;
//						
//						SEC = SEC + 1;
//						if(SEC > 59)
//						{
//							SEC = 0;
//							MIN = MIN + 1;
//						}
//						if(MIN > 59)
//						{
//							MIN = 0;
//							HOUR = HOUR + 1;
//						}
//						if(HOUR > 23)
//						{
//							HOUR = 0;
//						}
//				}
//				
//				d0 = ((int)SEC % 10) + 48;
//				d1 = floor((int)SEC/10 + 48);
//				d2 = ((int)MIN % 10) + 48;
//				d3 = floor((int)MIN/10 + 48);
//				d4 = ((int)HOUR % 10) + 48;
//				d5 = floor((int)HOUR/10 + 48);
//				
//				time_display[0] = d0;
//				time_display[1] = d1;
//				time_display[3] = d2;
//				time_display[4] = d3;
//				time_display[6] = d4;
//				time_display[7] = d5;
//		
//				currentMillis = micros;
//				elapsed_loop_counter = currentMillis - previousMillis;
//			
//				delayTime = time_per_deg*45.5f; //we want 35.5 degrees for each line of the letters   

//				// Print starts at 85 deg
//				if((elapsed_loop_counter >=  time_per_deg*(60)) && (elapsed_loop_counter <  time_per_deg*(61)) &&  text_ok)
//				{ 
//						displayString(time_display,delayTime);
//						text_ok = 0;
//				}
//		}
//}




