//----------------------- Includes and Imports --------------------------------

#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "tm4c123gh6pm.h"
#include "stdio.h"

// -------------------------- msec to ticks convertor -------------------------

int CalculateTicks(int msec){
  return ((msec) * (16 * 1000))-1 ;
}

// ----------------------------- State Machine --------------------------------

// ----------------------------- Normal Traffic Lights ------------------------

// Light 1 Red, Light 2 Green, Light 2 yellow, Light 2 Red, Light 1 Green, Light 1 Yellow

enum state{L1R, L2G, L2Y, L2R, L1G, L1Y}; 
enum state CurrState = 0;
enum state NextState = 1;

// ----------------------------- Pedestrian Lights ----------------------------

// -Both Red, Light 1 Green Light 2 Red, Light 1 Red Light 2 Green, Both Green-
enum state2{BR, L1GL2R, L1RL2G, BG};
enum state2 CurrState2 = 0;
enum state2 NextState2 = 1;


// ---------------------------- Light Turning Functions -----------------------

// ---------------------------- First Set of Lights ---------------------------
void Light1(char c){
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,0); // Turn off all LEDs
  if(c=='R'){ // if it's red turn on the pin corresponding to the red LED
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5 , GPIO_PIN_5);
  } else if (c=='Y'){ // if it's yellow turn on the pin corresponding to the yellow LED
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6 , GPIO_PIN_6);
  } else if (c=='G'){ // if it's green turn on the pin corresponding to the green LED
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7 , GPIO_PIN_7);
  } else{
  return; // otherwise return nothing
  }
}


// ---------------------------- Second Set of Lights --------------------------
void Light2(char c){
  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,0); // Turn off all LEDs
  if(c=='R'){ // if it's red turn on the pin corresponding to the red LED
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 , GPIO_PIN_1);
  } else if (c=='Y'){ // if it's yellow turn on the pin corresponding to the yellow LED
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2 , GPIO_PIN_2);
  } else if (c=='G'){ // if it's green turn on the pin corresponding to the green LED
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3 , GPIO_PIN_3);
  } else{
  return; // otherwise return nothing
  }
}


// ---------------------------- Realignment Function --------------------------

void ReAlign(void){
    if(CurrState == 1){ // if the current state is 1, turn on red LED on the first set of lights
     Light1('R');
    } else if(CurrState == 2){ // if the current state is 2, turn on green LED on the second set of lights
     Light2('G');
    } else if(CurrState == 3){ // if the current state is 3, turn on yellow LED on the second set of lights
     Light2('Y');
    } else if(CurrState == 4){ // if the current state is 4, turn on red LED on the second set of lights
     Light2('R');
    } else if(CurrState == 5){ // if the current state is 5, turn on green LED on the first set of lights
     Light1('G');
    } else if(CurrState == 0){ // if the current state is 0, turn on yellow LED on the first set of lights
     Light1('Y');
    }
}


// ---------------------------- Timer Delay Functions -------------------------
// ---------------------------- Timer 0A Delay Function -----------------------

void Time0A_delay(int period){
  TimerDisable(TIMER0_BASE, TIMER_A); // Disable Timer 1
  TimerLoadSet(TIMER0_BASE, TIMER_A, CalculateTicks(period)); // CalculateTicks is used to take input in msec
  // This function sets the reload value for the timer
  TimerEnable(TIMER0_BASE, TIMER_A); // Enable Timer 1
}


// ---------------------------- Timer 1A Delay Function -----------------------

void Time1A_delay(int period){
  // Configuration Sandwich
  TimerDisable(TIMER1_BASE, TIMER_A); // Disable Timer 1
  TimerLoadSet(TIMER1_BASE, TIMER_A, CalculateTicks(period)); // CalculateTicks is used to take input in msec
  TimerEnable(TIMER1_BASE, TIMER_A); // Enable Timer 1
}



// ------------------------ Handler Functions ------------------------------

// ------------------------ Timer 0A Handler ------------------------------
void Timer0AHandler(void){
  // State Machine
    if(CurrState == 0){
     NextState = 1; //Set Next State to be 1
     Light1('R'); // Turn Red LED corresponding to first set of lights
     Time0A_delay(1000); // Set Delay 1 second
    } else if(CurrState == 1){
     NextState = 2; //Set Next State to be 2
     Light2('G'); // Turn Green LED corresponding to second set of lights
     Time0A_delay(5000); // Set Delay 5 second
    } else if(CurrState == 2){
     NextState = 3; //Set Next State to be 3
     Light2('Y'); // Turn Yellow LED corresponding to second set of lights
     Time0A_delay(2000); // Set Delay 2 second
    } else if(CurrState == 3){
     NextState = 4; //Set Next State to be 4
     Light2('R'); //Turn Red LED corresponding to second set of lights
     Time0A_delay(1000); // Set Delay 1 second
    } else if(CurrState == 4){
     NextState = 5; //Set Next State to be 5
     Light1('G'); // Turn Green LED corresponding to first set of lights
     Time0A_delay(5000); // Set Delay 5 second
    } else if(CurrState == 5){
     NextState = 0; //Set Next State to be 6
     Light1('Y'); // Turn Yellow LED corresponding to first set of lights
     Time0A_delay(2000); // Set Delay 2 second
    }
  CurrState = NextState; // Set Current State = Next state for next iteration
  TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT); // Clear Interrupt Flag
}

// ----------------- Timer 1A Handler / Pedestrian Timer ----------------------

void Timer1AHandler(void){
    TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT); // Clear interrupt flag
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0);
    // Turn off all Pedestrian LEDs
  if(CurrState2 == 0){
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on 1st Red LED
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on 2nd Red LED
    Time1A_delay(2000); // Delay for 2 seconds
    ReAlign(); // Realignment function to set the traffic back to previous state
    TimerEnable(TIMER0_BASE, TIMER_A);
  } else if (CurrState2 == 1){
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // Turn on 1st GREEN LED
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PIN_6); // Turn on 2nd RED LED
    CurrState2 = 0; // Set Current State to be in initial state
    Light1('R'); // Turn 1st set of traffic lights to be red
    Time1A_delay(2000); // Set delay 2 seconds
  } else if (CurrState2 == 2) {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7); // Turn on 2nd GREEN LED
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); // Turn on 1st RED LED
    CurrState2 = 0; // Set Current State to be in initial state
    Light2('R'); // Turn 2nd set of traffic lights to be RED
    Time1A_delay(2000); // Delay for 2 seconds
    
// ----------------- BONUS: HANDLE IF 2 BUTTONS PRESSED -----------------------
    
  } else if (CurrState2 == 3) {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7); // Turn on 2nd GREEN LED
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // Turn on 1st GREEN LED
    Light1('R'); // Set 1st set of Traffic Lights to be RED
    Light2('R'); // Set 2nd set of Traffic Lights to be RED
    CurrState2 = 0; // Set Current State to be in initial state
    Time1A_delay(2000); // Delay for 2 seconds
  }

}

// ------------------------ GPIO PORTF Handler ------------------------------

void PortFHandler()
{
  TimerDisable(TIMER0_BASE, TIMER_A); // Disable Timer
  GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4); // Clear interrupt flag
  TimerEnable(TIMER1_BASE, TIMER_A); // Enable Timer
  if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)){  // If 1st button is not pressed
    if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)){ // If 2nd button is not pressed
      CurrState2 = 3; // Set current state for pedestrian traffic lights to be 3 / All pedestrian traffic red
    } else{
    CurrState2 = 2; // Set current state for pedestrian traffic to be 2
    }
  }
  else if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)){ // if 2nd button is not pressed
   if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)){ // if 1st button is not pressed
      CurrState2 = 3; // Set current state for pedestrian traffic lights to be 3 / All pedestrian traffic red
   }else{
     CurrState2 = 1; // Set current state for pedestrian traffic lights to be 1
   }
  }
  Time1A_delay(1); // Delay for 10 seconds to start interrupts on timer 1A
}

// ------------------------ TIMER INITIALIZATIONS ------------------------------

// ------------------------ TIMER 0 INITIALIZATIONS ------------------------------

void Timer0INIT(){
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set System Clock
   SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable the timer 0 peripheral
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){} // Check that timer 0 peripheral is enabled
   TimerDisable(TIMER0_BASE, TIMER_A); // Configuration Sandwich / Disable Timer
   TimerIntDisable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); // Disable Interrupts
   TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Set timer 0 to be periodic
   TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT); // Clear Interrupts on timer 0
   IntPrioritySet(INT_TIMER0A, 0x02); // Set priority of the timer's interrupt to be 2nd
   TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0AHandler); // Register the interrupt handler
   TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); // Enable Timer
}

// ------------------------ TIMER 1 INITIALIZATIONS ------------------------------

void Timer1INIT(){
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set System Clock
   SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable the timer 1 peripheral
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)){} // Check that timer 1 peripheral is enabled
   TimerDisable(TIMER1_BASE, TIMER_A);  // Configuration Sandwich / Disable Timer
   TimerIntDisable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); // Disable Interrupts
   TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Set timer 1 to be periodic
   TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT); // Clear Interrupts on timer 0
   IntPrioritySet(INT_TIMER1A, 0x01); // Set priority of the timer's interrupt to be 2nd
   TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1AHandler); // Register the interrupt handler
   TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); // Enable Timer
}



// ------------------------ GPIO PORTS INITIALIZATIONS ------------------------

// ------------------------ GPIO PORT A INITIALIZATION ------------------------

void PortAINIT(){
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable the GPIOA peripheral
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){} // Check that its enabled
   HWREG(GPIO_PORTA_BASE+GPIO_O_LOCK)= GPIO_LOCK_KEY; // Enable Lock Register
   HWREG(GPIO_PORTA_BASE+GPIO_O_CR)= GPIO_PIN_0; // Enable Commit Register
   GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); // Set Pin 5,6,7 to be output
}

// ------------------------ GPIO PORT E INITIALIZATION ------------------------

void PortEINIT(){
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable the GPIOE peripheral
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){} // Check that its enabled
   HWREG(GPIO_PORTE_BASE+GPIO_O_LOCK)= GPIO_LOCK_KEY; // Enable Lock Register
   HWREG(GPIO_PORTE_BASE+GPIO_O_CR)= GPIO_PIN_0; // Enable Commit Register
   GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3); // Set Pin 1,2,3 to be output
}

// ------------------------ GPIO PORT C INITIALIZATION ------------------------

void PortCINIT(){
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable the GPIOC
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){} // Check that its enabled
   HWREG(GPIO_PORTC_BASE+GPIO_O_LOCK)= GPIO_LOCK_KEY; // Enable Lock Register
   HWREG(GPIO_PORTC_BASE+GPIO_O_CR)= GPIO_PIN_0; // Enable Commit Register
   GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); // Set Pin 4,5,6,7 to be output
   GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_4 | GPIO_PIN_6); // Turn on RED lights by default
}

// ------------------------ GPIO PORT F INITIALIZATION ------------------------

void PortFINIT(){
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable the GPIOF
   while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){} // Check that its enabled
   HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK)= GPIO_LOCK_KEY; // Enable Lock Register
   HWREG(GPIO_PORTF_BASE+GPIO_O_CR)= GPIO_PIN_0; // Enable Commit Register
   GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // Set Pin 0 and Pin 4 to be inputs because we used them as the pedestrian switches
   GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Set the pull up register for the buttons
   GPIOIntDisable(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4); // Disable Interrupts
   GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4); // Clear Interrupts
   IntPrioritySet(INT_GPIOF , 0x00); // Set PortF handle to be of highest priority
   GPIOIntRegister(GPIO_PORTF_BASE,PortFHandler); // Register the interrupt handler
   GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4,GPIO_FALLING_EDGE); // Set the interrupt type to be on the falling edge of the clock
   GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // Enable Interrupts
}


// ------------------------------- MAIN FUNCTION ----------------------------
int main()
{
    //Call initializations
    Timer0INIT(); 
    Timer1INIT();
    PortAINIT();
    PortEINIT();
    PortCINIT();
    PortFINIT();

    Time0A_delay(100); // call delay to start interrupts
    
    while(1){
    __asm("wfi\n");
    };

    
   return 0;
}