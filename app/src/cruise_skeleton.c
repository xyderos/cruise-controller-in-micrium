#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

#define DE0_NANO 1
#define DE2_115 2

#define BOARD_TYPE DE2_115

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/* ANSI Color codes */
#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_MAGENTA "\033[1;35m"
#define COLOR_CYAN "\033[1;36m"
#define COLOR_RESET "\033[0m"

void set_color(char* color_code)
{
    printf(color_code);
}

void reset_color()
{
    printf(COLOR_RESET); // reset color
}
/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 1536 

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadDetectionTask_Stack[TASK_STACKSIZE];
OS_STK ExtraLoadTask_Stack[TASK_STACKSIZE];

// Task Priorities

#define WATCHDOGTASK_PRIO 4 
#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define BUTTONIOTASK_PRIO  13
#define SWITCHIOTASK_PRIO  14
#define EXTRALOADTASK_PRIO 16
#define OVERLOADDETECTIONTASK_PRIO 18 

// Task Periods

#define WATCHDOG_PERIOD 300
#define OVERLOAD_PERIOD 300
#define CONTROL_PERIOD  300
#define BUTTON_IO_PERIOD 100
#define SWITCH_IO_PERIOD 300
#define EXTRALOADTASK_PERIOD 300
#define VEHICLE_PERIOD  300
/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine;
OS_EVENT *Mbox_GasPedal;
OS_EVENT *Mbox_CruiseControl;
OS_EVENT *Mbox_Brake1;
OS_EVENT *Mbox_WatchdogReset;

// Semaphores
OS_EVENT *vehicleTaskSemaphore;
OS_EVENT *controlTaskSemaphore;
OS_EVENT *buttonIOTaskSemaphore;
OS_EVENT *switchIOTaskSemaphore;
OS_EVENT *watchdogTaskSemaphore;
OS_EVENT *overloadDetectionSemaphore;
OS_EVENT *extraOverloadSemaphore;
OS_EVENT *overloadSemaphore;

// SW-Timer
OS_TMR *vehicleTaskTimer;
OS_TMR *controlTaskTimer;
OS_TMR *buttonIOTaskTimer;
OS_TMR *switchIOTaskTimer;
OS_TMR *watchdogTimer;
OS_TMR *extraOverloadTimer;
OS_TMR *overloadTimer;
/*
 * Types
 */
enum active {on = 2, off = 1};

enum status {ok, overloaded};

enum status cond=ok;

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs

int extraLoadValue = 0;

/*
 * Helper functions
 */

int buttons_pressed(void){
#if BOARD_TYPE == DE2_115
    return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);
#elif BOARD_TYPE == DE0_NANO
    return 0;
#endif
}

int switches_pressed(void){
#if BOARD_TYPE == DE2_115
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);
#elif BOARD_TYPE == DE0_NANO
    return 0;
#endif
}

void writeGreenLeds(){
#if BOARD_TYPE == DE2_115
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
#elif BOARD_TYPE == DE0_NANO
    IOWR_ALTERA_AVALON_PIO_DATA(PIO_LED_BASE, (INT8U)led_green);
#endif
}

void writeRedLeds(){
#if BOARD_TYPE == DE2_115
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
#elif BOARD_TYPE == DE0_NANO
    // do nothing (DE0 Nano has no red leds..)
    // TODO: Find another solution, possibly using GPIO + external LEDs
#endif
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context){
    OSTmrSignal(); /* Signals a 'tick' to the SW timers */

    return delay;
}

static int b2sLUT[] = {
    0x40, //0
    0x79, //1
    0x24, //2
    0x30, //3
    0x19, //4
    0x12, //5
    0x02, //6
    0x78, //7
    0x00, //8
    0x18, //9
    0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
    return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
#if BOARD_TYPE == DE2_115
void show_velocity_on_sevenseg(INT8S velocity){
    int tmp = velocity;
    int out;
    INT8U out_high = 0;
    INT8U out_low = 0;
    INT8U out_sign = 0;

    if(velocity < 0){
        out_sign = int2seven(10);
        tmp *= -1;
    }else out_sign = int2seven(0);

    out_high = int2seven(tmp / 10);
    out_low = int2seven(tmp - (tmp/10) * 10);

    out = int2seven(0) << 21 |
        out_sign << 14 |
        out_high << 7  |
        out_low;
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}
#endif

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position){

  if( 0 < position && position < 400) led_red = led_red | 0x20000;
  else if (400 <= position && position < 800)led_red = led_red | 0x10000;
  else if (800 <= position && position < 1200)  led_red = led_red | 0x8000;
  else if (1200 <= position && position < 1600) led_red = led_red | 0x4000;
  else if (1600 <= position && position < 2000)led_red = led_red |0x2000; 
  else if (2000 <= position) led_red = led_red | 0x1000;

}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 *
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void* pdata) {

  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;
  void* msg;
  INT8U* throttle;
  INT16S acceleration;
  INT16U position = 0;
  INT16S velocity = 0;
  enum active brake_pedal = off;
  enum active engine = off;
  printf("Vehicle task created!\n");

  while(1){

    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

    OSTmrStart(vehicleTaskTimer, &err);

    // wait until vehicleTaskSemaphore is 1
    OSSemPend(vehicleTaskSemaphore, 0, &err);

    /* Non-blocking read of mailbox:
       - message in mailbox: update throttle
       - no message:         use old throttle
    */
    msg = OSMboxPend(Mbox_Throttle, 1, &err);
    if (err == OS_NO_ERR)throttle = (INT8U*) msg;
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err);
    if (err == OS_NO_ERR)brake_pedal = (enum active) msg;
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err);
    if (err == OS_NO_ERR) engine = (enum active) msg;

    printf(" value received is: %d \n", (unsigned int)msg );

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off){
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)  acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800) acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else acceleration = - brake_factor*velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);
    show_position(position);


    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)position = 0;

#if BOARD_TYPE == DE2_115
    show_velocity_on_sevenseg((INT8S) velocity);
#elif BOARD_TYPE == DE0_NANO
    // no sevenseg on DE0_NANO
    // TODO: Find some other way to display this
#endif
  }
}

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata){
  INT8U err;
  INT8U throttle = 40; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;
  INT16S cruiseVelocity;
  int delta = 0;
  enum active brake_pedal = off;
  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 
  printf("Control Task created!\n");

    while (1){
      // get current velocity (sent by VehicleTask)
      msg = OSMboxPend(Mbox_Velocity, 0, &err);
      current_velocity = (INT16S*) msg;


      // check if gas pedal is pressed (sent by ButtonIO task)
      msg = OSMboxPend(Mbox_GasPedal,1,&err);
      gas_pedal = (enum active) msg;
      // check if brake pedal is pressed (sent by ButtonIO task)
      msg = OSMboxPend(Mbox_Brake1,1,&err);
      brake_pedal = (enum active) msg;
      // check if the cruise control button is pressed (sent by ButtonIO task)
      msg = OSMboxPend(Mbox_CruiseControl,1,&err);
      if ((enum active) msg == on) {

        printf("\n[CRUISE CONTROL] Cruise Control button press event received\n");
        if (cruise_control == on) {
          printf("\n[CRUISE CONTROL] Turning cruise control OFF.\n\n");
          cruise_control = off;
        } else { // (brake_pedal== off && gas_pedal == off && *current_velocity >=20)
          printf("\n[CRUISE CONTROL] Turning cruise control ON.\n\n");

          cruise_control = on;
          cruiseVelocity = *current_velocity;

          printf("\n\n[CRUISE CONTROL] Cruise control enabled, cruise velocity set to : %d \n\n", *current_velocity);
        }

        if (cruise_control == off) {
          if (gas_pedal == on) throttle++;
          else throttle--;
        } else {
          if (*current_velocity >= 25) {
            delta = *current_velocity - cruiseVelocity;
            printf(">> Cruise control delta is: %d (current: %d vs cruise: %d)\n", delta, *current_velocity, cruiseVelocity);

            if (delta <= -4) throttle++;
            else if(delta >= 4) throttle--;
          }
        }
      }
      else{

        if(gas_pedal == on)throttle++;
        else throttle--;


        if(brake_pedal == on) throttle =0;

      }
      err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
      OSTmrStart(controlTaskTimer, &err);

      // wait until controlTaskSemaphore is signaled (value = 1)
      OSSemPend(controlTaskSemaphore, 0, &err);
    }
}

void buttonIOTaskTimerCallback(void *ptmr, void *callback_arg){
    OSSemPost(buttonIOTaskSemaphore);
}

void switchIOTaskTimerCallback(void *ptmr, void *callback_arg){
    OSSemPost(switchIOTaskSemaphore);
}

void controlTaskTimerCallback(void *ptmr, void *callback_arg){
    OSSemPost(controlTaskSemaphore);
}

void extraLoadTaskTimerCallback(void *ptmr, void *callback_arg){
  OSSemPost(extraOverloadSemaphore);
}

void vehicleTaskTimerCallback(void *ptmr, void *callback_arg){
    OSSemPost(vehicleTaskSemaphore);
}

void overloadTimerCallback(void *ptmr, void *callback_arg){
  OSSemPost(overloadSemaphore);
}

void ButtonIOTask(void* pdata){
  INT8U err;
  printf("Button IO task created!\n");
  int buttons;
  while (1){
      buttons = buttons_pressed();
      led_green = 0;

      if (buttons & CRUISE_CONTROL_FLAG) {
        printf("[BUTTON IO TASK] Cruise control pressed.\n");
        led_green = led_green | LED_GREEN_2;
        OSMboxPost(Mbox_CruiseControl, on);
      }
      else OSMboxPost(Mbox_CruiseControl, off);

      if (buttons & BRAKE_PEDAL_FLAG) {
        printf("[BUTTON IO TASK] Brake pedal pressed.\n");
        led_green = led_green | LED_GREEN_4;
        OSMboxPost(Mbox_Brake, on);
        OSMboxPost(Mbox_Brake1, on);
      }
      else {
        OSMboxPost(Mbox_Brake, off);
        OSMboxPost(Mbox_Brake1, off);
      }

      if (buttons & GAS_PEDAL_FLAG) {
        printf("[BUTTON IO TASK] Gas pedal pressed.\n");
        led_green = led_green | LED_GREEN_6;
        OSMboxPost(Mbox_GasPedal, on);
      }
      else OSMboxPost(Mbox_GasPedal, off);

      writeGreenLeds();
      OSTmrStart(buttonIOTaskTimer, &err);

      // wait until controlTaskSemaphore is signaled (value = 1)
      OSSemPend(buttonIOTaskSemaphore, 0, &err);
  }
}

void SwitchIOTask(void* pdata){
  INT8U err;
  int switches;
  printf("Switches IO task created!\n");

  while (1){
    switches = switches_pressed();

    led_red &= 0xFFFFFFFC;

    if (switches & ENGINE_FLAG) {
      printf("[SWITCH IO TASK] Engine switch on.\n");
      led_red = led_red | LED_RED_0;
      OSMboxPost(Mbox_Engine, (void*)on);
    }
    if (switches & TOP_GEAR_FLAG) {
      printf("[SWITCH IO TASK] Top gear switch on.\n");
      led_red = led_red | LED_RED_1;
    }
    switches  = ((switches >> 4 ) &  0x3F);
    if(switches > 50) extraLoadValue= 50;
    else extraLoadValue = switches;
    writeRedLeds();
    OSTmrStart(switchIOTaskTimer, &err);
    // wait until controlTaskSemaphore is signaled (value = 1)
    OSSemPend(switchIOTaskSemaphore, 0, &err);
  }
}

void watchdogTimerCallback(void *ptmr, void *callback_arg){
    OSSemPost(watchdogTaskSemaphore);
}

void WatchdogTask(void* pdata){
  // TODO
  // if the value is 0, the overload detection task has not been able to
  // post to the semaphore (increase it's value) in time, meaning there is an overload
  // condition going on
  static OS_SEM_DATA semData;
  INT8U err;
  while (1) {
    OSSemPend(watchdogTaskSemaphore, 0, &err);
    set_color(COLOR_YELLOW);
    err = OSSemQuery(overloadDetectionSemaphore, &semData);
    if (err == OS_NO_ERR) {

      if (semData.OSCnt < 1) {
        printf("[WATCHDOG TASK] No overload detected (QSQSize = %d).\n", semData.OSCnt);
        OSSemPost(overloadDetectionSemaphore);
      }
      else printf("[WATCHDOG TASK] OVERLOAD DETECTED (QSQSize = %d).\n", semData.OSCnt);
    }
    else
        printf("[WATCHDOG TASK] WATCHDOG TIMER CALLBACK ERROR CODE: %d\n", err);
      reset_color();
  }
}

void OverloadDetectionTask(void* pdata){
  INT8U  err;
  printf("Overload detection task initialized...\n");
  while (1) {
    set_color(COLOR_RED);
    printf("[OVERLOAD DETECTION TASK] Posting overload detection semaphore..\n");
    reset_color();

    OSTmrStart(overloadTimer, &err);
    OSSemPend(overloadDetectionSemaphore, 0, &err);
  }
}

void ExtraLoadTask(void* pdata){

  INT8U err;
  for(;;){

    set_color(COLOR_BLUE);
    printf("[EXTRA LOAD TASK STARTED]\n");
    // add extra load
    unsigned int max = 40000;
    unsigned int x = 0;

    while((x++) <extraLoadValue *10000);

    printf("[EXTRA LOAD TASK DONE]\n");
    reset_color();
    OSTmrStart(extraOverloadTimer, &err);
    OSSemPend(extraOverloadSemaphore, 0, &err);
  }
}

void printCreateTaskError(INT8U err) {
  switch (err) {
  case OS_NO_ERR:
    printf("OS_NO_ERR: If the function was successful.\n");
    break;
  case OS_PRIO_INVALID:
    printf("OS_PRIO_INVALID: If prio is higher than OS_LOWEST_PRIO .\n");
    break;
  case OS_NO_MORE_TCB:
    printf("OS_NO_MORE_TCB: If µC/OS-II doesn’t have any more OS_TCBs to assign.\n");
    break;
  default:
    printf("(Unknown error code.)\n");
  }
}
/*
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */

void StartTask(void* pdata){
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000;
  printf("delay in ticks %d\n", delay);

    /*
     * Create Hardware Timer with a period of 'delay'
     */
    if (alt_alarm_start (&alarm,delay, alarm_handler, dcontext) < 0) printf("No system clock available!n");

    /* 
     * Create and start Software Timer 
     */

    extraOverloadTimer = OSTmrCreate(
                                   EXTRALOADTASK_PERIOD / HW_TIMER_PERIOD,   // dly
                                   0,                                  // period
                                   OS_TMR_OPT_ONE_SHOT,                // opt
                                   extraLoadTaskTimerCallback,           // callback
                                   (void *)0,                          // *callback_arg
                                   (INT8U*) "extra oveload task task timer",      // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
                                   &err                                // *perr
                                   );


    overloadTimer = OSTmrCreate(
                                     OVERLOAD_PERIOD / HW_TIMER_PERIOD,   // dly
                                     0,                                  // period
                                     OS_TMR_OPT_ONE_SHOT,                // opt
                                     overloadTimerCallback,           // callback
                                     (void *)0,                          // *callback_arg
                                     (INT8U*) "extra oveload task task timer",      // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
                                     &err                                // *perr
                                     );

    vehicleTaskTimer = OSTmrCreate(
            VEHICLE_PERIOD / HW_TIMER_PERIOD,   // dly
            0,                                  // period
            OS_TMR_OPT_ONE_SHOT,                // opt
            vehicleTaskTimerCallback,           // callback
            (void *)0,                          // *callback_arg
            (INT8U*) "Vehicle task timer",      // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
            &err                                // *perr
            );


    buttonIOTaskTimer = OSTmrCreate(
            BUTTON_IO_PERIOD / HW_TIMER_PERIOD,                              // dly
            0,                              // period
            OS_TMR_OPT_ONE_SHOT,            // opt
            buttonIOTaskTimerCallback,       // callback
            (void *)0,                      // *callback_arg
            (INT8U*) "Button IO task timer",        // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
            &err                            // *perr
            );

    switchIOTaskTimer = OSTmrCreate(
            SWITCH_IO_PERIOD / HW_TIMER_PERIOD, // dly
            0,                                  // period
            OS_TMR_OPT_ONE_SHOT,                // opt
            switchIOTaskTimerCallback,          // callback
            (void *)0,                          // *callback_arg
            (INT8U*) "Switch IO task timer",    // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
            &err                            // *perr
            );


    controlTaskTimer = OSTmrCreate(
            CONTROL_PERIOD / HW_TIMER_PERIOD, // dly
            0,                              // period
            OS_TMR_OPT_ONE_SHOT,            // opt
            controlTaskTimerCallback,       // callback
            (void *)0,                      // *callback_arg
            (INT8U*) "Control task timer",        // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
            &err                            // *perr
            );


    watchdogTimer = OSTmrCreate(
            0,                              // dly
            WATCHDOG_PERIOD / HW_TIMER_PERIOD,  // period
            OS_TMR_OPT_PERIODIC,            // opt
            watchdogTimerCallback,          // callback
            (void *)0,                      // *callback_arg
            (INT8U*)"Watchdog timer",        // *pname, compiler gives an warning without cast (see UCOSII/src/os_tmr.c for definition)
            &err                            // *perr
            );

    OSTmrStart(watchdogTimer, &err);


    /*
     * Creation of Kernel Objects
     */

    vehicleTaskSemaphore = OSSemCreate(0);
    overloadDetectionSemaphore = OSSemCreate(1);
    watchdogTaskSemaphore = OSSemCreate(0);
    buttonIOTaskSemaphore = OSSemCreate(0);
    switchIOTaskSemaphore = OSSemCreate(0);
    extraOverloadSemaphore= OSSemCreate(0);
    controlTaskSemaphore = OSSemCreate(0);
    overloadSemaphore = OSSemCreate(0);

    // Mailboxes
    Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
    Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
    Mbox_Brake = OSMboxCreate((void*) 1); /* Empty Mailbox - Brake */
    Mbox_Engine = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine */
    Mbox_GasPedal = OSMboxCreate((void*) 1); /* Empty Mailbox - Gas Pedal */
    Mbox_CruiseControl = OSMboxCreate((void*) 0); /* Empty Mailbox - Cruise control */
    Mbox_WatchdogReset = OSMboxCreate((void*) 0); /* Empty Mailbox - Watchdog reset signal */


    /*
     * Create statistics task
     */

    OSStatInit();

    /* 
     * Creating Tasks in the system 
     */


    err = OSTaskCreateExt(
            ControlTask, // Pointer to task code
            NULL,        // Pointer to argument that is
            // passed to task
            &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
            // of task stack
            CONTROLTASK_PRIO,
            CONTROLTASK_PRIO,
            (void *)&ControlTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,
            OS_TASK_OPT_STK_CHK);
    if (err == OS_NO_ERR) {
        printf("ControlTask created successfully.\n");
    } else {
        printf("ControlTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }

    err = OSTaskCreateExt(
            VehicleTask, // Pointer to task code
            NULL,        // Pointer to argument that is
            // passed to task
            &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
            // of task stack
            VEHICLETASK_PRIO,
            VEHICLETASK_PRIO,
            (void *)&VehicleTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,
            OS_TASK_OPT_STK_CHK);

    if (err == OS_NO_ERR) {
        printf("VehicleTask created successfully.\n");
    } else {
        printf("VehicleTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }


    err = OSTaskCreateExt(
            ButtonIOTask, // Pointer to task code
            NULL,      // Pointer to argument that is
            // passed to task
            (void *)&ButtonIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
            // of task stack 
            BUTTONIOTASK_PRIO,
            BUTTONIOTASK_PRIO,
            (void *)&ButtonIOTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,  
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    if (err == OS_NO_ERR) {
        printf("ButtonIOTask created successfully.\n");
    } else {
        printf("ButtonIOTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }
    err = OSTaskCreateExt(
            SwitchIOTask,       // Pointer to task code
            NULL,               // Pointer to argument that is
            // passed to task
            (void *)&SwitchIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
            // of task stack 
            SWITCHIOTASK_PRIO,
            SWITCHIOTASK_PRIO,
            (void *)&SwitchIOTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,  
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    if (err == OS_NO_ERR) {
        printf("SwitchIOTask created successfully.\n");
    } else {
        printf("SwitchIOTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }

    err = OSTaskCreateExt(
            WatchdogTask,                                  // Pointer to task code
            NULL,      										// Pointer to argument that is passed to task
            (void*)&WatchdogTask_Stack[TASK_STACKSIZE-1], 	// Pointer to top of task stack 
            WATCHDOGTASK_PRIO,
            WATCHDOGTASK_PRIO,
            (void*)&WatchdogTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,  
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    if (err == OS_NO_ERR) {
        printf("WatchdogTask created successfully.\n");
    } else {
        printf("WatchdogTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }

    err = OSTaskCreateExt(
            OverloadDetectionTask,                          // Pointer to task code
            NULL,      										// Pointer to argument that is passed to task
            (void *)&OverloadDetectionTask_Stack[TASK_STACKSIZE-1], 	// Pointer to top of task stack 
            OVERLOADDETECTIONTASK_PRIO,
            OVERLOADDETECTIONTASK_PRIO,
            (void *)&OverloadDetectionTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,  
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    if (err == OS_NO_ERR) {
        printf("OverloadDetectionTask created successfully.\n");
    } else {
        printf("OverloadDetectionTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }

    err = OSTaskCreateExt(
            ExtraLoadTask,                                  // Pointer to task code
            NULL,      										// Pointer to argument that is passed to task
            (void*)&ExtraLoadTask_Stack[TASK_STACKSIZE-1], 	// Pointer to top of task stack 
            EXTRALOADTASK_PRIO,
            EXTRALOADTASK_PRIO,
            (void*)&ExtraLoadTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,  
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    if (err == OS_NO_ERR) {
        printf("ExtraLoadTask created successfully.\n");
    } else {
        printf("ExtraLoadTask creation failed with error %d: ", err);
        printCreateTaskError(err);
    }

    printf("All Tasks and Kernel Objects generated!\n");

    /* Task deletes itself */

    OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

    printf("Lab: Cruise Control\n");

    OSTaskCreateExt(
            StartTask, // Pointer to task code
            NULL,      // Pointer to argument that is
            // passed to task
            (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
            // of task stack 
            STARTTASK_PRIO,
            STARTTASK_PRIO,
            (void *)&StartTask_Stack[0],
            TASK_STACKSIZE,
            (void *) 0,  
            OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSStart();

    return 0;
}
