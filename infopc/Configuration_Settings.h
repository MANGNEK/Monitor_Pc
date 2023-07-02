#define  LOLIN_D32
//#define  LOLIN32_LITE

//--------------------------- Bluetooth Classic -----------------------------------
/*ESP32 Communication type, Uncomment only one option!!!*/

/*Uncomment to enable BT, else default to USB serial only,
  Baud Rate of PhatStats and HardwareSerialMonitor must match*/

#define enable_BT              // Use baud of 9600, enable only Bluetooth Classic serial connection

/* Enable the built in LED blinking when transmitting, saves power when using battery if disabled,*/
#define enableTX_LED //
int TX_LED_Delay = 0; // TX blink delay


//----------------------------- Battery Monitor ------------------------------------
/*WARNING!!! Requires voltage divider (GND ---[100K]--- (Pin34 ADC) ----[100k]--- BATT+) (0%)3.2v to (100%)4.2v Range,*/

//#define batteryMonitor // (experimental) Read current LiPo battery level if connected.

//-------------------------------- DISCLAIMER -------------------------------------------
/*
  !!!THE WEMOS LOLIN32, NOT LIMITED TO, APPEARS TO HAVE NO "UNDERVOLTAGE PROTECTION"
  OR "OVER DISCHARGE PROTECTION" ON THE CHARGING CIRCUIT!!!

  OTHER BOARDS ARE THE SAME!!! USE A LiPo BATTERY WITH BUILT IN PROTECTION, EVEN THEN,
  BUILT IN PROTECTION IS CONSIDERED A LAST RESORT SAFETY NET OR "BELTS AND BRACERS" APPROACH.

  YOU MAY GET SOME BENEFITS, OVERVOLTAGE,OVERCURRENT AND SHORT CIRCUIT PROTECTION BUT, USUALLY
  THE OVER DISCHARGE PROTECTION CUT OFF VOLTAGE IS AROUND 2.4v WHICH IS WAY TOO LOW FOR THE
  CONTINUED MAINTAINED HEALTH OF THE BATTERY.

  RECOMMENDED OVER DISCHARGE PROTECTION VOLTAGES FOR LiPo's ARE AROUND 2.9 - 3 VOLTS.

  ALTERNATIVELY USE A BATTERY BANK THROUGH THE USB CONNECTOR

  !!!LITHIUM POLYMER PACKS / BATTERIES CAN BE VERY DANGEROUS, WITH A RISK OF FIRE!!!

  If you are going to use a battery or LiPo pack you must take some responsibility, do your research!!!.
  No advice will be given, or implied regarding which you should use etc.

  Use the battery/type in accordance with the manufacturer's recommendations.*/


//--------------------------- Uncomment your CPU/GPU Display  -----------------------------------

/* Uncomment your CPU,*/

#define INTEL_CPU
//#define AMD_CPU


/* Uncomment your GPU,*/

#define NVIDIA_GRAPHICS
//#define NVIDIA_GTX_Ti_GRAPHICS
//#define NVIDIA_RTX_GRAPHICS
//#define NVIDIA_RTX_SUPER_GRAPHICS

//#define AMD_GRAPHICS
//#define INTEL_GRAPHICS

/* Characters to delete from the start of the CPU/GPU name eg: Remove "Intel" or "Nvidia" to save space*/
#define cpuNameStartLength 10  // i5-9600k = 10 / i9-13900k = 19
#define gpuNameStartLength 18

//>>>>>>>>>>>>>>>>>>>>>>>>>>>

/* Manually name the  CPU,*/
//#define Manual_cpuName
String set_CPUname = "i9-12900K";

/* Manually name the GPU,*/
//#define Manual_gpuName
String set_GPUname = "Geforce RTX 3080";

/* Manually set GPU ram total,*/
//#define Manual_gpuRam
String set_GPUram = "256";

//>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define noDegree      // lose the "o"
#define smallPercent  // Use small percent symbol
//---------------------------------------------------------------------------------------

/* CPU is overclocked with Turbo boost disabled, to stop "TURBO" indicator,*/
//#define CPU_OverClocked

/* CPU & GPU Thermal Junction Max Temperature in "c" before throttling,*/
#define CPU_TJMAX 100  //  TJ Max for the Intel 9900K    = 100c
#define GPU_TJMAX 83   //  TJ Max for the Nvidia GTX1080 = 83c

/* CPU & GPU Turbo/Boost Frequency Values in Mhz */
#define CPU_BOOST 1600  //  Enter Stock CPU Frequency eg. Intel Core i5 9600k = 3700MHz
#define GPU_BOOST 1110  //  Enter Stock GPU Frequency eg. MSi GamingX 1080  = 1683MHz

//-------------------------------- Show GPU Stats ------------------------------------

/* Remove Specific GPU stats in landscape mode(enable_gpuVram, enable_gpuShader),
  this helps to make room if your GPU memory is over 9999MB */

//#define enable_gpuVram
//#define enable_gpuShader
#define enable_BigGPUw      // undefine  enable_gpuShader & enable_gpuShader first

//-----------------------------

#define enable_gpuCore
#define enable_gpuPowerStats    // Nvidia Specific???
#define enable_gpuFanStatsPerc  // this is only displayed in Portrait!!!
#define enable_gpuFanStatsRPM

//----------------------------- Frequency Gains Indicator --------------------------------

/* Uncomment to enable the display of frequency gains */
#define enable_ShowFrequencyGain

/* Uncomment only one of the units to display below, MHz or Percent */
#define ShowFrequencyGainMHz        // Show Overlock/Turbo & Boost Clock Frequency Gains in MHZ  eg: "+24MHz"
//#define ShowFrequencyGainPerc       // Show Overlock/Turbo & Boost Clock Frequency Gains in Percent  eg: "+24%"

//----------------------------- Throttle/Boost Indicator --------------------------------

#define enable_ThrottleIndicator // Show TJMax Indicator 
#define enable_BoostIndicator    // Show CPU & GPU Turbo/Boost Indicator

//-------------------------------- NeoPixel Modes -------------------------------------
#define NUM_PIXELS  8
/* If  NeoBrightness = 0 Phat-Stats will start with no NeoPixels lit. Turn the Rotary Encoder to turn on the NeoPixels, */
int NeoBrightness   = 250;           // Global start up brightness

/* Uncomment only one of the below*/
//#define enable_NeopixelGauges     // NeoPixel ring bargraph example

//#define enable_Thresholdtriggers_PCB // New PCB 4x NeoPixel Rotate States,0,1,2,3  Trigger functions when CPU or GPU thresholds are met



//----------------------------- TFT Fixed or PWM Brightness ------------------------------------
//#define fixedBacklight // enable a fixed backlight (no PWM) powered from VCC

/*TFT Start Up Brightness*/
volatile int brightness_count = 250; // Start Up PWM Brightness

//-------------------------- Display Activity Shutdown -----------------------------------

/* Uncomment below to turn off the screen on serial timeout, else keep last display info eg: incase of PC Crash*/
//#define enableActivityChecker

/* How long the display takes to timeout due to inactive serial data from the windows application */
#define lastActiveDelay 8000

//-------------------------------- Misco Setting -----------------------------------------

//#define  RETRO_MONO    //CRT Monochrome screen
//#define  RETRO_AMBER   //CRT Amber screen
//#define  RETRO_GREEN   //CRT Green screen

#define splashScreenLS // quick splash screen landscape hack job, also in FeatureSet

/* Debounce  Button, button mode is a bit flaky at present as it needs interrupts, Sometimes it gets caught during a screen refresh
  and does not change. WIO Terminal & ESP32 seem to like 1000ms and works just!!! ok */

int debounceButton = 0; //  Use a 0.1uf/100nf/(104) p capacitor from button Pin to GND if poss

/* Delay screen event, to stop screen data corruption ESP8622 / ESP32 use 25, most others 5 or 0 will do*/
int Serial_eventDelay = 15;  // 15 is the minimum setting for an ESP32 with a Silicon Labs CP210x serial chip

int baudRate  = 9600; // set serial baud rate to match that of HardwareSerialMonitor 115200 will use more resources



// BT Board ID


#define device_BT "PC Hardware Monitor 1"


//-------------------------------------------- Versions ---------------------------------- -
/*

     V3.1.1.ADV    Advanced

        Button to change aspect
        NeoPixels

        Add enable_gpuVram, enable_gpuShader, enable_gpuCore
        Remove specific GPU stats in landscape mode, this helps to make room if your GPU memory is over 9999MB
        Fix set_GPUram

     V3.1.3.BT.ADV   (ESP32)

        Fixes
        Retro CRT look option


    V3.1.6.ADV/KiSS

       Clean up code
       Turn off the backlight untill the stats have updated when using button mode.
       Add PCB_ThresholdTriggerEvents for custom PCB, so neopixels rotate with button mode.

      (ADV) Add option to display the splash screen in landscape.

  Note: Gnat-Stats/Phat-Stats is optimised for desktop CPU's with dedicated graphics cards, such as Nvidia/Radeon.
      You may get wierd results on mobile CPUs and integrated GPU's (iGPU's) on laptops.

*/







//---------------------------- Debug Screen Erasers ---------------------------------------

/* f Screen, Update Erasers, */
//#define Debug

//-------------- Show Networks Stats when using Phatstats edition of WeeStatServer ---------------
#define enable_LibreNet // Reserved // undefine  enable_gpuCore, enable_gpuShader, enable_gpuShader first, enable_gpuPowerStats
//--------------------------- Throttle/Boost Gains MHZ or % ------------------------------
