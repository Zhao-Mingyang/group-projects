/*
 * SysTick interrupt, in whose handler ADC shall be managed
 *
 * ADC Input:   Pin 5.0
 * ADC Output:  Port 2, Pin 0 to 7
 * Loop Frequency Detection:    Pin 6.0
 * Master Clock Verification:   Pin 4.3
 *
 * Master Clock Frequency:  48 MHz
 * SysTick Frequency:   50 KHz, i.e. 960 times of MSCL
 * ADC Resolution:  10 bits
 * ADC Memory:  ADC_MEM0
 */
/* Header File Declaration */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define PTS 50      //amount of points in seven cycles of a sine wave
#define PI 3.14159

/*************************** Variable Declaration **********************************/
static volatile float cos_value[PTS];
static volatile int cos_counter = 0;

static float num_1[3] = {                1,   -1.077091808207,                 1};
static float den_1[3] = {                1,  -0.9783996467977,   0.8167432698727};
static float num_gain_1 = 0.9083716349363;
static float den_gain_1 = 1;
/*
static float num_2[3] = {                1,   -1.077091808207,                 1};
static float den_2[3] = {                1,   -1.116433460126,   0.8731363010639};
static float num_gain_2 = 0.931360297695;
static float den_gain_2 = 1;
*/
static float buffer_input[3] = {0,0,0};
static float buffer_stage1_output[3] = {0,0,0};
//static float buffer_stage2_output[3] = {0,0,0};

static float signal = 0;
static float signal_inputRegulated = 0;
static float signal_bandstopped = 0;
static float signal_multiplexed = 0;
static float signal_outputRegulated = 0;
/************************************ Initial Function *******************************/
void initialCosineWave(void) {
    int i = 0;
    for (i=0; i<PTS; i++) {
        cos_value[i] = cos(2.0 * PI * 7.0f * i /(float)PTS);
    }
}

void initialPin(void) {
    /* Pin6.0 loop period detected pin */
    P6DIR |= BIT0;
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0);
    /* Output Port 2 Pin set up  */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);

}

void initialClock(void) {
    /* HFXT crystal */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Clock pin4.3 detection */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setExternalClockSourceFrequency(32000,48000000);
    /* PCM high voltage set up */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);
    /* 48MHz be MCLK */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

void initialADC(void) {
    /* 1.2 Reference Voltage Setup */
    P5SEL0 |= BIT6 | BIT7;
    P5SEL1 |= BIT6 | BIT7;
    REFCTL0 |= REFON;
    REFCTL0 |= REFOUT;
    REFCTL0 &= ~(REFVSEL_3);
    REFCTL0 |= REFVSEL_0;
    while (REFCTL0 & REFGENBUSY);
    /* 10 bit ADC read
     * Manual Mode*/
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    ADC14_setResolution(ADC_10BIT);
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A5, false);
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
}

void initialInterupt(void) {
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(960);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_SysTick_enableInterrupt();
}

/************************************ Loop Function **********************************/
float getInput(void) {
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
    while (ADC14_isBusy());
    return ADC14_getResult(ADC_MEM0);
}

float regulateInputSignal(float input) {
    return (input/4 - 127.5);
}

void updateBuffer(float *buffer) {
    buffer[0] = buffer[1];
    buffer[1] = buffer[2];
}

float stageFilter(float *buffer_x, float *buffer_y, float *stage_num, float *stage_den, float numGain, float denGain) {
    return numGain * (stage_num[0]*buffer_x[2] + stage_num[1]*buffer_x[1] + stage_num[2] * buffer_x[0]) -
            denGain * (stage_den[1]*buffer_y[1] + stage_den[2] * buffer_y[0]);
}

float bandstopFilter(float input) {
    updateBuffer(buffer_input);
    updateBuffer(buffer_stage1_output);
    //updateBuffer(buffer_stage2_output);
    buffer_input[2] = input;
    buffer_stage1_output[2] = stageFilter(buffer_input,buffer_stage1_output,num_1,den_1,num_gain_1,den_gain_1);
    //buffer_stage2_output[2] = stageFilter(buffer_stage1_output,buffer_stage2_output,num_2,den_2,num_gain_2,den_gain_2);

    return buffer_stage1_output[2];
}

float multiplexer(float input) {
    float output = input * cos_value[cos_counter];
    cos_counter++;
    if(cos_counter == PTS) cos_counter = 0;
    return output;
}

float regulateOutputSignal(float input) {
    return (input + 127.5);
}

/************************************ main function *********************************/
int main(void){
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    initialCosineWave();
    initialPin();
    initialClock();
    initialADC();
    initialInterupt();
    /* Start Interrupt */
    MAP_Interrupt_enableMaster();

    while (1)
    {
        //MAP_PCM_gotoLPM0();
    }
}

/********************************* ISR Loop Function ***************************************/
void SysTick_Handler(void) {
    P6OUT |= BIT0; // set P6 PIN0 High

    signal = getInput();
    signal_inputRegulated = regulateInputSignal(signal);
    signal_bandstopped = bandstopFilter(signal_inputRegulated);
    signal_multiplexed = multiplexer(signal_bandstopped);
    signal_outputRegulated = regulateOutputSignal(signal_multiplexed);

    P2OUT = signal_outputRegulated;

    P6OUT &= ~BIT0;
}



