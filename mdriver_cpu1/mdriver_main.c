/*
 * Main code for MDriver Firmware
 * -------------------------------------------------
 * This code is intended to be used with the MDriver Hardware version 1.0
 */

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include <stdbool.h>
#include "fbctrl.h"
#include <math.h>
#include <mdriver_adc.h>
#include <mdriver_cpu1.h>
#include <mdriver_cpu1.h>
#include <mdriver_cpu1.h>
#include <mdriver_epwm.h>
#include <mdriver_fsm.h>
#include <mdriver_hw_defs.h>
#include "ipc.h"

bool run_main_control_task=false;
bool enable_waveform_debugging=false;

#define IERR_TOL 2.0
#define OVERCURRENT_THRESHOLD 4.0

void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Boot CM core
    //
#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Setup heartbeat GPIO & input relay gpios
    //heartbeat
    GPIO_setDirectionMode(HEARTBEAT_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(HEARTBEAT_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //main input relay
    GPIO_setDirectionMode(MAIN_RELAY_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(MAIN_RELAY_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //input relay to slave
    GPIO_setDirectionMode(SLAVE_RELAY_GPIO, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(SLAVE_RELAY_GPIO,GPIO_PIN_TYPE_STD);        //push pull output
    //LED 1 for debugging
    GPIO_setDirectionMode(LED_1_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(LED_1_GPIO,GPIO_PIN_TYPE_STD);
    GPIO_writePin(LED_1_GPIO,1);
    //LED 2 for debugging
    GPIO_setDirectionMode(LED_2_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(LED_2_GPIO,GPIO_PIN_TYPE_STD);
    GPIO_writePin(LED_2_GPIO,1);
    //start of frame (SOF) GPIO
    GPIO_setDirectionMode(SOF_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(SOF_GPIO,GPIO_PIN_TYPE_STD);
    GPIO_writePin(SOF_GPIO,1);

    //
    // Initialize ADCs
    //
    //setup ADC clock, single-ended mode, enable ADCs
    initADCs();
    //setup ADC SOC configurations
    initADCSOCs();

    //
    // Initialize Half Bridges
    //
    //channel A
    setup_pinmux_config_bridge(&cha_bridge,0);
    //channel B
    setup_pinmux_config_bridge(&chb_bridge,1);
    //channel C
    setup_pinmux_config_bridge(&chc_bridge,2);
    //channel D
    setup_pinmux_config_bridge(&chd_bridge,3);
    //channel E
    setup_pinmux_config_bridge(&che_bridge,4);
    //channel F
    setup_pinmux_config_bridge(&chf_bridge,5);


    //
    // Initialize synchronization of bridges
    //
    //enable phase synchronization for all PWM channels
    //synchronization is to ePWM12 module on Pin22
    unsigned int channelno=0;
    for(channelno=0; channelno< NO_CHANNELS; channelno++){
        synchronize_pwm_to_epwm12(driver_channels,channelno);
    }
    //enable PWM outputs of ePWM11
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(20,GPIO_PIN_TYPE_STD);        //push pull output
    GPIO_setPinConfig(GPIO_20_EPWM11A);
    //set up ePWM11 (DAQ sample clock)
    initEPWMWithoutDB(EPWM11_BASE,false);
    setupEPWMActiveHighComplementary(EPWM11_BASE);
    EPWM_setClockPrescaler(EPWM11_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_8);
    EPWM_setTimeBasePeriod(EPWM11_BASE, EPWM_TIMER_TBPRD_BRIDGE/2);
    EPWM_setCounterCompareValue(EPWM11_BASE, EPWM_COUNTER_COMPARE_A, EPWM_TIMER_TBPRD_BRIDGE/4);
    //enable PWM outputs of EPWM5
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);   //output
    GPIO_setPadConfig(8,GPIO_PIN_TYPE_STD);        //push pull output
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    //synchronize all PWMs and trigger ADC conversions from PWM of Channel A
    EPWM_enableSyncOutPulseSource(EPWM12_BASE,EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
    EPWM_forceSyncPulse(EPWM12_BASE);
    EPWM_enableADCTrigger(EPWM7_BASE,EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM7_BASE,EPWM_SOC_A,EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM7_BASE,EPWM_SOC_A,1);

    //
    // Enable Half Bridges
    //
    //set_enabled(&cha_buck,true,true);
    set_enabled(&cha_bridge,false,true);
    set_enabled(&chb_bridge,false,true);
    set_enabled(&chc_bridge,false,true);
    set_enabled(&chd_bridge,false,true);
    set_enabled(&che_bridge,false,true);
    set_enabled(&chf_bridge,false,true);

    //
    // Initialize Reed Switch Interface
    //
    unsigned int n=0;
    for(n=0; n<NO_CHANNELS; n++){
        GPIO_setDirectionMode(driver_channels[n]->enable_resonant_gpio, GPIO_DIR_MODE_OUT);   //output
        GPIO_setPadConfig(driver_channels[n]->enable_resonant_gpio,GPIO_PIN_TYPE_STD);        //push pull output
    }

    //
    // Setup main control task interrupt
    //
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    Interrupt_initModule();
    // Initializes the PIE vector table with pointers to the shell Interrupt Service Routines (ISRs)
    Interrupt_initVectorTable();
    //--------------- IPC interrupt ---------------
    //clear any IPC flags
    IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);
    //register IPC interrupt from CM to CPU1 using IPC_INT0
    IPC_registerInterrupt(IPC_CPU1_L_CM_R, IPC_INT0, IPC_ISR0);
    //synchronize CM and CPU1 using IPC_FLAG31
    IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);
    //--------------- CPU1 Timer0 interrupt (main task) ---------------
    // Register ISR for cupTimer0
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    // Initialize CPUTimer0
    configCPUTimer(CPUTIMER0_BASE, 1e6*deltaT);
    // Enable CPUTimer0 Interrupt within CPUTimer0 Module
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    // Enable TIMER0 Interrupt on CPU coming from TIMER0
    Interrupt_enable(INT_TIMER0);
    // Start CPUTimer0
    CPUTimer_startTimer(CPUTIMER0_BASE);
    //--------------- CPU1 Timer1 interrupt (communication active) ---------------
    // Register ISR for cupTimer1
    Interrupt_register(INT_TIMER1, &cpuTimer1ISR);
    // Initialize CPUTimer1
    configCPUTimer(CPUTIMER1_BASE, 500000);
    // Enable CPUTimer0 Interrupt within CPUTimer1 Module
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    // Enable TIMER1 Interrupt on CPU coming from TIMER1
    Interrupt_enable(INT_TIMER1);
    // Start CPUTimer0
    CPUTimer_startTimer(CPUTIMER1_BASE);

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;


    //
    // Set up EnetCLK to use SYSPLL as the clock source and set the
    // clock divider to 2.
    //
    // This way we ensure that the PTP clock is 100 MHz. Note that this value
    // is not automatically/dynamically known to the CM core and hence it needs
    // to be made available to the CM side code beforehand.
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

    //
    // Configure the GPIOs for ETHERNET.
    //

    //
    // MDIO Signals
    //
    GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);

    //
    // Use this only for RMII Mode
    //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
    //

    //
    //MII Signals
    //
    GPIO_setPinConfig(GPIO_109_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_110_ENET_MII_COL);

    GPIO_setPinConfig(GPIO_75_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_122_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_123_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_124_ENET_MII_TX_DATA3);

    //
    //Use this only if the TX Error pin has to be connected
    //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
    //

    GPIO_setPinConfig(GPIO_118_ENET_MII_TX_EN);

    GPIO_setPinConfig(GPIO_114_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_115_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_116_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_117_ENET_MII_RX_DATA3);
    GPIO_setPinConfig(GPIO_113_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_112_ENET_MII_RX_DV);

    GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);
    GPIO_setPinConfig(GPIO_111_ENET_MII_RX_CLK);

    //
    //Power down pin to bring the external PHY out of Power down
    //
    GPIO_setDirectionMode(108, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(108, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(108,1);

    //
    //PHY Reset Pin to be driven High to bring external PHY out of Reset
    //

    GPIO_setDirectionMode(119, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(119, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(119,1);

    //
    // Initialize Outputs
    //
    for(n=0; n<NO_CHANNELS; n++){
        READY_enter(n);
        driver_channels[n]->channel_state=READY;
    }

    int32_t errint[NO_CHANNELS]={0};
    int32_t prev_errint[NO_CHANNELS]={0};
    uint32_t loop_counter=0;
    const uint32_t dec=1000;
    // Main Loop
    while(1){
        if(run_main_task){
            //toggle heartbeat gpio
            GPIO_writePin(HEARTBEAT_GPIO,0);


            //---------------------
            // State Machine
            //---------------------
            //Main Relay Opening Logic
            unsigned int channel_counter=0;
            bool main_relay_active=false;
            for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                run_channel_fsm(driver_channels[channel_counter]);
                //we enable the main relay when one channel is not in state READY anymore (e.g. when one channel requires power)
                if(driver_channels[channel_counter]->channel_state!=READY)
                    main_relay_active=true;
            }
            GPIO_writePin(MAIN_RELAY_GPIO,main_relay_active);
            GPIO_writePin(SLAVE_RELAY_GPIO,main_relay_active);
            GPIO_writePin(LED_1_GPIO,!main_relay_active);
            //Communication Active Logic (If no communication, issue a STOP command
            if(!communication_active){
                for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                    fsm_req_flags_stop[channel_counter]=1;
                }
            }

            //---------------------
            // Signal Acquisition & Filtering
            //---------------------

            // Read ADCs sequentially, this updates the system_dyn_state structure
            readAnalogInputs();

            //---------------------
            // Overcurrent Protection
            //---------------------
            for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                if(fabsf(system_dyn_state.is[channel_counter]) > OVERCURRENT_THRESHOLD){
                    // Overcurrent detected, issue stop to all channels
                    unsigned int k;
                    for(k=0; k<NO_CHANNELS; k++){
                        fsm_req_flags_stop[k]=1;
                    }
                    break;
                }
            }

            //---------------------
            // Control Law Execution & Output Actuation
            //---------------------
            unsigned int i=0;
            //set output duties for bridge [regular mode]
            for(i=0; i<NO_CHANNELS; i++){
                if(driver_channels[i]->channel_state==RUN_REGULAR){
                    #ifdef TUNE_CLOSED_LOOP
                        if(enable_waveform_debugging)
                            des_currents[i]=ides;
                        else
                            ides=0.0;
                    #endif
                    //execute the PI control low
                    float voltage_dclink=VIN;
                    //compute feed forward actuation term (limits [-1,1] for this duty) - feed-forward term currently not used
                    #ifdef FEED_FORWARD_ONLY
                        float act_voltage_ff=des_currents[i]*RDC;
                        float act_voltage_fb=0.0;
                    #endif
                    #ifdef CLOSED_LOOP
                        float act_voltage_ff=0.0;
                        //compute feedback actuation term (limits [-1,1] for this duty)
                        bool output_saturated=fabsf((current_pi+i)->u)>=0.95*voltage_dclink;

                        float act_voltage_fb=update_pid(current_pi+i,des_currents[i],system_dyn_state.is[i],output_saturated);

                        //sanity check for PID control law using moving average of error flags
                        if(fabsf(des_currents[i]-system_dyn_state.is[i])>IERR_TOL){
                            errint[i]++;
                        }
                        if(loop_counter%dec==0){
                            int32_t err_moving_avg=errint[i]-prev_errint[i];
                            prev_errint[i]=errint[i];
                            //if average error count over the last `dec` cycles was higher than 75%, issue stop flags
                            if(err_moving_avg>0.75*dec){
                                for(channel_counter=0; channel_counter<NO_CHANNELS; channel_counter++){
                                    fsm_req_flags_stop[channel_counter]=1;
                                }
                            }

                        }

                    #endif
                    float duty_ff=act_voltage_ff/voltage_dclink;
                    float duty_fb=act_voltage_fb/(voltage_dclink);

                    //convert normalized duty cycle, limit it and apply
                    float duty_bridge=0.5*(1+(duty_ff+duty_fb));
                    if(duty_bridge>0.9)
                        duty_bridge=0.9;
                    if(duty_bridge<0.1)
                        duty_bridge=0.1;
                    set_duty_bridge(driver_channels[i]->bridge_config,duty_bridge,i);
                }

            }

            //---------------------
            // Read State Of Bridges
            //---------------------
            uint32_t cha_bridge_state_u=GPIO_readPin(cha_bridge.state_v_gpio);
            uint32_t cha_bridge_state_v=GPIO_readPin(cha_bridge.state_u_gpio);

            uint32_t chb_bridge_state_u=GPIO_readPin(chb_bridge.state_v_gpio);
            uint32_t chb_bridge_state_v=GPIO_readPin(chb_bridge.state_u_gpio);

            uint32_t chc_bridge_state_u=GPIO_readPin(chc_bridge.state_v_gpio);
            uint32_t chc_bridge_state_v=GPIO_readPin(chc_bridge.state_u_gpio);

            run_main_task=false;
            GPIO_writePin(HEARTBEAT_GPIO,1);
            loop_counter++;
        }
    }
}


