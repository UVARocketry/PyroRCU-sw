/*
 * File:   main.c
 * Author: henry
 *
 * Created on July 28, 2021, 6:10 PM
 */

#include <xc.h> //compiler

//standard C libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

//code common to all RCUs
#include "libpicutil/config_mem.h" //configuration fuses for chip options
#include "libpicutil/time.h"
#include "libpicutil/uart_debug.h"
#include "libpicutil/leds.h"
#include "libcan/can_messages.h"
#include "libcan/can.h"
#include "libpicutil/adc.h"

//code specific to this RCU

#define _XTAL_FREQ 64000000UL //needed for delays to work, but not much else

const uint8_t RCU_ID_LOCAL = RCU_ID_POWER_PYRO_RCU;

/*
 * 
 */

uint16_t last_2Hz_time, last_10Hz_time, last_200Hz_time, last_hb_rx_time, shutdown_req_time;

uint8_t hb_rx_flag, shutdown, connected;

struct Heartbeat_t hb;
struct PowerControl_t pwrcon;
struct PowerStatus_t pwrstat;
struct PyroControl_t pyrocon;
struct PyroStatus_t pyrostat;

char msg[64];

void on_can_rx(const struct can_msg_t *msg);

uint8_t shutdown_req_flag;

int main() {
    //RC3 is the connection LED
    //RC2 is the power LED
    ANSELCbits.ANSELC2 = 0;
    TRISCbits.TRISC2 = 0;

    INTCON0bits.GIE = 1; //enable global interrupts

    time_init();
    leds_init();
    uart_init();
    adc_init();
    can_rx_callback = &on_can_rx;
    can_init();

    ANSELB = 0x00; //no analog on port B
    TRISB = 0b00000110; //RB3-6 are outputs
    //RB0 is ignitor continuity
    //RB1 is drogue chute continuity
    //RB2 is main chute continuity
    //RB3 is ignitor fire
    //RB4 is drogue fire
    //RB5 is main fire
    //RB6 is the power switch
    //RB7 is external power sense

    while (1) {
        if (time_millis() - last_200Hz_time > 5) {//200Hz
            last_200Hz_time = time_millis();
            //update pyro outputs as required
            LATBbits.LATB3 = pyrocon.fire_ignitor;
            LATBbits.LATB4 = pyrocon.fire_drogue;
            LATBbits.LATB5 = pyrocon.fire_main;

            leds_connected(connected); //blink LED to show connection status
        }
        if (time_millis() - last_10Hz_time > 10) {//10Hz
            last_10Hz_time = time_millis();

            //transmit continuity reading for each pyro channel
            pyrostat.ignitor_cont = PORTBbits.RB0;
            pyrostat.drogue_cont = PORTBbits.RB1;
            pyrostat.main_cont = PORTBbits.RB2;
            can_txq_push(ID_PYRO_STATUS, sizeof (struct PyroStatus_t), (uint8_t*) & pyrostat);

            if (pwrstat.flags.shutdown_req) { //if a shutdown was previously requested
                if (pwrcon.shutdown_request) { //if it is still requested
                    if (time_millis() - shutdown_req_time > 20000) { //and if some time has elapsed since first request
                        shutdown = 1; //then, you can shutdown
                    }
                } else {
                    pwrstat.flags.shutdown_req = 0; //shutdown no longer requested
                }
            } else {
                if (pwrcon.shutdown_request) { //first shutdown request
                    pwrstat.flags.shutdown_req = 1;
                    shutdown_req_time = time_millis(); //note the time of it
                }
            }

            //update power switch output based on shutdown flag
            //HIGH output turns power to system on
            LATBbits.LATB6 = !shutdown;

            //read voltages, currents. transmit power status
            pwrstat.voltage_12V_mV = adc_read(0); //RA0 is the battery voltage sense
            pwrstat.current_12V_mA = adc_read(1); //RA1 is battery current sense
            pwrstat.current_5V_mA = adc_read(2); //RA2 is logic current sense
            pwrstat.flags.external_power_connected = PORTBbits.RB7;
            pwrstat.flags.overcurrent_12V = pwrstat.current_12V_mA > 15000;
            pwrstat.flags.overcurrent_5V = pwrstat.current_5V_mA > 5000;
            pwrstat.flags.overvoltage = pwrstat.voltage_12V_mV > 14000;
            pwrstat.flags.undervoltage = pwrstat.voltage_12V_mV < 11000;
            can_txq_push(ID_POWER_STATUS, sizeof (struct PowerStatus_t), (uint8_t*) & pwrstat);

            //check for heartbeat timeout from main RCU
            if (connected) {
                if (time_millis() - last_hb_rx_time > 1000) {
                    connected = 0;
                }
            }
        }
        if (time_millis() - last_2Hz_time > 500) { //2Hz
            last_2Hz_time = time_millis();
            //send a heartbeat msg
            hb.health = HEALTH_NOMINAL;
            hb.uptime_s = time_secs();
            can_txq_push(ID_HEARTBEAT, sizeof (struct Heartbeat_t), (uint8_t *) & hb);
            //power LED blinks for shutdown requested
            if (pwrstat.flags.shutdown_req) {
                LATCbits.LATC2 ^= 1;
            } else {
                //solid for power on
                LATCbits.LATC2 = 1;
            }
        }
        if (hb_rx_flag) { //on heartbeat receive
            hb_rx_flag = 0;
            connected = 1;
            last_hb_rx_time = time_millis(); //note the time for timeout checking
            LATCbits.LATC3 = 1; //LED on for start of 100ms blink after hb rx
        }
        if (shutdown) { //on shutdown
            LATCbits.LATC2 = LATCbits.LATC3 = 0; //LEDs off
            LATB = 0b01111000; //pyro channels and power switch off
            //todo:
            //put MCU in sleep mode, all peripherals disabled
            //set pins to minimize power draw
            //power is restored by pressing a button connected to MCLR to reboot this MCU
            //this restores system power.
            while (1)
                ;
        }
    }
}

void on_can_rx(const struct can_msg_t *msg) {
    switch (msg->id) {
        case (ID_HEARTBEAT | RCU_ID_MAIN_RCU): //heartbeat from main RCU
            //set flag to indicate heartbeat received. main loop can note the time
            if (msg->len == sizeof (struct Heartbeat_t)) {
                hb_rx_flag = 1;
            }
            break;
        case (ID_POWER_CONTROL | RCU_ID_MAIN_RCU):
            if (msg->len == sizeof (struct PowerControl_t)) {
                pwrcon = *(struct PowerControl_t*) msg;
            }
            break;
        case (ID_PYRO_CONTROL | RCU_ID_MAIN_RCU):
            if (msg->len == sizeof (struct PyroControl_t)) {
                pyrocon = *(struct PyroControl_t*) msg;
            }
            break;
    }
}
