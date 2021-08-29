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

const uint8_t RCU_ID_LOCAL = RCU_ID_PYRO_RCU;

/*
 * 
 */

uint16_t last_2Hz_time, last_10Hz_time, last_200Hz_time, last_hb_rx_time, shutdown_req_tim;
uint8_t hb_rx_flag, shutdown, connected;

struct Heartbeat_t hb;
struct PyroControl_t pyrocon;
struct PyroStatus_t pyrostat;

char msg[64];

void on_can_rx(const struct can_msg_t *msg);

uint8_t pyrocon_rx_flag;

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

    //RB0 is ignitor continuity
    //RB1 is drogue chute continuity
    //RB2 is main chute continuity
    //RB3 is ignitor fire
    //RB4 is drogue fire
    //RB5 is main fire
    ANSELB = 0x00; //no analog on port B
    TRISB3 = TRISB4 = TRISB5 = 0; //outputs on RB3,4,5
    WPUB0 = WPUB1 = WPUB2 = 1; //weak pull-up on RB0,1,2

    while (1) {
        uint16_t ms = time_millis();
        if (pyrocon_rx_flag || ms - last_200Hz_time > 5) { //200Hz or pyro update request
            pyrocon_rx_flag = 0;
            last_200Hz_time = ms;

            if (connected) {
                //update pyro outputs as required
                CAN_RX_SUSPEND(); //prevent new msg rx from disrupting write to output pins
                LATB3 = pyrocon.fire_ignitor;
                LATB4 = pyrocon.fire_drogue;
                LATB5 = pyrocon.fire_main;
                CAN_RX_RESUME();
            } else {
                LATB3 = LATB4 = LATB5 = 0; //all ignitors off on connection loss
                //todo:
                //in an emergency situation where the bus fails, we still want this board to fire
                //the chute pyros so that we don't kermit soduko
                //one way to accomplish this is to have the Main RCU send a "pyro timer" msg
                //on the bus, with fields directing the pyros to fire a certain amount of time in the future
                //While everything is working properly, the timers come from a physics calculation done by the Main RCU
                //as to apogee and such. If the bus stops working in flight, the pyros will still fire when the most recent
                //timers elapse.
            }

            leds_connected(connected); //blink LED to show connection status

            pyrostat.pyro_voltage_raw = adc_read(0); //RA0 is voltage sense for ignitor battery
        }
        if (ms - last_10Hz_time > 100) { //10Hz
            last_10Hz_time = ms;

            //handle connection timeouts
            connected = can_hb_check_connected(ms);

            //transmit continuity reading for each pyro channel
            pyrostat.ignitor_cont = ~PORTBbits.RB0; //active low (pulldown by transistor switch)
            pyrostat.drogue_cont = ~PORTBbits.RB1;
            pyrostat.main_cont = ~PORTBbits.RB2;
            can_txq_push(CAN_ID_PyroStatus, CAN_CONVERT(pyrostat));
        }
        if (ms - last_2Hz_time > 500) { //2Hz
            last_2Hz_time = ms;
            //send a heartbeat msg
            hb.health = HEALTH_NOMINAL;
            hb.uptime_s = time_secs();
            can_txq_push(CAN_ID_Heartbeat, CAN_CONVERT(hb));
        }
    }
}

void on_can_rx(const struct can_msg_t *msg) {
    switch (msg->id) {
        case (CAN_ID_Heartbeat | RCU_ID_MAIN_RCU): //heartbeat from main RCU
            //set flag to indicate heartbeat received. main loop can note the time
            if (msg->len == sizeof (struct Heartbeat_t)) {
                hb_rx_flag = 1;
            }
            break;
        case (CAN_ID_PyroControl | RCU_ID_MAIN_RCU):
            if (msg->len == sizeof (struct PyroControl_t)) {
                pyrocon = *((struct PyroControl_t *) (msg->data));
                pyrocon_rx_flag = 1;
            }
            break;
    }
}
