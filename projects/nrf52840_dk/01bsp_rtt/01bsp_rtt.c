/**
\brief This program implement RTT on nrf52840.

\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Novermber 2023.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "ppi.h"
#include "sctimer.h"
#include "timer.h"
#include "leds.h"
#include "eui64.h"
#include "radio.h"
#include "uart.h"
#include "nrf52840.h"


//=========================== defines =========================================

#define NUM_PPI_CHANNEL_USED  4

#define SET_TIME_PERIOD       16000000  //1s on 16MHz
#define SCHEDULED_TIME        16000000  //1s on 16MHz
#define TIMER_PERIOD          (0xffff>>4)    ///< 0xffff = 2s@32kHz

#define SENDER_LAST_ID        173       //---707
#define RECEIVER_LAST_ID      172       //---014

#define CC0                   0               //CC0 using for EoF capture into CC0
#define CC1                   1               //CC1 using for SoF capture into CC1

//-------radio define-------
#define CHANNEL               16
#define LENGTH_PACKET         20+LENGTH_CRC   //maximum length is 127 bytes
#define LEN_PKT_TO_SEND       20+LENGTH_CRC

//-------PPI define-------
#define PPI0                   0               //PPI0 using for Radio EoF -> Timer Capture  T1 into CC0
#define PPI1                   1               //PPI1 using for Radio SoF -> Timer Capture  T2 into CC1
#define PPI2                   2               //PPI2 using for Timer schedule -> Radio TX 
//=========================== variables =======================================
//uint8_t stringToSend[]  = "+002 Ptest.24.00.12.-010\n";

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
    
    uint8_t              num_rx_startFrame;
    uint8_t              num_rx_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
    uint8_t              txpk_buf[LENGTH_PACKET];
    uint8_t              txpk_len;
    //uint8_t              stringtosend[6];

    uint8_t              packet[LENGTH_PACKET];
    uint8_t              packet_len;
    int8_t               rxpk_rssi;
    uint8_t              rxpk_lqi;
    bool                 rxpk_crc;
                  
                  uint8_t         flags;
                  app_state_t     state;
                  uint8_t uart_lastTxByteIndex;
   volatile       uint8_t uartDone;
   volatile       uint8_t uartSendNow;
   volatile       uint8_t uartToSend[6];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

int sender_main(void);
int receiver_main(void);

void cb_endFrame(PORT_TIMER_WIDTH timestamp);
void cb_startFrame(PORT_TIMER_WIDTH timestamp);

void     cb_uart_tx_done(void);
uint8_t  cb_uart_rx(void);

void cb_timer(void);

void delay_ms(int16_t times);


//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint8_t board_eui[8];
    eui64_get(board_eui);

    while(1) {

        if (board_eui[7] == SENDER_LAST_ID){
            sender_main();
        }
        else if (board_eui[7] == RECEIVER_LAST_ID){
             receiver_main();
        }   
    }  
}

//=========================== callbacks =======================================

int sender_main(void){
    board_init();
    timer_init();
    
    uint32_t T1;
    uint32_t T2;

    //config PPI
    ppi_radio_eof_timer_capture(PPI0, CC0);    //using PPI channel 0, EoF capture time value to CC0    T1
    ppi_enable(PPI0);
    ppi_radio_sof_timer_capture(PPI1, CC1);    //using PPI channel 1, SoF capture time value to CC1    T2   
    ppi_enable(PPI1);
    
    timer_clear();
    timer_start();

    // setup UART
    uart_setCallbacks(cb_uart_tx_done,cb_uart_rx);
    uart_enableInterrupts();

    app_vars.uartDone = 1;
    

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    // prepare radio
    radio_rfOn();

    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);

    // prepare packet
    //app_vars.txpk_buf[0] = 1; // Packet number or other data
    //app_vars.txpk_len = sizeof(app_vars.txpk_buf);

    // send packet
    //radio_loadPacket(app_vars.txpk_buf, app_vars.txpk_len);
    //radio_txEnable();
    //leds_error_on();
    //radio_txNow();

    
    radio_rxEnable();
    //leds_error_off();
    //leds_sync_on();
    //radio_rxNow();
    
    app_vars.state = APP_STATE_RX;
    
    // start by a transmit
    app_vars.flags |= APP_FLAG_TIMER;
    //app_vars.flags &= ~APP_FLAG_TIMER;
    //while(!(NRF_RADIO->EVENTS_END == 1)){
    //}                 //waiting for receiving a packet
    while(1) {

        while(app_vars.flags==0x00) {
            board_sleep();
        }

        while (app_vars.flags) {
            
            //===========  APP_FLAG_START_FRAME (TX or RX)
            if (app_vars.flags & APP_FLAG_START_FRAME) {
                // start of frame

                switch (app_vars.state) {
                    case APP_STATE_RX:
                        // started receiving a packet
                        //T2 = timer_getCapturedValue(CC1);
                        // led
                        leds_error_on();
                        break;
                    case APP_STATE_TX:
                        // started sending a packet

                        // led
                        leds_sync_on();
                    break;
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_START_FRAME;
            }

            //==== APP_FLAG_END_FRAME (TX or RX)
            if (app_vars.flags & APP_FLAG_END_FRAME) {
                //end of frame

                switch (app_vars.state) {
                    case APP_STATE_RX:
                        
                        // done receiving a packet
                        radio_getReceivedFrame(
                                                app_vars.packet,
                                                &app_vars.packet_len,
                                                sizeof(app_vars.packet),
                                                &app_vars.rxpk_rssi,
                                                &app_vars.rxpk_lqi,
                                                &app_vars.rxpk_crc
                                            );

                        //T1 = timer_getCapturedValue(CC0);
                       T2 = timer_getCapturedValue(CC1);

                        uint32_t tmp = T2 - T1;
                        int i = 0;
                        app_vars.uartToSend[i++] = (uint8_t)((tmp >> 24) & 0x000000ff);
                        app_vars.uartToSend[i++] = (uint8_t)((tmp >> 16) & 0x000000ff);
                        app_vars.uartToSend[i++] = (uint8_t)((tmp >> 8) & 0x000000ff);
                        app_vars.uartToSend[i++] = (uint8_t)((tmp >> 0) & 0x000000ff);

                        app_vars.uartToSend[i++] = '\r';
                        app_vars.uartToSend[i++] = '\n';

                        //uart_writeByte(app_vars.uartToSend[6]);

                        // send string over UART
                        if (app_vars.uartDone == 1) {
                            app_vars.uartDone              = 0;
                            app_vars.uart_lastTxByteIndex  = 0;
                            uart_writeByte(app_vars.uartToSend[app_vars.uart_lastTxByteIndex]);
                        }

                        leds_error_off();
                        break;
                    
                    case APP_STATE_TX:
                        // done sending a packet

                        // switch to RX mode
                        radio_rxEnable();
                        radio_rxNow();
                        app_vars.state = APP_STATE_RX;

                        T1 = timer_getCapturedValue(CC0);

                        // led
                        leds_sync_off();
                        break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            //==== APP_FLAG_TIMER

            if (app_vars.flags & APP_FLAG_TIMER){
                // timer fired

                if (app_vars.state==APP_STATE_RX){
                    // stop listening
                    radio_rfOff();

                    // prepare packet
                    app_vars.txpk_buf[0] = 1; // Packet number or other data
                    app_vars.txpk_len = sizeof(app_vars.txpk_buf);

                    // send packet
                    radio_loadPacket(app_vars.txpk_buf, app_vars.txpk_len);
                    radio_txEnable();
                    radio_txNow();

                    app_vars.state = APP_STATE_TX;
                }

                app_vars.flags &= ~APP_FLAG_TIMER;
            }
        }
    }
}


int receiver_main(void){
    board_init();
    timer_init();
    
    //config PPI
    ppi_timer_compare_radio_start(PPI2,2);
    ppi_enable(PPI2);

    timer_clear();
    timer_start();

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);

    // prepare packet
    app_vars.txpk_buf[0] = 2; // Packet number or other data
    app_vars.txpk_len = sizeof(app_vars.txpk_buf);

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
    sctimer_enable();

    radio_rxEnable();
    //leds_sync_on();
    radio_rxNow();

    app_vars.state = APP_STATE_RX;
    
    // start by a listening
    //app_vars.flags |= APP_FLAG_START_FRAME;

    // start by a transmit
    app_vars.flags |= APP_FLAG_TIMER;

    while(1) {
        while(app_vars.flags == 0x00) {
            board_sleep();
        }

        while(app_vars.flags) {
            
            //==== APP_FLAG_START_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_START_FRAME) {
                //start of frame
                switch (app_vars.state) {
                    case APP_STATE_RX:
                        // started receiving a packet
                        timer_schedule(2,16000000);    //16000 = 1ms
                        // led
                        leds_error_on();
                        break;
                    case APP_STATE_TX:
                        // started sending a packet
    
                        // led
                        leds_sync_on();
                    break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_START_FRAME;
            }
            
            //==== APP_FLAG_END_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_END_FRAME) {
                //end of frame
                switch (app_vars.state) {
                    case APP_STATE_RX:
                        
                        //done receiving a packet
                        radio_getReceivedFrame(
                                    app_vars.packet,
                                    &app_vars.packet_len,
                                    sizeof(app_vars.packet),
                                    &app_vars.rxpk_rssi,
                                    &app_vars.rxpk_lqi,
                                    &app_vars.rxpk_crc
                                );
                        
                        //timer_schedule(2,16000);    //16000 = 1ms

                        radio_loadPacket(app_vars.txpk_buf, app_vars.txpk_len);
                        radio_txEnable();
                        
                        //delay_ms(3000);
                        radio_txNow();
                        app_vars.state = APP_STATE_TX;
                        // led
                        leds_error_off();
                        //memset(&app_vars.packet,0,sizeof(app_vars.packet));
                        break;

                    case APP_STATE_TX:
                        // done sending a packet

                        // switch to RX mode
                        radio_rxEnable();
                        radio_rxNow();
                        app_vars.state = APP_STATE_RX;

                        // led
                        leds_sync_off();
                        break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            //==== APP_FLAG_TIMER

            if (app_vars.flags & APP_FLAG_TIMER){
                // timer fired

                if (app_vars.state==APP_STATE_TX){
                    radio_rxEnable();
                    radio_rxNow();
                    app_vars.state = APP_STATE_RX;
                }

                app_vars.flags &= ~APP_FLAG_TIMER;
            }
        }
    }
}

//=================callbacks function=====================
void cb_endFrame(PORT_TIMER_WIDTH timestamp) {

    // set flag
    app_vars.flags |= APP_FLAG_END_FRAME;

    // update debug stats
    app_dbg.num_endFrame++;

    if (app_vars.state == APP_STATE_RX) {
        app_dbg.num_rx_endFrame++;
    }
}

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_START_FRAME;
    
    // update debug stats
    app_dbg.num_startFrame++;

    if (app_vars.state == APP_STATE_RX) {
        app_dbg.num_rx_startFrame++;
    }
}

void cb_uart_tx_done(void) {
    app_vars.uart_lastTxByteIndex++;
    if (app_vars.uart_lastTxByteIndex<6) {
        uart_writeByte(app_vars.uartToSend[app_vars.uart_lastTxByteIndex]);
    } else {
        app_vars.uartDone = 1;
    }
}

uint8_t cb_uart_rx(void) {
    uint8_t byte;

    // toggle LED
    leds_error_toggle();

    // read received byte
    byte = uart_readByte();

    // echo that byte over serial
    uart_writeByte(byte);

    return 0;
}

void cb_timer(void) {
    // set flag
    app_vars.flags |= APP_FLAG_TIMER;

    // update debug stats
    app_dbg.num_timer++;

    sctimer_setCompare(sctimer_readCounter()+TIMER_PERIOD);
}

void delay_ms(int16_t times)
{
    // 计算延时的时钟周期数
    uint32_t cycles = times * 1000 * (SystemCoreClock / 1000000) / 3;
    // 使用DWT寄存器进行延时
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < cycles)
        ;
}