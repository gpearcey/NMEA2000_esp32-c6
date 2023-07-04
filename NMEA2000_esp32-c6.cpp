/*
NMEA2000_esp32.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for ESP32 modules. See also NMEA2000 library.

Thanks to Thomas Barth, barth-dev.de, who has written ESP32 CAN code. To avoid extra
libraries, I implemented his code directly to the NMEA2000_esp32 to avoid extra
can.h library, which may cause even naming problem.
*/

#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
// enable support for ESP-IDF v5.0+
#include "esp_private/periph_ctrl.h"
#else
#include "driver/periph_ctrl.h"
#endif

#include "rom/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/pcr_reg.h"
#include "soc/dport_access.h"
#include "NMEA2000_esp32-c6.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#if !defined(round)
#include <math.h>
#endif

bool tNMEA2000_esp32c6::CanInUse=false;
tNMEA2000_esp32c6 *pNMEA2000_esp32=0;

void ESP32Can1Interrupt(void *);

//*****************************************************************************
tNMEA2000_esp32c6::tNMEA2000_esp32c6(gpio_num_t _TxPin,  gpio_num_t _RxPin) :
    tNMEA2000(), IsOpen(false),
                                                                         speed(CAN_SPEED_250KBPS), TxPin(_TxPin), RxPin(_RxPin),
    RxQueue(NULL), TxQueue(NULL) {}

//*****************************************************************************
bool tNMEA2000_esp32c6::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/) {
  if ( uxQueueSpacesAvailable(TxQueue)==0 ) return false; // can not send to queue

  //tCANFrame frame;
  //frame.id=id;
  //frame.len=len>8?8:len;
  //memcpy(frame.buf,buf,len);

  twai_message_t message;
  message.identifier = id;
  message.extd = 1;
  message.data_length_code = len>8?8:len;
  memcpy(message.data, buf, len);

  //Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
      printf("Message queued for transmission\n");
  } else {
      printf("Failed to queue message for transmission\n");
  }

  return true;
}

//*****************************************************************************
void tNMEA2000_esp32c6::InitCANFrameBuffers() {
    if (MaxCANReceiveFrames<10 ) MaxCANReceiveFrames=50; // ESP32 has plenty of RAM
    if (MaxCANSendFrames<10 ) MaxCANSendFrames=40;
    uint16_t CANGlobalBufSize=MaxCANSendFrames-4;
    MaxCANSendFrames=4;  // we do not need much libary internal buffer since driver has them.
    RxQueue=xQueueCreate(MaxCANReceiveFrames,sizeof(tCANFrame));
    TxQueue=xQueueCreate(CANGlobalBufSize,sizeof(tCANFrame));

  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_esp32c6::CANOpen() {
    if (IsOpen) return true;

    if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.

    pNMEA2000_esp32=this;
    IsOpen=true;
  CAN_init();

    CanInUse=IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_esp32c6::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  //CAN_read_frame();
  bool HasFrame=false;
  tCANFrame frame;

    //receive next CAN frame from queue
    if ( xQueueReceive(RxQueue,&frame, 0)==pdTRUE ) {
      HasFrame=true;
      id=frame.id;
      len=frame.len;
      memcpy(buf,frame.buf,frame.len);
      printf("frame received\n");
  }

  return HasFrame;
}

//*****************************************************************************
void tNMEA2000_esp32c6::CAN_init() {
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}

//*****************************************************************************
void tNMEA2000_esp32c6::CAN_read_frame() {
  tCANFrame frame;
  CAN_FIR_t FIR;

  //Wait for message to be received
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
      printf("Message received\n");
      printf("Message id %ld \n", message.identifier);
  } else {
      printf("Failed to receive message\n");
      return;
  }

  // Handle only extended frames
  if (message.extd) {  //extended frame
    //Get Message ID
      frame.id = message.identifier;
    frame.len = message.data_length_code;

    //deep copy data bytes
    for( size_t i=0; i<frame.len; i++ ) {
      frame.buf[i]=message.data[i];
    }

    //send frame to input queue
    xQueueSendToBackFromISR(RxQueue,&frame,0);
    printf("frame added to RxQueue \n");
  }

}

//*****************************************************************************
void tNMEA2000_esp32c6::CAN_send_frame(tCANFrame &frame) {
  printf("CAN_Send_frame");
  CAN_FIR_t FIR;

  FIR.U=0;
  FIR.B.DLC=frame.len>8?8:frame.len;
  FIR.B.FF=CAN_frame_ext;

	//copy frame information record
	MODULE_CAN_0->MBX_CTRL.FCTRL.FIR.U=FIR.U;

  //Write message ID
  _CAN_SET_EXT_ID_0(frame.id);

  // Copy the frame data to the hardware
  for ( size_t i=0; i<frame.len; i++) {
    MODULE_CAN_0->MBX_CTRL.FCTRL.TX_RX.EXT.data[i]=frame.buf[i];
  }

  // Transmit frame
  MODULE_CAN_0->CMR.B.TR=1;
}

//*****************************************************************************
void tNMEA2000_esp32c6::InterruptHandler() {
  printf("interupt handler calledddddddddddddd");
  CAN_read_frame();
	//Interrupt flag buffer
  uint32_t interrupt;

  // Read interrupt status and clear flags
  interrupt = (MODULE_CAN_0->IR.U & 0xff);

  // Handle TX complete interrupt
    if ((interrupt & __CAN_IRQ_TX) != 0) {
    tCANFrame frame;
      if ( (xQueueReceiveFromISR(TxQueue,&frame,NULL)==pdTRUE) ) {
      CAN_send_frame(frame);
    }
  }

  // Handle RX frame available interrupt
    if ((interrupt & __CAN_IRQ_RX) != 0) {
    CAN_read_frame();
  }

  // Handle error interrupts.
    if ((interrupt & (__CAN_IRQ_ERR						//0x4
                      | __CAN_IRQ_DATA_OVERRUN			//0x8
                      | __CAN_IRQ_WAKEUP				//0x10
                      | __CAN_IRQ_ERR_PASSIVE			//0x20
                      | __CAN_IRQ_ARB_LOST				//0x40
                      | __CAN_IRQ_BUS_ERR				//0x80
        )) != 0) {
    /*handler*/
  }
}

//*****************************************************************************
void ESP32Can1Interrupt(void *) {
  printf("interrupt handler called");
  pNMEA2000_esp32->InterruptHandler();
}
