#include "ros/ros.h"
#include "lor.c"

#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

void tx_f(txData *tx){
    LoRa_ctl *modem = (LoRa_ctl *)(tx->userPtr);
    printf("tx done;\t");
    printf("sent string: \"%s\"\n\n", tx->buf);//Data we've sent

    LoRa_receive(modem);
}

void rx_f(rxData *rx){
    LoRa_ctl *modem = (LoRa_ctl *)(rx->userPtr);

    printf("CRC error: %d;\t", rx->CRC);
    printf("Data size: %d;\t", rx->size);
    printf("received string: \"%s\";\t", rx->buf);//Data we've received
    printf("RSSI: %d;\t", rx->RSSI);
    printf("SNR: %f\n", rx->SNR);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Lora Com Node");
  ros::NodeHandle n;

  char txbuf[255];
  char rxbuf[255];
  LoRa_ctl modem;

  //See for typedefs, enumerations and there values in LoRa.h header file
  modem.spiCS = 0;//Raspberry SPI CE pin number
  modem.tx.callback = tx_f;
  modem.tx.data.buf = txbuf;
  modem.rx.callback = rx_f;
  modem.rx.data.buf = rxbuf;
  modem.rx.data.userPtr = (void *)(&modem);//To handle with chip from rx callback
  modem.tx.data.userPtr = (void *)(&modem);//To handle with chip from tx callback
  memcpy(modem.tx.data.buf, "Rover alive - booted", 5);//copy data we'll sent to buffer
  modem.tx.data.size = 5;//Payload len
  modem.eth.preambleLen=6;
  modem.eth.bw = BW62_5;//Bandwidth 62.5KHz
  modem.eth.sf = SF12;//Spreading Factor 12
  modem.eth.ecr = CR8;//Error coding rate CR4/8
  modem.eth.CRC = 1;//Turn on CRC checking
  modem.eth.freq = 868000000;// 434.8MHz
  modem.eth.resetGpioN = 4;//GPIO4 on lora RESET pin
  modem.eth.dio0GpioN = 17;//GPIO17 on lora DIO0 pin to control Rxdone and Txdone interrupts
  modem.eth.outPower = OP20;//Output power
  modem.eth.powerOutPin = PA_BOOST;//Power Amplifire pin
  modem.eth.AGC = 1;//Auto Gain Control
  modem.eth.OCP = 240;// 45 to 240 mA. 0 to turn off protection
  modem.eth.implicitHeader = 0;//Explicit header mode
  modem.eth.syncWord = 0x12;
  //For detail information about SF, Error Coding Rate, Explicit header, Bandwidth, AGC, Over current protection and other features refer to sx127x datasheet https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf

  LoRa_begin(&modem);
  LoRa_send(&modem);

  printf("Time on air data - Tsym: %f;\t", modem.tx.data.Tsym);
  printf("Tpkt: %f;\t", modem.tx.data.Tpkt);
  printf("payloadSymbNb: %u\n", modem.tx.data.payloadSymbNb);

  ros::spin();

  return 0;
}
