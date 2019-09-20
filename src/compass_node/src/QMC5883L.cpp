#include "QMC5883L.h"

#include <wiringPiI2C.h>
#include <math.h>
#include <iostream>

/*
 * QMC5883L
 * http://wiki.epalsite.com/images/7/72/QMC5883L-Datasheet-1.0.pdf
 * https://github.com/dthain/QMC5883L
 */

/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

int fd, result;

static void write_register( int addr, int reg, int value )
{
  wiringPiI2CWrite(fd, reg);
  wiringPiI2CWrite(fd, value);

  // Wire.beginTransmission(addr);
  // Wire.write(reg);
  // Wire.write(value);
  // Wire.endTransmission();
}

static int read_register( int addr, int reg, int count )
{
  wiringPiI2CWrite(fd, reg);
  // Wire.beginTransmission(addr);
  // Wire.write(reg);
  // Wire.endTransmission();

  int n = wiringPiI2CReadReg16(fd, reg);

  // Wire.requestFrom(addr,count);
  // int n = Wire.available();
  // if(n!=count) return 0;

  return n;
}

void QMC5883L::reconfig()
{
  write_register(addr,QMC5883L_CONFIG,oversampling|range|rate|mode);
}

void QMC5883L::reset()
{
  write_register(addr,QMC5883L_RESET,0x01);
  reconfig();
}

void QMC5883L::setOversampling( int x )
{
  switch(x) {
    case 512:
      oversampling = QMC5883L_CONFIG_OS512;
      break;
    case 256:
      oversampling = QMC5883L_CONFIG_OS256;
      break;
    case 128:
      oversampling = QMC5883L_CONFIG_OS128;
      break;
    case 64:
      oversampling = QMC5883L_CONFIG_OS64;
      break;
  }
  reconfig();
}

void QMC5883L::setRange( int x )
{
  switch(x) {
    case 2:
      range = QMC5883L_CONFIG_2GAUSS;
      break;
    case 8:
      range = QMC5883L_CONFIG_8GAUSS;
      break;
  }
  reconfig();
}

void QMC5883L::setSamplingRate( int x )
{
  switch(x) {
    case 10:
      rate = QMC5883L_CONFIG_10HZ;
      break;
    case 50:
      rate = QMC5883L_CONFIG_50HZ;
      break;
    case 100:
      rate = QMC5883L_CONFIG_100HZ;
      break;
    case 200:
      rate = QMC5883L_CONFIG_200HZ;
      break;
  }
  reconfig();
}

void QMC5883L::init() {
  /* This assumes the wire library has been initialized. */
  addr = QMC5883L_ADDR;
  oversampling = QMC5883L_CONFIG_OS512;
  range = QMC5883L_CONFIG_2GAUSS;
  rate = QMC5883L_CONFIG_50HZ;
  mode = QMC5883L_CONFIG_CONT;

  fd = wiringPiI2CSetup(QMC5883L_ADDR);

  reset();
}

int QMC5883L::ready()
{
  if(!read_register(addr,QMC5883L_STATUS,1)) return 0;
  int status = wiringPiI2CRead(fd);
  // int status = Wire.read();
  return status & QMC5883L_STATUS_DRDY;
}

int QMC5883L::readRaw( int *x, int *y, int *z, int *t )
{
  while(!ready()) {}

  if(!read_register(addr,QMC5883L_X_LSB,6)) return 0;

  // *x = Wire.read() | (Wire.read()<<8);
  // *y = Wire.read() | (Wire.read()<<8);
  // *z = Wire.read() | (Wire.read()<<8);

  *x = wiringPiI2CRead(fd) | (wiringPiI2CRead(fd)<<8);
  *y = wiringPiI2CRead(fd)| (wiringPiI2CRead(fd)<<8);
  *z = wiringPiI2CRead(fd) | (wiringPiI2CRead(fd)<<8);


  return 1;
}

void QMC5883L::resetCalibration() {
  xhigh = yhigh = 0;
  xlow = ylow = 0;
}

int QMC5883L::readHeading()
{
  int x, y, z, t;
        std::cout << "heading reading" << std::endl;

  if(!readRaw(&x,&y,&z,&t)) return 0;

  /* Update the observed boundaries of the measurements */

  if(x<xlow) xlow = x;
  if(x>xhigh) xhigh = x;
  if(y<ylow) ylow = y;
  if(y>yhigh) yhigh = y;

  /* Bail out if not enough data is available. */

  if( xlow==xhigh || ylow==yhigh ) return 0;

  /* Recenter the measurement by subtracting the average */

  x -= (xhigh+xlow)/2;
  y -= (yhigh+ylow)/2;

  /* Rescale the measurement to the range observed. */

  float fx = (float)x/(xhigh-xlow);
  float fy = (float)y/(yhigh-ylow);

  int heading = 180.0*atan2(fy,fx)/M_PI;
  if(heading<=0) heading += 360;

  return heading;
}
