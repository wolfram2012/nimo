#ifndef COMS_PROTOCOL_H
#define COMS_PROTOCOL_H
#include <stdint.h>

namespace coms
{
	typedef struct
	{
  		uint8_t  steering_cmd :8;
  		uint8_t  :8;
  		uint8_t  lamps_cmd :8;
  		uint8_t  gear_cmd :8;
  		uint8_t  throttle_cmd:8;
  		uint8_t  brake_cmd:8;
      uint8_t  :8;
      uint8_t  :8;
	} ComsCmd;

  typedef struct
  {
      uint16_t cur_speed :16;
      uint16_t cur_steering :16;
      uint8_t  test1:8;
      uint8_t  test2:8;
      uint8_t  test3:8;
      uint8_t  test4:8;
  } ComsReport;

  typedef struct
  {
      
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport70;

  typedef struct
  {
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport71;

  typedef struct
  {
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport72;

  typedef struct
  {
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport75;

  typedef struct
  {
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport76;

  typedef struct
  {
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport78;

  typedef struct
  {
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
      uint8_t  :8;
  } LwImuReport79;

  enum
  {
    ID_COMS_CMD            =      103,
    ID_COMS_REORT          =      93,
    ID_LW_IMU70_REPORT     =      70,
    ID_LW_IMU71_REPORT     =      71,
    ID_LW_IMU72_REPORT     =      72,
    ID_LW_IMU75_REPORT     =      75,
    ID_LW_IMU76_REPORT     =      76,
    ID_LW_IMU78_REPORT     =      78,
    ID_LW_IMU79_REPORT     =      79
  };

}

#endif