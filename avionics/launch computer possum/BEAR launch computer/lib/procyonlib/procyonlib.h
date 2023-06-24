#ifndef HEADER_FILE
#define HEADER_FILE

struct datatotransmit
{
  int state;
  unsigned long uptimemillis;
  unsigned long missiontimemillis;

  float roll;
  float pitch;
  float yaw;

  float roll_rate;
  float pitch_rate;
  float yaw_rate;

  float accel_x;
  float accel_y;
  float accel_z;

  float absaccel;
  float absvel;

  float vertical_vel;
  float baro_alt;

  float batteryvolt;
};

struct datanew
{
  int command;
};


#endif