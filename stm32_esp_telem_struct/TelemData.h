struct attitude {
    float roll;
    float pitch;
    float yaw;
}__attribute__ ((packed));

struct acc {
    float x;
    float y;
    float z;
}__attribute__ ((packed));

struct cam_data {
    uint8_t detected;
    int16_t x;
    int16_t y;
    int16_t z_cam;
    int16_t yaw;
}__attribute__ ((packed));

struct mag {
    int16_t x;
    int16_t y;
    int16_t z;
}__attribute__ ((packed));

struct position_body {
    float x;
    float y;
    float z;
}__attribute__ ((packed));

struct velocity_body {
    float x;
    float y;
    float z;
}__attribute__ ((packed));

struct EKF {
    float roll_acc;
    float pitch_acc;

    float roll_gyro;
    float pitch_gyro;

    float roll_comp;
    float pitch_comp;

    float roll_ekf;
    float pitch_ekf;

}__attribute__ ((packed));

struct PID_telem {
    float P;
    float I;
    float D;
    float pd_roll_sat_buf;


}__attribute__ ((packed));

struct pwm {
	unsigned short w1;
	unsigned short w2;
	unsigned short w3;
	unsigned short w4;
}__attribute__ ((packed));

struct gps {
	struct position_body lla;
	struct position_body pos_ned;
	struct position_body pos_body;
	struct position_body vel_body;
}__attribute__ ((packed));

struct ch {
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
	uint16_t ch9;
}__attribute__ ((packed));

struct telem_pack {
  struct attitude attitude;
  struct attitude attitude_des;
  struct attitude attitude_rate;
  struct attitude attitude_rate_des;
  struct pwm pwm;
  struct EKF ekf;
  struct PID_telem pid_roll;
  struct PID_telem pid_pitch;
  float sonar_alt;
  float alt_thr;
  struct position_body position_body;
  struct velocity_body velocity_body;
  unsigned long time_millis;
  struct cam_data cam_data;
  struct acc acc;
  struct mag mag;
  struct gps gps;
  struct ch ch;
}__attribute__ ((packed));
