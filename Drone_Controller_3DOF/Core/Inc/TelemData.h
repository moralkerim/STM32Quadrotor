struct attitude {
    float roll;
    float pitch;
    float yaw;
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

struct telem_pack {
  struct attitude attitude;
  struct attitude attitude_des;
  struct attitude attitude_rate;
  struct attitude attitude_rate_des;
  struct pwm pwm;
  struct EKF ekf;
  struct PID_telem pid_roll;
  struct PID_telem pid_pitch;
  float baro_alt;
  float sonar_alt;
  float sonar_vel;
  unsigned long time_millis;
}__attribute__ ((packed));
