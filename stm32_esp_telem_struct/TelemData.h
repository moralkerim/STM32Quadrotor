struct  attitude {
    float roll;
    float pitch;
    float yaw;
}__attribute__ ((packed));

struct pwm {
  unsigned short w1=1000;
  unsigned short w2=1000;
  unsigned short w3=1000;
  unsigned short w4=1000;
}__attribute__ ((packed));

struct telem_pack {
  struct attitude attitude;
  struct pwm pwm;
}__attribute__ ((packed));
