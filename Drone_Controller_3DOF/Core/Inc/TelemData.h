struct attitude {
    float roll;
    float pitch;
    float yaw;
}__attribute__ ((packed));

struct pwm {
	unsigned short w1;
	unsigned short w2;
	unsigned short w3;
	unsigned short w4;
}__attribute__ ((packed));

struct telem_pack {
  struct attitude attitude;
  struct pwm pwm;
}__attribute__ ((packed));
