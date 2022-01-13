struct attitude {
    char roll[6];
    char pitch[6];
    char yaw[6];
};

struct pwm {
  char w1[4];
  char w2[4];
  char w3[4];
  char w4[4];
};

struct telem_pack {
  struct attitude attitude;
  struct pwm pwm;
};