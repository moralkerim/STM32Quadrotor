Instructions for HMC5883L Compass:

1. Include the files in Src and Inc.
2. Define int16_t variables to store magnetic field components.
3. Define float variable to store heading angle.
4. Initialize with HMC5883L_initialize(); (Optional: Test connection with HMC5883L_testConnection();)
5. In the loop, use HMC5883L_getMagData(&mx, &my, &mz); to get magnetic field data.
6. Use HMC5883L_getHeading(mx, my, mz); to get heading angle. (Requires calibrated data).

Note: To change device configuration must change some parameters in HMC5883L_initialize();