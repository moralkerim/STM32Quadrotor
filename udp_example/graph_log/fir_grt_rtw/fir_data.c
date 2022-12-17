/*
 * fir_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "fir".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Fri Dec 16 19:54:30 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "fir.h"

/* Block parameters (default storage) */
P_fir_T fir_P = {
  /* Expression: Estimator.IMU.filterGyroNum
   * Referenced by: '<S1>/IIR_IMUgyro_r'
   */
  { 0.28212412246252078, 1.272539291716861, 2.4208439774454464,
    2.4208439774454469, 1.2725392917168616, 0.28212412246252111 },

  /* Expression: Estimator.IMU.filterGyroDen
   * Referenced by: '<S1>/IIR_IMUgyro_r'
   */
  { 1.0, 2.2287149173647669, 2.5244618916938624, 1.5772531712757027,
    0.541022406829818, 0.079562396085500989 },

  /* Expression: 0
   * Referenced by: '<S1>/IIR_IMUgyro_r'
   */
  0.0
};
