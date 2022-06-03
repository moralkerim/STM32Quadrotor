/*
 * Discrete_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Discrete".
 *
 * Model version              : 1.212
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Fri Jun  3 15:28:21 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Discrete.h"
#include "Discrete_private.h"

/* Block parameters (default storage) */
P_Discrete_T Discrete_P = {
  /* Mask Parameter: DiscretePIDController_D
   * Referenced by: '<S27>/Derivative Gain'
   */
  0.1,

  /* Mask Parameter: DiscretePIDController_InitialCo
   * Referenced by: '<S28>/Filter'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController_N
   * Referenced by: '<S36>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: DiscretePIDController_P
   * Referenced by: '<S38>/Proportional Gain'
   */
  1.0,

  /* Computed Parameter: Filter_gainval
   * Referenced by: '<S28>/Filter'
   */
  0.0025
};
