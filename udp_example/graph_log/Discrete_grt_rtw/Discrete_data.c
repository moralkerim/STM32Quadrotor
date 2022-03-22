/*
 * Discrete_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Discrete".
 *
 * Model version              : 1.0
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Tue Mar 15 15:11:25 2022
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
  /* Mask Parameter: DiscretePIDController_I
   * Referenced by: '<S32>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: DiscretePIDController_InitialCo
   * Referenced by: '<S35>/Integrator'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController_LowerSatu
   * Referenced by:
   *   '<S42>/Saturation'
   *   '<S28>/DeadZone'
   */
  -100.0,

  /* Mask Parameter: DiscretePIDController_UpperSatu
   * Referenced by:
   *   '<S42>/Saturation'
   *   '<S28>/DeadZone'
   */
  100.0,

  /* Expression: 0
   * Referenced by: '<S26>/Constant1'
   */
  0.0,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S35>/Integrator'
   */
  0.2,

  /* Expression: 0
   * Referenced by: '<S26>/ZeroGain'
   */
  0.0
};
