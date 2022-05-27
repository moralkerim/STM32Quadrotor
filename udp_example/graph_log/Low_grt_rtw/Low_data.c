/*
 * Low_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Low".
 *
 * Model version              : 1.188
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Thu May 26 16:51:58 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Low.h"
#include "Low_private.h"

/* Block parameters (default storage) */
P_Low_T Low_P = {
  /* Mask Parameter: LowPassFilterDiscreteorContinuo
   * Referenced by: '<S1>/K'
   */
  1.0,

  /* Mask Parameter: LowPassFilterDiscreteorContin_c
   * Referenced by: '<S2>/Time constant'
   */
  0.031830988618379068,

  /* Expression: 0
   * Referenced by: '<S5>/Constant'
   */
  0.0,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S7>/Integrator'
   */
  0.0025,

  /* Expression: antiwindupUpperLimit
   * Referenced by: '<S7>/Integrator'
   */
  0.0,

  /* Expression: antiwindupLowerLimit
   * Referenced by: '<S7>/Integrator'
   */
  0.0,

  /* Expression: windupUpperLimit
   * Referenced by: '<S7>/Saturation'
   */
  0.0,

  /* Expression: windupLowerLimit
   * Referenced by: '<S7>/Saturation'
   */
  0.0
};
