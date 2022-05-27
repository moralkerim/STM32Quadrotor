/*
 * Low.c
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

/* Block signals (default storage) */
B_Low_T Low_B;

/* Block states (default storage) */
DW_Low_T Low_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Low_T Low_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Low_T Low_Y;

/* Real-time model */
static RT_MODEL_Low_T Low_M_;
RT_MODEL_Low_T *const Low_M = &Low_M_;

/* Model step function */
void Low_step(void)
{
  real_T rtb_K;
  real_T rtb_Max;
  real_T rtb_Saturation;
  boolean_T rtb_Compare;

  /* RelationalOperator: '<S5>/Compare' incorporates:
   *  Constant: '<S2>/Time constant'
   *  Constant: '<S5>/Constant'
   *  Sum: '<S2>/Sum1'
   */
  rtb_Compare = (Low_P.LowPassFilterDiscreteorContin_c - Low_B.Probe[0] <=
                 Low_P.Constant_Value);

  /* Gain: '<S1>/K' incorporates:
   *  Inport: '<Root>/u'
   */
  rtb_K = Low_P.LowPassFilterDiscreteorContinuo * Low_U.u;

  /* DiscreteIntegrator: '<S7>/Integrator' */
  if (Low_DW.Integrator_IC_LOADING != 0) {
    Low_DW.Integrator_DSTATE = rtb_K;
    if (Low_DW.Integrator_DSTATE >= Low_P.Integrator_UpperSat) {
      Low_DW.Integrator_DSTATE = Low_P.Integrator_UpperSat;
    } else {
      if (Low_DW.Integrator_DSTATE <= Low_P.Integrator_LowerSat) {
        Low_DW.Integrator_DSTATE = Low_P.Integrator_LowerSat;
      }
    }
  }

  if (rtb_Compare || (Low_DW.Integrator_PrevResetState != 0)) {
    Low_DW.Integrator_DSTATE = rtb_K;
    if (Low_DW.Integrator_DSTATE >= Low_P.Integrator_UpperSat) {
      Low_DW.Integrator_DSTATE = Low_P.Integrator_UpperSat;
    } else {
      if (Low_DW.Integrator_DSTATE <= Low_P.Integrator_LowerSat) {
        Low_DW.Integrator_DSTATE = Low_P.Integrator_LowerSat;
      }
    }
  }

  if (Low_DW.Integrator_DSTATE >= Low_P.Integrator_UpperSat) {
    Low_DW.Integrator_DSTATE = Low_P.Integrator_UpperSat;
  } else {
    if (Low_DW.Integrator_DSTATE <= Low_P.Integrator_LowerSat) {
      Low_DW.Integrator_DSTATE = Low_P.Integrator_LowerSat;
    }
  }

  /* Saturate: '<S7>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S7>/Integrator'
   */
  if (Low_DW.Integrator_DSTATE > Low_P.Saturation_UpperSat) {
    rtb_Saturation = Low_P.Saturation_UpperSat;
  } else if (Low_DW.Integrator_DSTATE < Low_P.Saturation_LowerSat) {
    rtb_Saturation = Low_P.Saturation_LowerSat;
  } else {
    rtb_Saturation = Low_DW.Integrator_DSTATE;
  }

  /* End of Saturate: '<S7>/Saturation' */

  /* Outport: '<Root>/y' */
  Low_Y.y = rtb_Saturation;

  /* MinMax: '<S2>/Max' incorporates:
   *  Constant: '<S2>/Time constant'
   */
  rtb_Max = fmax(Low_B.Probe[0], Low_P.LowPassFilterDiscreteorContin_c);

  /* Update for DiscreteIntegrator: '<S7>/Integrator' incorporates:
   *  Fcn: '<S2>/Avoid Divide by Zero'
   *  Product: '<S1>/1//T'
   *  Sum: '<S1>/Sum1'
   */
  Low_DW.Integrator_IC_LOADING = 0U;
  Low_DW.Integrator_DSTATE += 1.0 / ((real_T)(rtb_Max == 0.0) *
    2.2204460492503131e-16 + rtb_Max) * (rtb_K - rtb_Saturation) *
    Low_P.Integrator_gainval;
  if (Low_DW.Integrator_DSTATE >= Low_P.Integrator_UpperSat) {
    Low_DW.Integrator_DSTATE = Low_P.Integrator_UpperSat;
  } else {
    if (Low_DW.Integrator_DSTATE <= Low_P.Integrator_LowerSat) {
      Low_DW.Integrator_DSTATE = Low_P.Integrator_LowerSat;
    }
  }

  Low_DW.Integrator_PrevResetState = (int8_T)rtb_Compare;

  /* End of Update for DiscreteIntegrator: '<S7>/Integrator' */

  /* Matfile logging */
  rt_UpdateTXYLogVars(Low_M->rtwLogInfo, (&Low_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0025s, 0.0s] */
    if ((rtmGetTFinal(Low_M)!=-1) &&
        !((rtmGetTFinal(Low_M)-Low_M->Timing.taskTime0) >
          Low_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Low_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Low_M->Timing.clockTick0)) {
    ++Low_M->Timing.clockTickH0;
  }

  Low_M->Timing.taskTime0 = Low_M->Timing.clockTick0 * Low_M->Timing.stepSize0 +
    Low_M->Timing.clockTickH0 * Low_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Low_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  Low_P.Integrator_UpperSat = rtInf;
  Low_P.Integrator_LowerSat = rtMinusInf;
  Low_P.Saturation_UpperSat = rtInf;
  Low_P.Saturation_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)Low_M, 0,
                sizeof(RT_MODEL_Low_T));
  rtmSetTFinal(Low_M, 10.0);
  Low_M->Timing.stepSize0 = 0.0025;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    Low_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Low_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Low_M->rtwLogInfo, (NULL));
    rtliSetLogT(Low_M->rtwLogInfo, "tout");
    rtliSetLogX(Low_M->rtwLogInfo, "");
    rtliSetLogXFinal(Low_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Low_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Low_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Low_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Low_M->rtwLogInfo, 1);
    rtliSetLogY(Low_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Low_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Low_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &Low_B), 0,
                sizeof(B_Low_T));

  /* states (dwork) */
  (void) memset((void *)&Low_DW, 0,
                sizeof(DW_Low_T));

  /* external inputs */
  Low_U.u = 0.0;

  /* external outputs */
  Low_Y.y = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Low_M->rtwLogInfo, 0.0, rtmGetTFinal(Low_M),
    Low_M->Timing.stepSize0, (&rtmGetErrorStatus(Low_M)));

  /* Start for Probe: '<S2>/Probe' */
  Low_B.Probe[0] = 0.0025;
  Low_B.Probe[1] = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S7>/Integrator' */
  Low_DW.Integrator_PrevResetState = 0;
  Low_DW.Integrator_IC_LOADING = 1U;
}

/* Model terminate function */
void Low_terminate(void)
{
  /* (no terminate code required) */
}
