/*
 * fir.c
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
#include "rtwtypes.h"
#include "fir_private.h"
#include <string.h>
#include "rt_nonfinite.h"

/* Block states (default storage) */
DW_fir_T fir_DW;

/* External inputs (root inport signals with default storage) */
ExtU_fir_T fir_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_fir_T fir_Y;

/* Real-time model */
static RT_MODEL_fir_T fir_M_;
RT_MODEL_fir_T *const fir_M = &fir_M_;

/* Model step function */
void fir_step(void)
{
  real_T denAccum;
  int32_T denIdx;
  int32_T j;

  /* DiscreteFilter: '<S1>/IIR_IMUgyro_r' incorporates:
   *  Inport: '<Root>/yaw_gyro'
   */
  denAccum = fir_U.yaw_gyro;
  denIdx = 1;
  for (j = 0; j < 5; j++) {
    denAccum -= fir_P.IIR_IMUgyro_r_DenCoef[denIdx] *
      fir_DW.IIR_IMUgyro_r_states[j];
    denIdx++;
  }

  fir_DW.IIR_IMUgyro_r_tmp = denAccum / fir_P.IIR_IMUgyro_r_DenCoef[0];
  denAccum = fir_P.IIR_IMUgyro_r_NumCoef[0] * fir_DW.IIR_IMUgyro_r_tmp;
  denIdx = 1;
  for (j = 0; j < 5; j++) {
    denAccum += fir_P.IIR_IMUgyro_r_NumCoef[denIdx] *
      fir_DW.IIR_IMUgyro_r_states[j];
    denIdx++;
  }

  /* Outport: '<Root>/yaw_rate' incorporates:
   *  DiscreteFilter: '<S1>/IIR_IMUgyro_r'
   */
  fir_Y.yaw_rate = denAccum;

  /* Update for DiscreteFilter: '<S1>/IIR_IMUgyro_r' */
  fir_DW.IIR_IMUgyro_r_states[4] = fir_DW.IIR_IMUgyro_r_states[3];
  fir_DW.IIR_IMUgyro_r_states[3] = fir_DW.IIR_IMUgyro_r_states[2];
  fir_DW.IIR_IMUgyro_r_states[2] = fir_DW.IIR_IMUgyro_r_states[1];
  fir_DW.IIR_IMUgyro_r_states[1] = fir_DW.IIR_IMUgyro_r_states[0];
  fir_DW.IIR_IMUgyro_r_states[0] = fir_DW.IIR_IMUgyro_r_tmp;

  /* Matfile logging */
  rt_UpdateTXYLogVars(fir_M->rtwLogInfo, (&fir_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.005s, 0.0s] */
    if ((rtmGetTFinal(fir_M)!=-1) &&
        !((rtmGetTFinal(fir_M)-fir_M->Timing.taskTime0) >
          fir_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(fir_M, "Simulation finished");
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
  if (!(++fir_M->Timing.clockTick0)) {
    ++fir_M->Timing.clockTickH0;
  }

  fir_M->Timing.taskTime0 = fir_M->Timing.clockTick0 * fir_M->Timing.stepSize0 +
    fir_M->Timing.clockTickH0 * fir_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void fir_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)fir_M, 0,
                sizeof(RT_MODEL_fir_T));
  rtmSetTFinal(fir_M, 10.0);
  fir_M->Timing.stepSize0 = 0.005;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    fir_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(fir_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(fir_M->rtwLogInfo, (NULL));
    rtliSetLogT(fir_M->rtwLogInfo, "tout");
    rtliSetLogX(fir_M->rtwLogInfo, "");
    rtliSetLogXFinal(fir_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(fir_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(fir_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(fir_M->rtwLogInfo, 0);
    rtliSetLogDecimation(fir_M->rtwLogInfo, 1);
    rtliSetLogY(fir_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(fir_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(fir_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&fir_DW, 0,
                sizeof(DW_fir_T));

  /* external inputs */
  fir_U.yaw_gyro = 0.0;

  /* external outputs */
  fir_Y.yaw_rate = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(fir_M->rtwLogInfo, 0.0, rtmGetTFinal(fir_M),
    fir_M->Timing.stepSize0, (&rtmGetErrorStatus(fir_M)));

  {
    int32_T i;

    /* InitializeConditions for DiscreteFilter: '<S1>/IIR_IMUgyro_r' */
    for (i = 0; i < 5; i++) {
      fir_DW.IIR_IMUgyro_r_states[i] = fir_P.IIR_IMUgyro_r_InitialStates;
    }

    /* End of InitializeConditions for DiscreteFilter: '<S1>/IIR_IMUgyro_r' */
  }
}

/* Model terminate function */
void fir_terminate(void)
{
  /* (no terminate code required) */
}
