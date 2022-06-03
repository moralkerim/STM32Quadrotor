/*
 * Discrete.c
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

/* Block states (default storage) */
DW_Discrete_T Discrete_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Discrete_T Discrete_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Discrete_T Discrete_Y;

/* Real-time model */
static RT_MODEL_Discrete_T Discrete_M_;
RT_MODEL_Discrete_T *const Discrete_M = &Discrete_M_;

/* Model step function */
void Discrete_step(void)
{
  real_T rtb_FilterCoefficient;

  /* Gain: '<S36>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter'
   *  Gain: '<S27>/Derivative Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S28>/SumD'
   */
  rtb_FilterCoefficient = (Discrete_P.DiscretePIDController_D * Discrete_U.u -
    Discrete_DW.Filter_DSTATE) * Discrete_P.DiscretePIDController_N;

  /* Outport: '<Root>/y' incorporates:
   *  Gain: '<S38>/Proportional Gain'
   *  Inport: '<Root>/u'
   *  Sum: '<S42>/Sum'
   */
  Discrete_Y.y = Discrete_P.DiscretePIDController_P * Discrete_U.u +
    rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S28>/Filter' */
  Discrete_DW.Filter_DSTATE += Discrete_P.Filter_gainval * rtb_FilterCoefficient;

  /* Matfile logging */
  rt_UpdateTXYLogVars(Discrete_M->rtwLogInfo, (&Discrete_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0025s, 0.0s] */
    if ((rtmGetTFinal(Discrete_M)!=-1) &&
        !((rtmGetTFinal(Discrete_M)-Discrete_M->Timing.taskTime0) >
          Discrete_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Discrete_M, "Simulation finished");
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
  if (!(++Discrete_M->Timing.clockTick0)) {
    ++Discrete_M->Timing.clockTickH0;
  }

  Discrete_M->Timing.taskTime0 = Discrete_M->Timing.clockTick0 *
    Discrete_M->Timing.stepSize0 + Discrete_M->Timing.clockTickH0 *
    Discrete_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Discrete_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Discrete_M, 0,
                sizeof(RT_MODEL_Discrete_T));
  rtmSetTFinal(Discrete_M, 10.0);
  Discrete_M->Timing.stepSize0 = 0.0025;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    Discrete_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Discrete_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Discrete_M->rtwLogInfo, (NULL));
    rtliSetLogT(Discrete_M->rtwLogInfo, "tout");
    rtliSetLogX(Discrete_M->rtwLogInfo, "");
    rtliSetLogXFinal(Discrete_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Discrete_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Discrete_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Discrete_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Discrete_M->rtwLogInfo, 1);
    rtliSetLogY(Discrete_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Discrete_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Discrete_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&Discrete_DW, 0,
                sizeof(DW_Discrete_T));

  /* external inputs */
  Discrete_U.u = 0.0;

  /* external outputs */
  Discrete_Y.y = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Discrete_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Discrete_M), Discrete_M->Timing.stepSize0, (&rtmGetErrorStatus(Discrete_M)));

  /* InitializeConditions for DiscreteIntegrator: '<S28>/Filter' */
  Discrete_DW.Filter_DSTATE = Discrete_P.DiscretePIDController_InitialCo;
}

/* Model terminate function */
void Discrete_terminate(void)
{
  /* (no terminate code required) */
}
