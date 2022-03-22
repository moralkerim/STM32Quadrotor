/*
 * Discrete.c
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
  real_T rtb_IntegralGain;
  real_T tmp;
  real_T tmp_0;
  boolean_T rtb_NotEqual;

  /* Saturate: '<S42>/Saturation' incorporates:
   *  DiscreteIntegrator: '<S35>/Integrator'
   */
  if (Discrete_DW.Integrator_DSTATE > Discrete_P.DiscretePIDController_UpperSatu)
  {
    /* Outport: '<Root>/y' */
    Discrete_Y.y = Discrete_P.DiscretePIDController_UpperSatu;
  } else if (Discrete_DW.Integrator_DSTATE <
             Discrete_P.DiscretePIDController_LowerSatu) {
    /* Outport: '<Root>/y' */
    Discrete_Y.y = Discrete_P.DiscretePIDController_LowerSatu;
  } else {
    /* Outport: '<Root>/y' */
    Discrete_Y.y = Discrete_DW.Integrator_DSTATE;
  }

  /* End of Saturate: '<S42>/Saturation' */

  /* DeadZone: '<S28>/DeadZone' incorporates:
   *  DiscreteIntegrator: '<S35>/Integrator'
   */
  if (Discrete_DW.Integrator_DSTATE > Discrete_P.DiscretePIDController_UpperSatu)
  {
    rtb_IntegralGain = Discrete_DW.Integrator_DSTATE -
      Discrete_P.DiscretePIDController_UpperSatu;
  } else if (Discrete_DW.Integrator_DSTATE >=
             Discrete_P.DiscretePIDController_LowerSatu) {
    rtb_IntegralGain = 0.0;
  } else {
    rtb_IntegralGain = Discrete_DW.Integrator_DSTATE -
      Discrete_P.DiscretePIDController_LowerSatu;
  }

  /* End of DeadZone: '<S28>/DeadZone' */

  /* RelationalOperator: '<S26>/NotEqual' incorporates:
   *  DiscreteIntegrator: '<S35>/Integrator'
   *  Gain: '<S26>/ZeroGain'
   */
  rtb_NotEqual = (Discrete_P.ZeroGain_Gain * Discrete_DW.Integrator_DSTATE !=
                  rtb_IntegralGain);

  /* Signum: '<S26>/SignPreSat' */
  if (rtb_IntegralGain < 0.0) {
    rtb_IntegralGain = -1.0;
  } else if (rtb_IntegralGain > 0.0) {
    rtb_IntegralGain = 1.0;
  } else if (rtb_IntegralGain == 0.0) {
    rtb_IntegralGain = 0.0;
  } else {
    rtb_IntegralGain = (rtNaN);
  }

  /* End of Signum: '<S26>/SignPreSat' */

  /* DataTypeConversion: '<S26>/DataTypeConv1' */
  if (rtIsNaN(rtb_IntegralGain)) {
    tmp_0 = 0.0;
  } else {
    tmp_0 = fmod(rtb_IntegralGain, 256.0);
  }

  /* Gain: '<S32>/Integral Gain' incorporates:
   *  Inport: '<Root>/u'
   */
  rtb_IntegralGain = Discrete_P.DiscretePIDController_I * Discrete_U.u;

  /* Signum: '<S26>/SignPreIntegrator' */
  if (rtb_IntegralGain < 0.0) {
    /* DataTypeConversion: '<S26>/DataTypeConv2' */
    tmp = -1.0;
  } else if (rtb_IntegralGain > 0.0) {
    /* DataTypeConversion: '<S26>/DataTypeConv2' */
    tmp = 1.0;
  } else if (rtb_IntegralGain == 0.0) {
    /* DataTypeConversion: '<S26>/DataTypeConv2' */
    tmp = 0.0;
  } else {
    /* DataTypeConversion: '<S26>/DataTypeConv2' */
    tmp = (rtNaN);
  }

  /* End of Signum: '<S26>/SignPreIntegrator' */

  /* DataTypeConversion: '<S26>/DataTypeConv2' */
  if (rtIsNaN(tmp)) {
    tmp = 0.0;
  } else {
    tmp = fmod(tmp, 256.0);
  }

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Constant1'
   *  DataTypeConversion: '<S26>/DataTypeConv1'
   *  DataTypeConversion: '<S26>/DataTypeConv2'
   *  Logic: '<S26>/AND3'
   *  RelationalOperator: '<S26>/Equal1'
   */
  if (rtb_NotEqual && ((int8_T)(tmp_0 < 0.0 ? (int32_T)(int8_T)-(int8_T)(uint8_T)
        -tmp_0 : (int32_T)(int8_T)(uint8_T)tmp_0) == (tmp < 0.0 ? (int32_T)
        (int8_T)-(int8_T)(uint8_T)-tmp : (int32_T)(int8_T)(uint8_T)tmp))) {
    rtb_IntegralGain = Discrete_P.Constant1_Value;
  }

  /* End of Switch: '<S26>/Switch' */

  /* Update for DiscreteIntegrator: '<S35>/Integrator' */
  Discrete_DW.Integrator_DSTATE += Discrete_P.Integrator_gainval *
    rtb_IntegralGain;

  /* Matfile logging */
  rt_UpdateTXYLogVars(Discrete_M->rtwLogInfo, (&Discrete_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.2s, 0.0s] */
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
  Discrete_M->Timing.stepSize0 = 0.2;

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

  /* InitializeConditions for DiscreteIntegrator: '<S35>/Integrator' */
  Discrete_DW.Integrator_DSTATE = Discrete_P.DiscretePIDController_InitialCo;
}

/* Model terminate function */
void Discrete_terminate(void)
{
  /* (no terminate code required) */
}
