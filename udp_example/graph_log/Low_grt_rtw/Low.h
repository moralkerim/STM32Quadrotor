/*
 * Low.h
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

#ifndef RTW_HEADER_Low_h_
#define RTW_HEADER_Low_h_
#include <math.h>
#include <float.h>
#include <string.h>
#include <stddef.h>
#ifndef Low_COMMON_INCLUDES_
#define Low_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* Low_COMMON_INCLUDES_ */

#include "Low_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Probe[2];                     /* '<S2>/Probe' */
} B_Low_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S7>/Integrator' */
  int8_T Integrator_PrevResetState;    /* '<S7>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S7>/Integrator' */
} DW_Low_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExtU_Low_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T y;                            /* '<Root>/y' */
} ExtY_Low_T;

/* Parameters (default storage) */
struct P_Low_T_ {
  real_T LowPassFilterDiscreteorContinuo;
                              /* Mask Parameter: LowPassFilterDiscreteorContinuo
                               * Referenced by: '<S1>/K'
                               */
  real_T LowPassFilterDiscreteorContin_c;
                              /* Mask Parameter: LowPassFilterDiscreteorContin_c
                               * Referenced by: '<S2>/Time constant'
                               */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S5>/Constant'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S7>/Integrator'
                                        */
  real_T Integrator_UpperSat;          /* Expression: antiwindupUpperLimit
                                        * Referenced by: '<S7>/Integrator'
                                        */
  real_T Integrator_LowerSat;          /* Expression: antiwindupLowerLimit
                                        * Referenced by: '<S7>/Integrator'
                                        */
  real_T Saturation_UpperSat;          /* Expression: windupUpperLimit
                                        * Referenced by: '<S7>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: windupLowerLimit
                                        * Referenced by: '<S7>/Saturation'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Low_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_Low_T Low_P;

/* Block signals (default storage) */
extern B_Low_T Low_B;

/* Block states (default storage) */
extern DW_Low_T Low_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Low_T Low_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Low_T Low_Y;

/* Model entry point functions */
extern void Low_initialize(void);
extern void Low_step(void);
extern void Low_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Low_T *const Low_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)')    - opens subsystem quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)
 * hilite_system('quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'quadrotor_controller/Sensor Fusion'
 * '<S1>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)'
 * '<S2>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant'
 * '<S3>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Initialization'
 * '<S4>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)'
 * '<S5>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Zero'
 * '<S6>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Initialization/Init_u'
 * '<S7>'   : 'quadrotor_controller/Sensor Fusion/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
 */
#endif                                 /* RTW_HEADER_Low_h_ */
