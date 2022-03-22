/*
 * Discrete.h
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

#ifndef RTW_HEADER_Discrete_h_
#define RTW_HEADER_Discrete_h_
#include <math.h>
#include <float.h>
#include <string.h>
#include <stddef.h>
#ifndef Discrete_COMMON_INCLUDES_
#define Discrete_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* Discrete_COMMON_INCLUDES_ */

#include "Discrete_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

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

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S35>/Integrator' */
} DW_Discrete_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExtU_Discrete_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T y;                            /* '<Root>/y' */
} ExtY_Discrete_T;

/* Parameters (default storage) */
struct P_Discrete_T_ {
  real_T DiscretePIDController_I;     /* Mask Parameter: DiscretePIDController_I
                                       * Referenced by: '<S32>/Integral Gain'
                                       */
  real_T DiscretePIDController_InitialCo;
                              /* Mask Parameter: DiscretePIDController_InitialCo
                               * Referenced by: '<S35>/Integrator'
                               */
  real_T DiscretePIDController_LowerSatu;
                              /* Mask Parameter: DiscretePIDController_LowerSatu
                               * Referenced by:
                               *   '<S42>/Saturation'
                               *   '<S28>/DeadZone'
                               */
  real_T DiscretePIDController_UpperSatu;
                              /* Mask Parameter: DiscretePIDController_UpperSatu
                               * Referenced by:
                               *   '<S42>/Saturation'
                               *   '<S28>/DeadZone'
                               */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S26>/Constant1'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S35>/Integrator'
                                        */
  real_T ZeroGain_Gain;                /* Expression: 0
                                        * Referenced by: '<S26>/ZeroGain'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Discrete_T {
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
extern P_Discrete_T Discrete_P;

/* Block states (default storage) */
extern DW_Discrete_T Discrete_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Discrete_T Discrete_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Discrete_T Discrete_Y;

/* Model entry point functions */
extern void Discrete_initialize(void);
extern void Discrete_step(void);
extern void Discrete_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Discrete_T *const Discrete_M;

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
 * hilite_system('untitled/Discrete PID Controller')    - opens subsystem untitled/Discrete PID Controller
 * hilite_system('untitled/Discrete PID Controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'untitled'
 * '<S1>'   : 'untitled/Discrete PID Controller'
 * '<S2>'   : 'untitled/Discrete PID Controller/Anti-windup'
 * '<S3>'   : 'untitled/Discrete PID Controller/D Gain'
 * '<S4>'   : 'untitled/Discrete PID Controller/Filter'
 * '<S5>'   : 'untitled/Discrete PID Controller/Filter ICs'
 * '<S6>'   : 'untitled/Discrete PID Controller/I Gain'
 * '<S7>'   : 'untitled/Discrete PID Controller/Ideal P Gain'
 * '<S8>'   : 'untitled/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'untitled/Discrete PID Controller/Integrator'
 * '<S10>'  : 'untitled/Discrete PID Controller/Integrator ICs'
 * '<S11>'  : 'untitled/Discrete PID Controller/N Copy'
 * '<S12>'  : 'untitled/Discrete PID Controller/N Gain'
 * '<S13>'  : 'untitled/Discrete PID Controller/P Copy'
 * '<S14>'  : 'untitled/Discrete PID Controller/Parallel P Gain'
 * '<S15>'  : 'untitled/Discrete PID Controller/Reset Signal'
 * '<S16>'  : 'untitled/Discrete PID Controller/Saturation'
 * '<S17>'  : 'untitled/Discrete PID Controller/Saturation Fdbk'
 * '<S18>'  : 'untitled/Discrete PID Controller/Sum'
 * '<S19>'  : 'untitled/Discrete PID Controller/Sum Fdbk'
 * '<S20>'  : 'untitled/Discrete PID Controller/Tracking Mode'
 * '<S21>'  : 'untitled/Discrete PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'untitled/Discrete PID Controller/Tsamp - Integral'
 * '<S23>'  : 'untitled/Discrete PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'untitled/Discrete PID Controller/postSat Signal'
 * '<S25>'  : 'untitled/Discrete PID Controller/preSat Signal'
 * '<S26>'  : 'untitled/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S27>'  : 'untitled/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S28>'  : 'untitled/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S29>'  : 'untitled/Discrete PID Controller/D Gain/Disabled'
 * '<S30>'  : 'untitled/Discrete PID Controller/Filter/Disabled'
 * '<S31>'  : 'untitled/Discrete PID Controller/Filter ICs/Disabled'
 * '<S32>'  : 'untitled/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S33>'  : 'untitled/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S34>'  : 'untitled/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S35>'  : 'untitled/Discrete PID Controller/Integrator/Discrete'
 * '<S36>'  : 'untitled/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S37>'  : 'untitled/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S38>'  : 'untitled/Discrete PID Controller/N Gain/Disabled'
 * '<S39>'  : 'untitled/Discrete PID Controller/P Copy/Disabled'
 * '<S40>'  : 'untitled/Discrete PID Controller/Parallel P Gain/Disabled'
 * '<S41>'  : 'untitled/Discrete PID Controller/Reset Signal/Disabled'
 * '<S42>'  : 'untitled/Discrete PID Controller/Saturation/Enabled'
 * '<S43>'  : 'untitled/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S44>'  : 'untitled/Discrete PID Controller/Sum/Passthrough_I'
 * '<S45>'  : 'untitled/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S46>'  : 'untitled/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S47>'  : 'untitled/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S48>'  : 'untitled/Discrete PID Controller/Tsamp - Integral/Passthrough'
 * '<S49>'  : 'untitled/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S50>'  : 'untitled/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S51>'  : 'untitled/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_Discrete_h_ */
