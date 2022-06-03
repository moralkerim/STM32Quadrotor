/*
 * Discrete.h
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

#ifndef RTW_HEADER_Discrete_h_
#define RTW_HEADER_Discrete_h_
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

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Filter_DSTATE;                /* '<S28>/Filter' */
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
  real_T DiscretePIDController_D;     /* Mask Parameter: DiscretePIDController_D
                                       * Referenced by: '<S27>/Derivative Gain'
                                       */
  real_T DiscretePIDController_InitialCo;
                              /* Mask Parameter: DiscretePIDController_InitialCo
                               * Referenced by: '<S28>/Filter'
                               */
  real_T DiscretePIDController_N;     /* Mask Parameter: DiscretePIDController_N
                                       * Referenced by: '<S36>/Filter Coefficient'
                                       */
  real_T DiscretePIDController_P;     /* Mask Parameter: DiscretePIDController_P
                                       * Referenced by: '<S38>/Proportional Gain'
                                       */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S28>/Filter'
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
 * hilite_system('quadrotor_controller/Altitude Controller/Discrete PID Controller')    - opens subsystem quadrotor_controller/Altitude Controller/Discrete PID Controller
 * hilite_system('quadrotor_controller/Altitude Controller/Discrete PID Controller/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'quadrotor_controller/Altitude Controller'
 * '<S1>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller'
 * '<S2>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Anti-windup'
 * '<S3>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/D Gain'
 * '<S4>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Filter'
 * '<S5>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Filter ICs'
 * '<S6>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/I Gain'
 * '<S7>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Ideal P Gain'
 * '<S8>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Integrator'
 * '<S10>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Integrator ICs'
 * '<S11>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/N Copy'
 * '<S12>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/N Gain'
 * '<S13>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/P Copy'
 * '<S14>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Parallel P Gain'
 * '<S15>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Reset Signal'
 * '<S16>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Saturation'
 * '<S17>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S18>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Sum'
 * '<S19>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Sum Fdbk'
 * '<S20>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tracking Mode'
 * '<S21>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S23>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/postSat Signal'
 * '<S25>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/preSat Signal'
 * '<S26>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Anti-windup/Disabled'
 * '<S27>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/D Gain/Internal Parameters'
 * '<S28>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S29>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S30>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/I Gain/Disabled'
 * '<S31>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S32>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S33>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Integrator/Disabled'
 * '<S34>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Integrator ICs/Disabled'
 * '<S35>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/N Copy/Disabled'
 * '<S36>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/N Gain/Internal Parameters'
 * '<S37>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S38>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S39>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S40>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S41>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S42>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Sum/Sum_PD'
 * '<S43>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S44>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S45>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S46>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tsamp - Integral/Disabled wSignal Specification'
 * '<S47>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S48>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S49>'  : 'quadrotor_controller/Altitude Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_Discrete_h_ */
