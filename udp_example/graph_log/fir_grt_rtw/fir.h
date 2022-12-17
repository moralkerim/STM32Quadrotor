/*
 * fir.h
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

#ifndef RTW_HEADER_fir_h_
#define RTW_HEADER_fir_h_
#ifndef fir_COMMON_INCLUDES_
#define fir_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* fir_COMMON_INCLUDES_ */

#include "fir_types.h"
#include <float.h>
#include <string.h>
#include <stddef.h>
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
  real_T IIR_IMUgyro_r_states[5];      /* '<S1>/IIR_IMUgyro_r' */
  real_T IIR_IMUgyro_r_tmp;            /* '<S1>/IIR_IMUgyro_r' */
} DW_fir_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T yaw_gyro;                     /* '<Root>/yaw_gyro' */
} ExtU_fir_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T yaw_rate;                     /* '<Root>/yaw_rate' */
} ExtY_fir_T;

/* Parameters (default storage) */
struct P_fir_T_ {
  real_T IIR_IMUgyro_r_NumCoef[6];    /* Expression: Estimator.IMU.filterGyroNum
                                       * Referenced by: '<S1>/IIR_IMUgyro_r'
                                       */
  real_T IIR_IMUgyro_r_DenCoef[6];    /* Expression: Estimator.IMU.filterGyroDen
                                       * Referenced by: '<S1>/IIR_IMUgyro_r'
                                       */
  real_T IIR_IMUgyro_r_InitialStates;  /* Expression: 0
                                        * Referenced by: '<S1>/IIR_IMUgyro_r'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_fir_T {
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
extern P_fir_T fir_P;

/* Block states (default storage) */
extern DW_fir_T fir_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_fir_T fir_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_fir_T fir_Y;

/* Model entry point functions */
extern void fir_initialize(void);
extern void fir_step(void);
extern void fir_terminate(void);

/* Real-time Model object */
extern RT_MODEL_fir_T *const fir_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'fir'
 * '<S1>'   : 'fir/YAW_FIR'
 */
#endif                                 /* RTW_HEADER_fir_h_ */
