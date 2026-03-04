/*
 * qs3_pen_ss_model.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "qs3_pen_ss_model".
 *
 * Model version              : 7.3
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Thu Feb  5 18:21:38 2026
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "qs3_pen_ss_model.h"
#include "rtwtypes.h"
#include <math.h>
#include <emmintrin.h>
#include "qs3_pen_ss_model_private.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "qs3_pen_ss_model_dt.h"

const real_T qs3_pen_ss_model_RGND = 0.0;/* real_T ground */

/* Block signals (default storage) */
B_qs3_pen_ss_model_T qs3_pen_ss_model_B;

/* Continuous states */
X_qs3_pen_ss_model_T qs3_pen_ss_model_X;

/* Disabled State Vector */
XDis_qs3_pen_ss_model_T qs3_pen_ss_model_XDis;

/* Block states (default storage) */
DW_qs3_pen_ss_model_T qs3_pen_ss_model_DW;

/* Real-time model */
static RT_MODEL_qs3_pen_ss_model_T qs3_pen_ss_model_M_;
RT_MODEL_qs3_pen_ss_model_T *const qs3_pen_ss_model_M = &qs3_pen_ss_model_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 4;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  qs3_pen_ss_model_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  qs3_pen_ss_model_output();
  qs3_pen_ss_model_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  qs3_pen_ss_model_output();
  qs3_pen_ss_model_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  qs3_pen_ss_model_output();
  qs3_pen_ss_model_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void qs3_pen_ss_model_output(void)
{
  /* local block i/o variables */
  real_T rtb_motorcurrent;
  real_T rtb_HILReadTimebase_o5;
  real_T rtb_HILReadTimebase_o6;
  boolean_T rtb_HILReadTimebase_o4;
  real_T StateSpaceModel_CSTATE;
  real_T StateSpaceModel_CSTATE_0;
  real_T rtb_HILReadTimebase_o3;
  real_T rtb_encoder;
  real_T temp;
  int_T iy;
  boolean_T tmp;
  if (rtmIsMajorTimeStep(qs3_pen_ss_model_M)) {
    /* set solver stop time */
    if (!(qs3_pen_ss_model_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&qs3_pen_ss_model_M->solverInfo,
                            ((qs3_pen_ss_model_M->Timing.clockTickH0 + 1) *
        qs3_pen_ss_model_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&qs3_pen_ss_model_M->solverInfo,
                            ((qs3_pen_ss_model_M->Timing.clockTick0 + 1) *
        qs3_pen_ss_model_M->Timing.stepSize0 +
        qs3_pen_ss_model_M->Timing.clockTickH0 *
        qs3_pen_ss_model_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(qs3_pen_ss_model_M)) {
    qs3_pen_ss_model_M->Timing.t[0] = rtsiGetT(&qs3_pen_ss_model_M->solverInfo);
  }

  tmp = rtmIsMajorTimeStep(qs3_pen_ss_model_M);
  if (tmp) {
    /* S-Function (hil_read_timebase_block): '<S1>/HIL Read Timebase' */

    /* S-Function Block: qs3_pen_ss_model/Qube-Servo 3 - IO (QAL)/HIL Read Timebase (hil_read_timebase_block) */
    {
      t_error result;
      result = hil_task_read(qs3_pen_ss_model_DW.HILReadTimebase_Task, 1,
        &qs3_pen_ss_model_DW.HILReadTimebase_AnalogBuffer,
        &qs3_pen_ss_model_DW.HILReadTimebase_EncoderBuffer[0],
        &qs3_pen_ss_model_DW.HILReadTimebase_DigitalBuffer,
        &qs3_pen_ss_model_DW.HILReadTimebase_OtherBuffer[0]
        );
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
      } else {
        rtb_motorcurrent = qs3_pen_ss_model_DW.HILReadTimebase_AnalogBuffer;
        rtb_encoder = qs3_pen_ss_model_DW.HILReadTimebase_EncoderBuffer[0];
        rtb_HILReadTimebase_o3 =
          qs3_pen_ss_model_DW.HILReadTimebase_EncoderBuffer[1];
        rtb_HILReadTimebase_o4 =
          qs3_pen_ss_model_DW.HILReadTimebase_DigitalBuffer;
        rtb_HILReadTimebase_o5 =
          qs3_pen_ss_model_DW.HILReadTimebase_OtherBuffer[0];
        rtb_HILReadTimebase_o6 =
          qs3_pen_ss_model_DW.HILReadTimebase_OtherBuffer[1];
      }
    }
  }

  /* SignalGenerator: '<Root>/Signal Generator' */
  temp = qs3_pen_ss_model_P.SignalGenerator_Frequency *
    qs3_pen_ss_model_M->Timing.t[0];
  if (temp - floor(temp) >= 0.5) {
    temp = qs3_pen_ss_model_P.SignalGenerator_Amplitude;
  } else {
    temp = -qs3_pen_ss_model_P.SignalGenerator_Amplitude;
  }

  /* Gain: '<Root>/Amplitude (V)' incorporates:
   *  SignalGenerator: '<Root>/Signal Generator'
   */
  qs3_pen_ss_model_B.AmplitudeV = qs3_pen_ss_model_P.AmplitudeV_Gain * temp;

  /* Gain: '<Root>/For +ve CCW  motion' */
  qs3_pen_ss_model_B.ForveCCWmotion = qs3_pen_ss_model_P.ForveCCWmotion_Gain *
    qs3_pen_ss_model_B.AmplitudeV;
  if (tmp) {
    /* S-Function (hil_write_block): '<S1>/HIL Write' */

    /* S-Function Block: qs3_pen_ss_model/Qube-Servo 3 - IO (QAL)/HIL Write (hil_write_block) */
    {
      t_error result;
      result = hil_write(qs3_pen_ss_model_DW.HILInitialize_Card,
                         &qs3_pen_ss_model_P.HILWrite_analog_channels, 1U,
                         NULL, 0U,
                         NULL, 0U,
                         &qs3_pen_ss_model_P.HILWrite_other_channels, 1U,
                         &qs3_pen_ss_model_B.ForveCCWmotion,
                         NULL,
                         NULL,
                         ((const real_T*) &qs3_pen_ss_model_RGND)
                         );
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
      }
    }

    /* Gain: '<Root>/Base: counts to rad' */
    qs3_pen_ss_model_B.plant = qs3_pen_ss_model_P.Basecountstorad_Gain *
      rtb_encoder;
  }

  /* StateSpace: '<Root>/State-Space Model' */
  rtb_encoder = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[1];
  temp = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[0];
  StateSpaceModel_CSTATE = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[2];
  StateSpaceModel_CSTATE_0 = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[3];
  for (iy = 0; iy <= 2; iy += 2) {
    /* StateSpace: '<Root>/State-Space Model' */
    _mm_storeu_pd(&qs3_pen_ss_model_B.StateSpaceModel[iy], _mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&qs3_pen_ss_model_P.C[iy + 4]),
      _mm_set1_pd(rtb_encoder)), _mm_mul_pd(_mm_loadu_pd
      (&qs3_pen_ss_model_P.C[iy]), _mm_set1_pd(temp))), _mm_mul_pd(_mm_loadu_pd(
      &qs3_pen_ss_model_P.C[iy + 8]), _mm_set1_pd(StateSpaceModel_CSTATE))),
      _mm_mul_pd(_mm_loadu_pd(&qs3_pen_ss_model_P.C[iy + 12]), _mm_set1_pd
                 (StateSpaceModel_CSTATE_0))));
  }

  /* End of StateSpace: '<Root>/State-Space Model' */
  if (tmp) {
    /* Gain: '<Root>/Pendulum: counts to rad' */
    qs3_pen_ss_model_B.plant_e = qs3_pen_ss_model_P.Pendulumcountstorad_Gain *
      rtb_HILReadTimebase_o3;
  }
}

/* Model update function */
void qs3_pen_ss_model_update(void)
{
  if (rtmIsMajorTimeStep(qs3_pen_ss_model_M)) {
    rt_ertODEUpdateContinuousStates(&qs3_pen_ss_model_M->solverInfo);
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
  if (!(++qs3_pen_ss_model_M->Timing.clockTick0)) {
    ++qs3_pen_ss_model_M->Timing.clockTickH0;
  }

  qs3_pen_ss_model_M->Timing.t[0] = rtsiGetSolverStopTime
    (&qs3_pen_ss_model_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.002s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++qs3_pen_ss_model_M->Timing.clockTick1)) {
      ++qs3_pen_ss_model_M->Timing.clockTickH1;
    }

    qs3_pen_ss_model_M->Timing.t[1] = qs3_pen_ss_model_M->Timing.clockTick1 *
      qs3_pen_ss_model_M->Timing.stepSize1 +
      qs3_pen_ss_model_M->Timing.clockTickH1 *
      qs3_pen_ss_model_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void qs3_pen_ss_model_derivatives(void)
{
  XDot_qs3_pen_ss_model_T *_rtXdot;
  real_T StateSpaceModel_CSTATE;
  real_T StateSpaceModel_CSTATE_0;
  real_T StateSpaceModel_CSTATE_1;
  real_T StateSpaceModel_CSTATE_2;
  int_T is;
  _rtXdot = ((XDot_qs3_pen_ss_model_T *) qs3_pen_ss_model_M->derivs);

  /* Derivatives for StateSpace: '<Root>/State-Space Model' */
  StateSpaceModel_CSTATE = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[1];
  StateSpaceModel_CSTATE_0 = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[0];
  StateSpaceModel_CSTATE_1 = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[2];
  StateSpaceModel_CSTATE_2 = qs3_pen_ss_model_X.StateSpaceModel_CSTATE[3];
  for (is = 0; is <= 2; is += 2) {
    _mm_storeu_pd(&_rtXdot->StateSpaceModel_CSTATE[is], _mm_add_pd(_mm_add_pd
      (_mm_add_pd(_mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&qs3_pen_ss_model_P.A[is +
      4]), _mm_set1_pd(StateSpaceModel_CSTATE)), _mm_mul_pd(_mm_loadu_pd
      (&qs3_pen_ss_model_P.A[is]), _mm_set1_pd(StateSpaceModel_CSTATE_0))),
                  _mm_mul_pd(_mm_loadu_pd(&qs3_pen_ss_model_P.A[is + 8]),
      _mm_set1_pd(StateSpaceModel_CSTATE_1))), _mm_mul_pd(_mm_loadu_pd
      (&qs3_pen_ss_model_P.A[is + 12]), _mm_set1_pd(StateSpaceModel_CSTATE_2))),
      _mm_mul_pd(_mm_loadu_pd(&qs3_pen_ss_model_P.B[is]), _mm_set1_pd
                 (qs3_pen_ss_model_B.AmplitudeV))));
  }

  /* End of Derivatives for StateSpace: '<Root>/State-Space Model' */
}

/* Model initialize function */
void qs3_pen_ss_model_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<S1>/HIL Initialize' */

  /* S-Function Block: qs3_pen_ss_model/Qube-Servo 3 - IO (QAL)/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("qube_servo3_usb", "0",
                      &qs3_pen_ss_model_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options
      (qs3_pen_ss_model_DW.HILInitialize_Card,
       "deadband_compensation=0.3;pwm_en=0;enc0_velocity=3.0;enc1_velocity=3.0;min_diode_compensation=0.3;max_diode_compensation=1.5",
       125);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(qs3_pen_ss_model_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
      return;
    }

    if ((qs3_pen_ss_model_P.HILInitialize_AIPStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_AIPEnter && is_switching)) {
      result = hil_set_analog_input_ranges
        (qs3_pen_ss_model_DW.HILInitialize_Card,
         &qs3_pen_ss_model_P.HILInitialize_AIChannels, 1U,
         &qs3_pen_ss_model_P.HILInitialize_AILow,
         &qs3_pen_ss_model_P.HILInitialize_AIHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if ((qs3_pen_ss_model_P.HILInitialize_AOPStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_AOPEnter && is_switching)) {
      result = hil_set_analog_output_ranges
        (qs3_pen_ss_model_DW.HILInitialize_Card,
         &qs3_pen_ss_model_P.HILInitialize_AOChannels, 1U,
         &qs3_pen_ss_model_P.HILInitialize_AOLow,
         &qs3_pen_ss_model_P.HILInitialize_AOHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if ((qs3_pen_ss_model_P.HILInitialize_AOStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_AOEnter && is_switching)) {
      result = hil_write_analog(qs3_pen_ss_model_DW.HILInitialize_Card,
        &qs3_pen_ss_model_P.HILInitialize_AOChannels, 1U,
        &qs3_pen_ss_model_P.HILInitialize_AOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if (qs3_pen_ss_model_P.HILInitialize_AOReset) {
      result = hil_watchdog_set_analog_expiration_state
        (qs3_pen_ss_model_DW.HILInitialize_Card,
         &qs3_pen_ss_model_P.HILInitialize_AOChannels, 1U,
         &qs3_pen_ss_model_P.HILInitialize_AOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    result = hil_set_digital_directions(qs3_pen_ss_model_DW.HILInitialize_Card,
      NULL, 0U, &qs3_pen_ss_model_P.HILInitialize_DOChannels, 1U);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
      return;
    }

    if ((qs3_pen_ss_model_P.HILInitialize_DOStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_DOEnter && is_switching)) {
      result = hil_write_digital(qs3_pen_ss_model_DW.HILInitialize_Card,
        &qs3_pen_ss_model_P.HILInitialize_DOChannels, 1U, (t_boolean *)
        &qs3_pen_ss_model_P.HILInitialize_DOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if (qs3_pen_ss_model_P.HILInitialize_DOReset) {
      result = hil_watchdog_set_digital_expiration_state
        (qs3_pen_ss_model_DW.HILInitialize_Card,
         &qs3_pen_ss_model_P.HILInitialize_DOChannels, 1U, (const
          t_digital_state *) &qs3_pen_ss_model_P.HILInitialize_DOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if ((qs3_pen_ss_model_P.HILInitialize_EIPStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_EIPEnter && is_switching)) {
      qs3_pen_ss_model_DW.HILInitialize_QuadratureModes[0] =
        qs3_pen_ss_model_P.HILInitialize_EIQuadrature;
      qs3_pen_ss_model_DW.HILInitialize_QuadratureModes[1] =
        qs3_pen_ss_model_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (qs3_pen_ss_model_DW.HILInitialize_Card,
         qs3_pen_ss_model_P.HILInitialize_EIChannels, 2U,
         (t_encoder_quadrature_mode *)
         &qs3_pen_ss_model_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if ((qs3_pen_ss_model_P.HILInitialize_EIStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_EIEnter && is_switching)) {
      qs3_pen_ss_model_DW.HILInitialize_InitialEICounts[0] =
        qs3_pen_ss_model_P.HILInitialize_EIInitial;
      qs3_pen_ss_model_DW.HILInitialize_InitialEICounts[1] =
        qs3_pen_ss_model_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(qs3_pen_ss_model_DW.HILInitialize_Card,
        qs3_pen_ss_model_P.HILInitialize_EIChannels, 2U,
        &qs3_pen_ss_model_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if ((qs3_pen_ss_model_P.HILInitialize_OOStart && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_OOEnter && is_switching)) {
      result = hil_write_other(qs3_pen_ss_model_DW.HILInitialize_Card,
        qs3_pen_ss_model_P.HILInitialize_OOChannels, 3U,
        qs3_pen_ss_model_P.HILInitialize_OOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }

    if (qs3_pen_ss_model_P.HILInitialize_OOReset) {
      result = hil_watchdog_set_other_expiration_state
        (qs3_pen_ss_model_DW.HILInitialize_Card,
         qs3_pen_ss_model_P.HILInitialize_OOChannels, 3U,
         qs3_pen_ss_model_P.HILInitialize_OOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_timebase_block): '<S1>/HIL Read Timebase' */

  /* S-Function Block: qs3_pen_ss_model/Qube-Servo 3 - IO (QAL)/HIL Read Timebase (hil_read_timebase_block) */
  {
    t_error result;
    result = hil_task_create_reader(qs3_pen_ss_model_DW.HILInitialize_Card,
      qs3_pen_ss_model_P.HILReadTimebase_SamplesInBuffer,
      &qs3_pen_ss_model_P.HILReadTimebase_AnalogChannels, 1U,
      qs3_pen_ss_model_P.HILReadTimebase_EncoderChannels, 2U,
      &qs3_pen_ss_model_P.HILReadTimebase_DigitalChannels, 1U,
      qs3_pen_ss_model_P.HILReadTimebase_OtherChannels, 2U,
      &qs3_pen_ss_model_DW.HILReadTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (qs3_pen_ss_model_DW.HILReadTimebase_Task, (t_buffer_overflow_mode)
         (qs3_pen_ss_model_P.HILReadTimebase_OverflowMode - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
    }
  }

  /* InitializeConditions for StateSpace: '<Root>/State-Space Model' */
  qs3_pen_ss_model_X.StateSpaceModel_CSTATE[0] =
    qs3_pen_ss_model_P.StateSpaceModel_InitialConditio[0];
  qs3_pen_ss_model_X.StateSpaceModel_CSTATE[1] =
    qs3_pen_ss_model_P.StateSpaceModel_InitialConditio[1];
  qs3_pen_ss_model_X.StateSpaceModel_CSTATE[2] =
    qs3_pen_ss_model_P.StateSpaceModel_InitialConditio[2];
  qs3_pen_ss_model_X.StateSpaceModel_CSTATE[3] =
    qs3_pen_ss_model_P.StateSpaceModel_InitialConditio[3];
}

/* Model terminate function */
void qs3_pen_ss_model_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<S1>/HIL Initialize' */

  /* S-Function Block: qs3_pen_ss_model/Qube-Servo 3 - IO (QAL)/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_digital_outputs = 0;
    t_uint32 num_final_other_outputs = 0;
    hil_task_stop_all(qs3_pen_ss_model_DW.HILInitialize_Card);
    hil_monitor_stop_all(qs3_pen_ss_model_DW.HILInitialize_Card);
    is_switching = false;
    if ((qs3_pen_ss_model_P.HILInitialize_AOTerminate && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_AOExit && is_switching)) {
      num_final_analog_outputs = 1U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((qs3_pen_ss_model_P.HILInitialize_DOTerminate && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_DOExit && is_switching)) {
      num_final_digital_outputs = 1U;
    } else {
      num_final_digital_outputs = 0;
    }

    if ((qs3_pen_ss_model_P.HILInitialize_OOTerminate && !is_switching) ||
        (qs3_pen_ss_model_P.HILInitialize_OOExit && is_switching)) {
      num_final_other_outputs = 3U;
    } else {
      num_final_other_outputs = 0;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_digital_outputs > 0
        || num_final_other_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(qs3_pen_ss_model_DW.HILInitialize_Card
                         , &qs3_pen_ss_model_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , NULL, 0
                         , &qs3_pen_ss_model_P.HILInitialize_DOChannels,
                         num_final_digital_outputs
                         , qs3_pen_ss_model_P.HILInitialize_OOChannels,
                         num_final_other_outputs
                         , &qs3_pen_ss_model_P.HILInitialize_AOFinal
                         , NULL
                         , (t_boolean *)
                         &qs3_pen_ss_model_P.HILInitialize_DOFinal
                         , qs3_pen_ss_model_P.HILInitialize_OOFinal
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog(qs3_pen_ss_model_DW.HILInitialize_Card,
            &qs3_pen_ss_model_P.HILInitialize_AOChannels,
            num_final_analog_outputs, &qs3_pen_ss_model_P.HILInitialize_AOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_digital_outputs > 0) {
          local_result = hil_write_digital
            (qs3_pen_ss_model_DW.HILInitialize_Card,
             &qs3_pen_ss_model_P.HILInitialize_DOChannels,
             num_final_digital_outputs, (t_boolean *)
             &qs3_pen_ss_model_P.HILInitialize_DOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_other_outputs > 0) {
          local_result = hil_write_other(qs3_pen_ss_model_DW.HILInitialize_Card,
            qs3_pen_ss_model_P.HILInitialize_OOChannels, num_final_other_outputs,
            qs3_pen_ss_model_P.HILInitialize_OOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(qs3_pen_ss_model_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(qs3_pen_ss_model_DW.HILInitialize_Card);
    hil_monitor_delete_all(qs3_pen_ss_model_DW.HILInitialize_Card);
    hil_close(qs3_pen_ss_model_DW.HILInitialize_Card);
    qs3_pen_ss_model_DW.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  qs3_pen_ss_model_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  qs3_pen_ss_model_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  qs3_pen_ss_model_initialize();
}

void MdlTerminate(void)
{
  qs3_pen_ss_model_terminate();
}

/* Registration function */
RT_MODEL_qs3_pen_ss_model_T *qs3_pen_ss_model(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)qs3_pen_ss_model_M, 0,
                sizeof(RT_MODEL_qs3_pen_ss_model_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&qs3_pen_ss_model_M->solverInfo,
                          &qs3_pen_ss_model_M->Timing.simTimeStep);
    rtsiSetTPtr(&qs3_pen_ss_model_M->solverInfo, &rtmGetTPtr(qs3_pen_ss_model_M));
    rtsiSetStepSizePtr(&qs3_pen_ss_model_M->solverInfo,
                       &qs3_pen_ss_model_M->Timing.stepSize0);
    rtsiSetdXPtr(&qs3_pen_ss_model_M->solverInfo, &qs3_pen_ss_model_M->derivs);
    rtsiSetContStatesPtr(&qs3_pen_ss_model_M->solverInfo, (real_T **)
                         &qs3_pen_ss_model_M->contStates);
    rtsiSetNumContStatesPtr(&qs3_pen_ss_model_M->solverInfo,
      &qs3_pen_ss_model_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&qs3_pen_ss_model_M->solverInfo,
      &qs3_pen_ss_model_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&qs3_pen_ss_model_M->solverInfo,
      &qs3_pen_ss_model_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&qs3_pen_ss_model_M->solverInfo,
      &qs3_pen_ss_model_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&qs3_pen_ss_model_M->solverInfo, (boolean_T**)
      &qs3_pen_ss_model_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&qs3_pen_ss_model_M->solverInfo, (&rtmGetErrorStatus
      (qs3_pen_ss_model_M)));
    rtsiSetRTModelPtr(&qs3_pen_ss_model_M->solverInfo, qs3_pen_ss_model_M);
  }

  rtsiSetSimTimeStep(&qs3_pen_ss_model_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&qs3_pen_ss_model_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&qs3_pen_ss_model_M->solverInfo, false);
  qs3_pen_ss_model_M->intgData.y = qs3_pen_ss_model_M->odeY;
  qs3_pen_ss_model_M->intgData.f[0] = qs3_pen_ss_model_M->odeF[0];
  qs3_pen_ss_model_M->intgData.f[1] = qs3_pen_ss_model_M->odeF[1];
  qs3_pen_ss_model_M->intgData.f[2] = qs3_pen_ss_model_M->odeF[2];
  qs3_pen_ss_model_M->intgData.f[3] = qs3_pen_ss_model_M->odeF[3];
  qs3_pen_ss_model_M->contStates = ((real_T *) &qs3_pen_ss_model_X);
  qs3_pen_ss_model_M->contStateDisabled = ((boolean_T *) &qs3_pen_ss_model_XDis);
  qs3_pen_ss_model_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&qs3_pen_ss_model_M->solverInfo, (void *)
                    &qs3_pen_ss_model_M->intgData);
  rtsiSetSolverName(&qs3_pen_ss_model_M->solverInfo,"ode4");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = qs3_pen_ss_model_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    qs3_pen_ss_model_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    qs3_pen_ss_model_M->Timing.sampleTimes =
      (&qs3_pen_ss_model_M->Timing.sampleTimesArray[0]);
    qs3_pen_ss_model_M->Timing.offsetTimes =
      (&qs3_pen_ss_model_M->Timing.offsetTimesArray[0]);

    /* task periods */
    qs3_pen_ss_model_M->Timing.sampleTimes[0] = (0.0);
    qs3_pen_ss_model_M->Timing.sampleTimes[1] = (0.002);

    /* task offsets */
    qs3_pen_ss_model_M->Timing.offsetTimes[0] = (0.0);
    qs3_pen_ss_model_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(qs3_pen_ss_model_M, &qs3_pen_ss_model_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = qs3_pen_ss_model_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    qs3_pen_ss_model_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(qs3_pen_ss_model_M, -1);
  qs3_pen_ss_model_M->Timing.stepSize0 = 0.002;
  qs3_pen_ss_model_M->Timing.stepSize1 = 0.002;

  /* External mode info */
  qs3_pen_ss_model_M->Sizes.checksums[0] = (2940744678U);
  qs3_pen_ss_model_M->Sizes.checksums[1] = (2570238403U);
  qs3_pen_ss_model_M->Sizes.checksums[2] = (2816117638U);
  qs3_pen_ss_model_M->Sizes.checksums[3] = (2206724761U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    qs3_pen_ss_model_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(qs3_pen_ss_model_M->extModeInfo,
      &qs3_pen_ss_model_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(qs3_pen_ss_model_M->extModeInfo,
                        qs3_pen_ss_model_M->Sizes.checksums);
    rteiSetTPtr(qs3_pen_ss_model_M->extModeInfo, rtmGetTPtr(qs3_pen_ss_model_M));
  }

  qs3_pen_ss_model_M->solverInfoPtr = (&qs3_pen_ss_model_M->solverInfo);
  qs3_pen_ss_model_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&qs3_pen_ss_model_M->solverInfo, 0.002);
  rtsiSetSolverMode(&qs3_pen_ss_model_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  qs3_pen_ss_model_M->blockIO = ((void *) &qs3_pen_ss_model_B);
  (void) memset(((void *) &qs3_pen_ss_model_B), 0,
                sizeof(B_qs3_pen_ss_model_T));

  /* parameters */
  qs3_pen_ss_model_M->defaultParam = ((real_T *)&qs3_pen_ss_model_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &qs3_pen_ss_model_X;
    qs3_pen_ss_model_M->contStates = (x);
    (void) memset((void *)&qs3_pen_ss_model_X, 0,
                  sizeof(X_qs3_pen_ss_model_T));
  }

  /* disabled states */
  {
    boolean_T *xdis = (boolean_T *) &qs3_pen_ss_model_XDis;
    qs3_pen_ss_model_M->contStateDisabled = (xdis);
    (void) memset((void *)&qs3_pen_ss_model_XDis, 0,
                  sizeof(XDis_qs3_pen_ss_model_T));
  }

  /* states (dwork) */
  qs3_pen_ss_model_M->dwork = ((void *) &qs3_pen_ss_model_DW);
  (void) memset((void *)&qs3_pen_ss_model_DW, 0,
                sizeof(DW_qs3_pen_ss_model_T));

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    qs3_pen_ss_model_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 22;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  qs3_pen_ss_model_M->Sizes.numContStates = (4);/* Number of continuous states */
  qs3_pen_ss_model_M->Sizes.numPeriodicContStates = (0);
                                      /* Number of periodic continuous states */
  qs3_pen_ss_model_M->Sizes.numY = (0);/* Number of model outputs */
  qs3_pen_ss_model_M->Sizes.numU = (0);/* Number of model inputs */
  qs3_pen_ss_model_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  qs3_pen_ss_model_M->Sizes.numSampTimes = (2);/* Number of sample times */
  qs3_pen_ss_model_M->Sizes.numBlocks = (12);/* Number of blocks */
  qs3_pen_ss_model_M->Sizes.numBlockIO = (5);/* Number of block outputs */
  qs3_pen_ss_model_M->Sizes.numBlockPrms = (135);/* Sum of parameter "widths" */
  return qs3_pen_ss_model_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
