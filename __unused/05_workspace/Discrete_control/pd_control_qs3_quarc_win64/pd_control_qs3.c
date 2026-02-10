/*
 * pd_control_qs3.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "pd_control_qs3".
 *
 * Model version              : 9.6
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Wed Sep 17 15:38:52 2025
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "pd_control_qs3.h"
#include "rtwtypes.h"
#include "pd_control_qs3_private.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "pd_control_qs3_dt.h"

const real_T pd_control_qs3_RGND = 0.0;/* real_T ground */

/* Block signals (default storage) */
B_pd_control_qs3_T pd_control_qs3_B;

/* Block states (default storage) */
DW_pd_control_qs3_T pd_control_qs3_DW;

/* Real-time model */
static RT_MODEL_pd_control_qs3_T pd_control_qs3_M_;
RT_MODEL_pd_control_qs3_T *const pd_control_qs3_M = &pd_control_qs3_M_;

/* Model output function */
void pd_control_qs3_output(void)
{
  real_T u0;
  real_T u1;
  real_T u2;

  /* S-Function (hil_read_timebase_block): '<S3>/HIL Read Timebase' */

  /* S-Function Block: pd_control_qs3/Qube-Servo 3 - IO (QAL)/HIL Read Timebase (hil_read_timebase_block) */
  {
    t_error result;
    result = hil_task_read(pd_control_qs3_DW.HILReadTimebase_Task, 1,
      &pd_control_qs3_DW.HILReadTimebase_AnalogBuffer,
      &pd_control_qs3_DW.HILReadTimebase_EncoderBuffer[0],
      &pd_control_qs3_DW.HILReadTimebase_DigitalBuffer,
      &pd_control_qs3_DW.HILReadTimebase_OtherBuffer[0]
      );
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
    } else {
      pd_control_qs3_B.HILReadTimebase_o1 =
        pd_control_qs3_DW.HILReadTimebase_AnalogBuffer;
      pd_control_qs3_B.HILReadTimebase_o2 =
        pd_control_qs3_DW.HILReadTimebase_EncoderBuffer[0];
      pd_control_qs3_B.HILReadTimebase_o3 =
        pd_control_qs3_DW.HILReadTimebase_EncoderBuffer[1];
      pd_control_qs3_B.HILReadTimebase_o4 =
        pd_control_qs3_DW.HILReadTimebase_DigitalBuffer;
      pd_control_qs3_B.HILReadTimebase_o5 =
        pd_control_qs3_DW.HILReadTimebase_OtherBuffer[0];
      pd_control_qs3_B.HILReadTimebase_o6 =
        pd_control_qs3_DW.HILReadTimebase_OtherBuffer[1];
    }
  }

  /* S-Function (smooth_signal_generator_block): '<Root>/Smooth Signal Generator' */
  /* S-Function Block: pd_control_qs3/Smooth Signal Generator (smooth_signal_generator_block) */
  {
    real_T w = TWO_PI * pd_control_qs3_P.SmoothSignalGenerator_Frequency;
    boolean_T params_changed = (pd_control_qs3_P.SmoothSignalGenerator_Amplitude
      != pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp || w !=
      pd_control_qs3_DW.SmoothSignalGenerator_RWORK.W);
    real_T T = TWO_PI / pd_control_qs3_DW.SmoothSignalGenerator_RWORK.W;
    if (pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T < 0.5 * T) {
      pd_control_qs3_B.SmoothSignalGenerator =
        pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp;
    } else {
      pd_control_qs3_B.SmoothSignalGenerator =
        -pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp;
    }

    /*
       If the amplitude or frequency parameter changes, then adjust the
       square wave parameters such that the square wave output is continuous.
     */
    if (params_changed) {
      /*
         Change the amplitude or frequency when the sign of the output
         changes. This technique ensures that we don't get intermediate
         jumps in amplitude and we don't get higher frequencies than
         expected due to a pulse being prematurely truncated.
       */
      if (pd_control_qs3_P.SmoothSignalGenerator_Amplitude == 0 ||
          pd_control_qs3_B.SmoothSignalGenerator == 0 ||
          pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Y == 0 ||
          (pd_control_qs3_B.SmoothSignalGenerator < 0) !=
          (pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Y < 0)) {
        pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp =
          pd_control_qs3_P.SmoothSignalGenerator_Amplitude;
        pd_control_qs3_DW.SmoothSignalGenerator_RWORK.W = w;
        T = TWO_PI / w;
        if (pd_control_qs3_B.SmoothSignalGenerator >= 0) {
          pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T = 0;
          pd_control_qs3_B.SmoothSignalGenerator =
            pd_control_qs3_P.SmoothSignalGenerator_Amplitude;
        } else {
          pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T = T / 2;
          pd_control_qs3_B.SmoothSignalGenerator =
            -pd_control_qs3_P.SmoothSignalGenerator_Amplitude;
        }
      }
    }

    pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T += 0.002;
    if (pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T >= T) {
      pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T = 0;
    }

    pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Y =
      pd_control_qs3_B.SmoothSignalGenerator;
  }

  /* Gain: '<Root>/Amplitude (rad)' */
  pd_control_qs3_B.Amplituderad = pd_control_qs3_P.Amplituderad_Gain *
    pd_control_qs3_B.SmoothSignalGenerator;

  /* Gain: '<Root>/counts to rads' */
  pd_control_qs3_B.countstorads = pd_control_qs3_P.countstorads_Gain *
    pd_control_qs3_B.HILReadTimebase_o2;

  /* Sum: '<Root>/Sum' */
  pd_control_qs3_B.Sum = pd_control_qs3_B.Amplituderad -
    pd_control_qs3_B.countstorads;

  /* Gain: '<S2>/Slider Gain' */
  pd_control_qs3_B.SliderGain = pd_control_qs3_P.ProportionalGain_gain *
    pd_control_qs3_B.Sum;

  /* Gain: '<Root>/counts//s to rad//s' */
  pd_control_qs3_B.countsstorads = pd_control_qs3_P.countsstorads_Gain *
    pd_control_qs3_B.HILReadTimebase_o5;

  /* Gain: '<S1>/Slider Gain' */
  pd_control_qs3_B.SliderGain_b = pd_control_qs3_P.DerivativeGain_gain *
    pd_control_qs3_B.countsstorads;

  /* Sum: '<Root>/Sum1' */
  pd_control_qs3_B.Sum1 = pd_control_qs3_B.SliderGain -
    pd_control_qs3_B.SliderGain_b;

  /* Saturate: '<Root>/+//- 10V Limit' */
  u0 = pd_control_qs3_B.Sum1;
  u1 = pd_control_qs3_P.u0VLimit_LowerSat;
  u2 = pd_control_qs3_P.u0VLimit_UpperSat;
  if (u0 > u2) {
    /* Saturate: '<Root>/+//- 10V Limit' */
    pd_control_qs3_B.u0VLimit = u2;
  } else if (u0 < u1) {
    /* Saturate: '<Root>/+//- 10V Limit' */
    pd_control_qs3_B.u0VLimit = u1;
  } else {
    /* Saturate: '<Root>/+//- 10V Limit' */
    pd_control_qs3_B.u0VLimit = u0;
  }

  /* End of Saturate: '<Root>/+//- 10V Limit' */

  /* S-Function (hil_write_block): '<S3>/HIL Write' */

  /* S-Function Block: pd_control_qs3/Qube-Servo 3 - IO (QAL)/HIL Write (hil_write_block) */
  {
    t_error result;
    result = hil_write(pd_control_qs3_DW.HILInitialize_Card,
                       &pd_control_qs3_P.HILWrite_analog_channels, 1U,
                       NULL, 0U,
                       NULL, 0U,
                       &pd_control_qs3_P.HILWrite_other_channels, 1U,
                       &pd_control_qs3_B.u0VLimit,
                       NULL,
                       NULL,
                       ((const real_T*) &pd_control_qs3_RGND)
                       );
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
    }
  }
}

/* Model update function */
void pd_control_qs3_update(void)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++pd_control_qs3_M->Timing.clockTick0)) {
    ++pd_control_qs3_M->Timing.clockTickH0;
  }

  pd_control_qs3_M->Timing.t[0] = pd_control_qs3_M->Timing.clockTick0 *
    pd_control_qs3_M->Timing.stepSize0 + pd_control_qs3_M->Timing.clockTickH0 *
    pd_control_qs3_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void pd_control_qs3_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<S3>/HIL Initialize' */

  /* S-Function Block: pd_control_qs3/Qube-Servo 3 - IO (QAL)/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("qube_servo3_usb", "0",
                      &pd_control_qs3_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options(pd_control_qs3_DW.HILInitialize_Card,
      "deadband_compensation=0.3;pwm_en=0;enc0_velocity=3.0;enc1_velocity=3.0;min_diode_compensation=0.3;max_diode_compensation=1.5",
      125);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(pd_control_qs3_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
      return;
    }

    if ((pd_control_qs3_P.HILInitialize_AIPStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_AIPEnter && is_switching)) {
      result = hil_set_analog_input_ranges(pd_control_qs3_DW.HILInitialize_Card,
        &pd_control_qs3_P.HILInitialize_AIChannels, 1U,
        &pd_control_qs3_P.HILInitialize_AILow,
        &pd_control_qs3_P.HILInitialize_AIHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if ((pd_control_qs3_P.HILInitialize_AOPStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_AOPEnter && is_switching)) {
      result = hil_set_analog_output_ranges(pd_control_qs3_DW.HILInitialize_Card,
        &pd_control_qs3_P.HILInitialize_AOChannels, 1U,
        &pd_control_qs3_P.HILInitialize_AOLow,
        &pd_control_qs3_P.HILInitialize_AOHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if ((pd_control_qs3_P.HILInitialize_AOStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_AOEnter && is_switching)) {
      result = hil_write_analog(pd_control_qs3_DW.HILInitialize_Card,
        &pd_control_qs3_P.HILInitialize_AOChannels, 1U,
        &pd_control_qs3_P.HILInitialize_AOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if (pd_control_qs3_P.HILInitialize_AOReset) {
      result = hil_watchdog_set_analog_expiration_state
        (pd_control_qs3_DW.HILInitialize_Card,
         &pd_control_qs3_P.HILInitialize_AOChannels, 1U,
         &pd_control_qs3_P.HILInitialize_AOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    result = hil_set_digital_directions(pd_control_qs3_DW.HILInitialize_Card,
      NULL, 0U, &pd_control_qs3_P.HILInitialize_DOChannels, 1U);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
      return;
    }

    if ((pd_control_qs3_P.HILInitialize_DOStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_DOEnter && is_switching)) {
      result = hil_write_digital(pd_control_qs3_DW.HILInitialize_Card,
        &pd_control_qs3_P.HILInitialize_DOChannels, 1U, (t_boolean *)
        &pd_control_qs3_P.HILInitialize_DOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if (pd_control_qs3_P.HILInitialize_DOReset) {
      result = hil_watchdog_set_digital_expiration_state
        (pd_control_qs3_DW.HILInitialize_Card,
         &pd_control_qs3_P.HILInitialize_DOChannels, 1U, (const t_digital_state *)
         &pd_control_qs3_P.HILInitialize_DOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if ((pd_control_qs3_P.HILInitialize_EIPStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_EIPEnter && is_switching)) {
      pd_control_qs3_DW.HILInitialize_QuadratureModes[0] =
        pd_control_qs3_P.HILInitialize_EIQuadrature;
      pd_control_qs3_DW.HILInitialize_QuadratureModes[1] =
        pd_control_qs3_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (pd_control_qs3_DW.HILInitialize_Card,
         pd_control_qs3_P.HILInitialize_EIChannels, 2U,
         (t_encoder_quadrature_mode *)
         &pd_control_qs3_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if ((pd_control_qs3_P.HILInitialize_EIStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_EIEnter && is_switching)) {
      pd_control_qs3_DW.HILInitialize_InitialEICounts[0] =
        pd_control_qs3_P.HILInitialize_EIInitial;
      pd_control_qs3_DW.HILInitialize_InitialEICounts[1] =
        pd_control_qs3_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(pd_control_qs3_DW.HILInitialize_Card,
        pd_control_qs3_P.HILInitialize_EIChannels, 2U,
        &pd_control_qs3_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if ((pd_control_qs3_P.HILInitialize_OOStart && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_OOEnter && is_switching)) {
      result = hil_write_other(pd_control_qs3_DW.HILInitialize_Card,
        pd_control_qs3_P.HILInitialize_OOChannels, 3U,
        pd_control_qs3_P.HILInitialize_OOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }

    if (pd_control_qs3_P.HILInitialize_OOReset) {
      result = hil_watchdog_set_other_expiration_state
        (pd_control_qs3_DW.HILInitialize_Card,
         pd_control_qs3_P.HILInitialize_OOChannels, 3U,
         pd_control_qs3_P.HILInitialize_OOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_timebase_block): '<S3>/HIL Read Timebase' */

  /* S-Function Block: pd_control_qs3/Qube-Servo 3 - IO (QAL)/HIL Read Timebase (hil_read_timebase_block) */
  {
    t_error result;
    result = hil_task_create_reader(pd_control_qs3_DW.HILInitialize_Card,
      pd_control_qs3_P.HILReadTimebase_SamplesInBuffer,
      &pd_control_qs3_P.HILReadTimebase_AnalogChannels, 1U,
      pd_control_qs3_P.HILReadTimebase_EncoderChannels, 2U,
      &pd_control_qs3_P.HILReadTimebase_DigitalChannels, 1U,
      pd_control_qs3_P.HILReadTimebase_OtherChannels, 2U,
      &pd_control_qs3_DW.HILReadTimebase_Task);
    if (result >= 0) {
      result = hil_task_set_buffer_overflow_mode
        (pd_control_qs3_DW.HILReadTimebase_Task, (t_buffer_overflow_mode)
         (pd_control_qs3_P.HILReadTimebase_OverflowMode - 1));
    }

    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
    }
  }

  /* Start for S-Function (smooth_signal_generator_block): '<Root>/Smooth Signal Generator' */

  /* S-Function Block: pd_control_qs3/Smooth Signal Generator (smooth_signal_generator_block) */
  {
    real_T T = 1.0 / pd_control_qs3_P.SmoothSignalGenerator_Frequency;
    pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp =
      pd_control_qs3_P.SmoothSignalGenerator_Amplitude;
    pd_control_qs3_DW.SmoothSignalGenerator_RWORK.W = TWO_PI *
      pd_control_qs3_P.SmoothSignalGenerator_Frequency;
    pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T = fmod
      (pd_control_qs3_P.SmoothSignalGenerator_InitialPh, TWO_PI) * T / TWO_PI;
    if (pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T < 0) {
      pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T += T;
    }

    if (pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T < 0.5 * T) {
      pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Y =
        pd_control_qs3_P.SmoothSignalGenerator_Amplitude;
    } else {
      pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Y =
        -pd_control_qs3_P.SmoothSignalGenerator_Amplitude;
    }
  }
}

/* Model terminate function */
void pd_control_qs3_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<S3>/HIL Initialize' */

  /* S-Function Block: pd_control_qs3/Qube-Servo 3 - IO (QAL)/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_digital_outputs = 0;
    t_uint32 num_final_other_outputs = 0;
    hil_task_stop_all(pd_control_qs3_DW.HILInitialize_Card);
    hil_monitor_stop_all(pd_control_qs3_DW.HILInitialize_Card);
    is_switching = false;
    if ((pd_control_qs3_P.HILInitialize_AOTerminate && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_AOExit && is_switching)) {
      num_final_analog_outputs = 1U;
    } else {
      num_final_analog_outputs = 0;
    }

    if ((pd_control_qs3_P.HILInitialize_DOTerminate && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_DOExit && is_switching)) {
      num_final_digital_outputs = 1U;
    } else {
      num_final_digital_outputs = 0;
    }

    if ((pd_control_qs3_P.HILInitialize_OOTerminate && !is_switching) ||
        (pd_control_qs3_P.HILInitialize_OOExit && is_switching)) {
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
      result = hil_write(pd_control_qs3_DW.HILInitialize_Card
                         , &pd_control_qs3_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , NULL, 0
                         , &pd_control_qs3_P.HILInitialize_DOChannels,
                         num_final_digital_outputs
                         , pd_control_qs3_P.HILInitialize_OOChannels,
                         num_final_other_outputs
                         , &pd_control_qs3_P.HILInitialize_AOFinal
                         , NULL
                         , (t_boolean *) &pd_control_qs3_P.HILInitialize_DOFinal
                         , pd_control_qs3_P.HILInitialize_OOFinal
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog(pd_control_qs3_DW.HILInitialize_Card,
            &pd_control_qs3_P.HILInitialize_AOChannels, num_final_analog_outputs,
            &pd_control_qs3_P.HILInitialize_AOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_digital_outputs > 0) {
          local_result = hil_write_digital(pd_control_qs3_DW.HILInitialize_Card,
            &pd_control_qs3_P.HILInitialize_DOChannels,
            num_final_digital_outputs, (t_boolean *)
            &pd_control_qs3_P.HILInitialize_DOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_other_outputs > 0) {
          local_result = hil_write_other(pd_control_qs3_DW.HILInitialize_Card,
            pd_control_qs3_P.HILInitialize_OOChannels, num_final_other_outputs,
            pd_control_qs3_P.HILInitialize_OOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(pd_control_qs3_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(pd_control_qs3_DW.HILInitialize_Card);
    hil_monitor_delete_all(pd_control_qs3_DW.HILInitialize_Card);
    hil_close(pd_control_qs3_DW.HILInitialize_Card);
    pd_control_qs3_DW.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/
void MdlOutputs(int_T tid)
{
  pd_control_qs3_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  pd_control_qs3_update();
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
  pd_control_qs3_initialize();
}

void MdlTerminate(void)
{
  pd_control_qs3_terminate();
}

/* Registration function */
RT_MODEL_pd_control_qs3_T *pd_control_qs3(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)pd_control_qs3_M, 0,
                sizeof(RT_MODEL_pd_control_qs3_T));

  /* Initialize timing info */
  {
    int_T *mdlTsMap = pd_control_qs3_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    pd_control_qs3_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    pd_control_qs3_M->Timing.sampleTimes =
      (&pd_control_qs3_M->Timing.sampleTimesArray[0]);
    pd_control_qs3_M->Timing.offsetTimes =
      (&pd_control_qs3_M->Timing.offsetTimesArray[0]);

    /* task periods */
    pd_control_qs3_M->Timing.sampleTimes[0] = (0.002);

    /* task offsets */
    pd_control_qs3_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(pd_control_qs3_M, &pd_control_qs3_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = pd_control_qs3_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    pd_control_qs3_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(pd_control_qs3_M, -1);
  pd_control_qs3_M->Timing.stepSize0 = 0.002;

  /* External mode info */
  pd_control_qs3_M->Sizes.checksums[0] = (2627873013U);
  pd_control_qs3_M->Sizes.checksums[1] = (1361881096U);
  pd_control_qs3_M->Sizes.checksums[2] = (2052606178U);
  pd_control_qs3_M->Sizes.checksums[3] = (3885486571U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    pd_control_qs3_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(pd_control_qs3_M->extModeInfo,
      &pd_control_qs3_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(pd_control_qs3_M->extModeInfo,
                        pd_control_qs3_M->Sizes.checksums);
    rteiSetTPtr(pd_control_qs3_M->extModeInfo, rtmGetTPtr(pd_control_qs3_M));
  }

  pd_control_qs3_M->solverInfoPtr = (&pd_control_qs3_M->solverInfo);
  pd_control_qs3_M->Timing.stepSize = (0.002);
  rtsiSetFixedStepSize(&pd_control_qs3_M->solverInfo, 0.002);
  rtsiSetSolverMode(&pd_control_qs3_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  pd_control_qs3_M->blockIO = ((void *) &pd_control_qs3_B);
  (void) memset(((void *) &pd_control_qs3_B), 0,
                sizeof(B_pd_control_qs3_T));

  {
    pd_control_qs3_B.HILReadTimebase_o1 = 0.0;
    pd_control_qs3_B.HILReadTimebase_o2 = 0.0;
    pd_control_qs3_B.HILReadTimebase_o3 = 0.0;
    pd_control_qs3_B.HILReadTimebase_o5 = 0.0;
    pd_control_qs3_B.HILReadTimebase_o6 = 0.0;
    pd_control_qs3_B.SmoothSignalGenerator = 0.0;
    pd_control_qs3_B.Amplituderad = 0.0;
    pd_control_qs3_B.countstorads = 0.0;
    pd_control_qs3_B.Sum = 0.0;
    pd_control_qs3_B.SliderGain = 0.0;
    pd_control_qs3_B.countsstorads = 0.0;
    pd_control_qs3_B.SliderGain_b = 0.0;
    pd_control_qs3_B.Sum1 = 0.0;
    pd_control_qs3_B.u0VLimit = 0.0;
  }

  /* parameters */
  pd_control_qs3_M->defaultParam = ((real_T *)&pd_control_qs3_P);

  /* states (dwork) */
  pd_control_qs3_M->dwork = ((void *) &pd_control_qs3_DW);
  (void) memset((void *)&pd_control_qs3_DW, 0,
                sizeof(DW_pd_control_qs3_T));
  pd_control_qs3_DW.HILInitialize_FilterFrequency[0] = 0.0;
  pd_control_qs3_DW.HILInitialize_FilterFrequency[1] = 0.0;
  pd_control_qs3_DW.HILReadTimebase_AnalogBuffer = 0.0;
  pd_control_qs3_DW.HILReadTimebase_OtherBuffer[0] = 0.0;
  pd_control_qs3_DW.HILReadTimebase_OtherBuffer[1] = 0.0;
  pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Amp = 0.0;
  pd_control_qs3_DW.SmoothSignalGenerator_RWORK.W = 0.0;
  pd_control_qs3_DW.SmoothSignalGenerator_RWORK.T = 0.0;
  pd_control_qs3_DW.SmoothSignalGenerator_RWORK.Y = 0.0;

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    pd_control_qs3_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 22;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  pd_control_qs3_M->Sizes.numContStates = (0);/* Number of continuous states */
  pd_control_qs3_M->Sizes.numY = (0);  /* Number of model outputs */
  pd_control_qs3_M->Sizes.numU = (0);  /* Number of model inputs */
  pd_control_qs3_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  pd_control_qs3_M->Sizes.numSampTimes = (1);/* Number of sample times */
  pd_control_qs3_M->Sizes.numBlocks = (14);/* Number of blocks */
  pd_control_qs3_M->Sizes.numBlockIO = (15);/* Number of block outputs */
  pd_control_qs3_M->Sizes.numBlockPrms = (99);/* Sum of parameter "widths" */
  return pd_control_qs3_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
