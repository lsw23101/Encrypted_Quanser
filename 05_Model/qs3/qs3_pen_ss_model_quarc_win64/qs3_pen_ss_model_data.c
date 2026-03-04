/*
 * qs3_pen_ss_model_data.c
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

/* Block parameters (default storage) */
P_qs3_pen_ss_model_T qs3_pen_ss_model_P = {
  /* Variable: A
   * Referenced by: '<Root>/State-Space Model'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 149.2750968650933, 261.60910736666204, 1.0,
    0.0, -0.013099426061372065, -0.012947107153681692, 0.0, 1.0, -0.0, -0.0 },

  /* Variable: B
   * Referenced by: '<Root>/State-Space Model'
   */
  { 0.0, 0.0, 55.694838696309795, 55.047224292864328 },

  /* Variable: C
   * Referenced by: '<Root>/State-Space Model'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  /* Mask Parameter: HILWrite_analog_channels
   * Referenced by: '<S1>/HIL Write'
   */
  0U,

  /* Mask Parameter: HILWrite_other_channels
   * Referenced by: '<S1>/HIL Write'
   */
  11000U,

  /* Expression: set_other_outputs_at_terminate
   * Referenced by: '<S1>/HIL Initialize'
   */
  1.0,

  /* Expression: set_other_outputs_at_switch_out
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: set_other_outputs_at_start
   * Referenced by: '<S1>/HIL Initialize'
   */
  1.0,

  /* Expression: set_other_outputs_at_switch_in
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: final_analog_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: final_pwm_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: final_other_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  { 1.0, 0.0, 0.0 },

  /* Expression: analog_input_maximums
   * Referenced by: '<S1>/HIL Initialize'
   */
  3.0,

  /* Expression: analog_input_minimums
   * Referenced by: '<S1>/HIL Initialize'
   */
  -3.0,

  /* Expression: analog_output_maximums
   * Referenced by: '<S1>/HIL Initialize'
   */
  15.0,

  /* Expression: analog_output_minimums
   * Referenced by: '<S1>/HIL Initialize'
   */
  -15.0,

  /* Expression: initial_analog_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: watchdog_analog_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: pwm_frequency
   * Referenced by: '<S1>/HIL Initialize'
   */
  23195.0,

  /* Expression: pwm_leading_deadband
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: pwm_trailing_deadband
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: initial_pwm_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  0.0,

  /* Expression: initial_other_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  { 0.0, 1.0, 0.0 },

  /* Expression: watchdog_other_outputs
   * Referenced by: '<S1>/HIL Initialize'
   */
  { 0.0, 0.0, 1.0 },

  /* Expression: 1
   * Referenced by: '<Root>/Signal Generator'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Signal Generator'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Amplitude (V)'
   */
  1.0,

  /* Expression: -1
   * Referenced by: '<Root>/For +ve CCW  motion'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Base: counts to rad'
   */
  1.0,

  /* Expression: [0 0 0 0]
   * Referenced by: '<Root>/State-Space Model'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /* Expression: 1
   * Referenced by: '<Root>/Pendulum: counts to rad'
   */
  1.0,

  /* Computed Parameter: HILInitialize_CKChannels
   * Referenced by: '<S1>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOWatchdog
   * Referenced by: '<S1>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_EIInitial
   * Referenced by: '<S1>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POModes
   * Referenced by: '<S1>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POConfiguration
   * Referenced by: '<S1>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_POAlignment
   * Referenced by: '<S1>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POPolarity
   * Referenced by: '<S1>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILReadTimebase_Clock
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  0,

  /* Computed Parameter: HILInitialize_AIChannels
   * Referenced by: '<S1>/HIL Initialize'
   */
  0U,

  /* Computed Parameter: HILInitialize_AOChannels
   * Referenced by: '<S1>/HIL Initialize'
   */
  0U,

  /* Computed Parameter: HILInitialize_DOChannels
   * Referenced by: '<S1>/HIL Initialize'
   */
  0U,

  /* Computed Parameter: HILInitialize_EIChannels
   * Referenced by: '<S1>/HIL Initialize'
   */
  { 0U, 1U },

  /* Computed Parameter: HILInitialize_EIQuadrature
   * Referenced by: '<S1>/HIL Initialize'
   */
  4U,

  /* Computed Parameter: HILInitialize_OOChannels
   * Referenced by: '<S1>/HIL Initialize'
   */
  { 11000U, 11001U, 11002U },

  /* Computed Parameter: HILReadTimebase_SamplesInBuffer
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  500U,

  /* Computed Parameter: HILReadTimebase_AnalogChannels
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  0U,

  /* Computed Parameter: HILReadTimebase_EncoderChannels
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  { 0U, 1U },

  /* Computed Parameter: HILReadTimebase_DigitalChannels
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  1U,

  /* Computed Parameter: HILReadTimebase_OtherChannels
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  { 14000U, 14001U },

  /* Computed Parameter: HILInitialize_Active
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_AOTerminate
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_AOExit
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_DOTerminate
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_DOExit
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POTerminate
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POExit
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_CKPStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_CKPEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_CKStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_CKEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_AIPStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_AIPEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_AOPStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_AOPEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_AOStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_AOEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_AOReset
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_DOPStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_DOPEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_DOStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_DOEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_DOReset
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_EIPStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_EIPEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_EIStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_EIEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POPStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POPEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POStart
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POEnter
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_POReset
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_OOReset
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILInitialize_DOFinal
   * Referenced by: '<S1>/HIL Initialize'
   */
  false,

  /* Computed Parameter: HILInitialize_DOInitial
   * Referenced by: '<S1>/HIL Initialize'
   */
  true,

  /* Computed Parameter: HILReadTimebase_Active
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  true,

  /* Computed Parameter: HILWrite_Active
   * Referenced by: '<S1>/HIL Write'
   */
  false,

  /* Computed Parameter: HILReadTimebase_OverflowMode
   * Referenced by: '<S1>/HIL Read Timebase'
   */
  1U
};
