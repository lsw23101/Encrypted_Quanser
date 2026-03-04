/*
 * qs3_pen_ss_model_dt.h
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

#include "ext_types.h"

/* data type size table */
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(t_card),
  sizeof(t_task),
  sizeof(t_boolean),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

/* data type name table */
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "t_card",
  "t_task",
  "t_boolean",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

/* data type transitions for block I/O structure */
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&qs3_pen_ss_model_B.AmplitudeV), 0, 0, 8 }
  ,

  { (char_T *)(&qs3_pen_ss_model_DW.HILInitialize_FilterFrequency[0]), 0, 0, 5 },

  { (char_T *)(&qs3_pen_ss_model_DW.HILInitialize_Card), 15, 0, 1 },

  { (char_T *)(&qs3_pen_ss_model_DW.HILReadTimebase_Task), 16, 0, 1 },

  { (char_T *)(&qs3_pen_ss_model_DW.HILWrite_PWORK), 11, 0, 4 },

  { (char_T *)(&qs3_pen_ss_model_DW.HILInitialize_ClockModes), 6, 0, 8 },

  { (char_T *)(&qs3_pen_ss_model_DW.HILReadTimebase_DigitalBuffer), 17, 0, 1 }
};

/* data type transition table for block I/O structure */
static DataTypeTransitionTable rtBTransTable = {
  7U,
  rtBTransitions
};

/* data type transitions for Parameters structure */
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&qs3_pen_ss_model_P.A[0]), 0, 0, 36 },

  { (char_T *)(&qs3_pen_ss_model_P.HILWrite_analog_channels), 7, 0, 2 },

  { (char_T *)(&qs3_pen_ss_model_P.HILInitialize_OOTerminate), 0, 0, 35 },

  { (char_T *)(&qs3_pen_ss_model_P.HILInitialize_CKChannels), 6, 0, 8 },

  { (char_T *)(&qs3_pen_ss_model_P.HILInitialize_AIChannels), 7, 0, 16 },

  { (char_T *)(&qs3_pen_ss_model_P.HILInitialize_Active), 8, 0, 37 },

  { (char_T *)(&qs3_pen_ss_model_P.HILReadTimebase_OverflowMode), 3, 0, 1 }
};

/* data type transition table for Parameters structure */
static DataTypeTransitionTable rtPTransTable = {
  7U,
  rtPTransitions
};

/* [EOF] qs3_pen_ss_model_dt.h */
