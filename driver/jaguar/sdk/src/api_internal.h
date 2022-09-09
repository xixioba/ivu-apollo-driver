/**
 *  Copyright (C) 2018 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef SRC_API_INTERNAL_H_
#define SRC_API_INTERNAL_H_

#include "./inno_lidar_api.h"

enum inno_i_callback_type {
  INNO_I_CALLBACK_NONE = 0,
  INNO_I_CALLBACK_CALI_GEO_WRITE,
  INNO_I_CALLBACK_CALI_REF_WRITE,
  INNO_I_CALLBACK_CALI_APD_WRITE,
  INNO_I_CALLBACK_PULSE_AREF_WRITE,
  INNO_I_CALLBACK_PULSE_ARET_WRITE,
  INNO_I_CALLBACK_PULSE_BREF_WRITE,
  INNO_I_CALLBACK_PULSE_BRET_WRITE,
  INNO_I_CALLBACK_BA_ENC_WRITE,
  INNO_I_CALLBACK_BI_ENC_WRITE,
  INNO_I_CALLBACK_PA_ENC_WRITE,
  INNO_I_CALLBACK_PI_ENC_WRITE,
  INNO_I_CALLBACK_CROSSTALK_WRITE,
  INNO_I_CALLBACK_MAX,
};

enum geo_mode {
  GEO_MODE_SIMPLE = 0,
  GEO_MODE_OFFAXIS,
  GEO_MODE_NPS
};

enum raw_capture_chan {
  RAW_CAPTURE_NONE = -1,
  RAW_CAPTURE_CHAN_0 = 0,
  RAW_CAPTURE_CHAN_1 = 1,
};

struct nv_pair {
  const char *name;
  double value;
};

struct dynamic_params {
  int no_of_params;
  struct nv_pair params[0];
};

DEFINE_COMPACT_STRUCT(inno_raw_point) {
  float x;                 /* in meter           */
  float y;                 /* in meter           */
  float z;                 /* in meter           */
  float radius;            /* in meter           */
  float intensity;
  float ref_intensity;
  float ref_time_0;
  float ref_time_1;
  uint16_t raw_data[16];
};
DEFINE_COMPACT_STRUCT_END

DEFINE_COMPACT_STRUCT(inno_raw_frame) {
  uint64_t idx;  /* frame index, start from 0                     */
  uint16_t sub_idx;    /* sub-frame index, start from 0 for every frame */
  uint16_t sub_seq;    /* sequence within a sub-frame             */
  inno_timestamp_us_t ts_us_start;   /* in microsecond            */
  inno_timestamp_us_t ts_us_end;     /* in microsecond            */
  unsigned int points_number;  /* number of points                */
  enum inno_timestamp_sync timestamp_sync_type;
  struct inno_raw_point points[0];
};
DEFINE_COMPACT_STRUCT_END

extern "C" {
  typedef void (*inno_lidar_i_callback_t)(int lidar_handle, void *context,
                                          enum inno_i_callback_type type,
                                          const char *buffer, int len);
  typedef int (*inno_lidar_raw_frame_callback_t)\
  (int lidar_handle, void *context,
   struct inno_raw_frame *raw_frame);
  int inno_lidar_set_dynamic_parameters(int handle, struct dynamic_params* dp);
  int inno_lidar_set_raw_frame_callback(\
      int handle, inno_lidar_raw_frame_callback_t raw_frame_callback);
  int inno_lidar_set_internal_callbacks(\
      int handle, enum inno_i_callback_type type,
      inno_lidar_i_callback_t callback);
  int inno_lidar_set_multiple_returns(int handle, int multi_returns);
  int inno_lidar_set_geo_mode(int handle, enum geo_mode mode);
  int inno_lidar_set_raw_capture_chan(\
      int handle, enum raw_capture_chan channel);
  int inno_lidar_get_internal_stats(int handle);
  int inno_lidar_set_use_beam_table(int handle, int use_beam_table);
};

#endif  // SRC_API_INTERNAL_H_
