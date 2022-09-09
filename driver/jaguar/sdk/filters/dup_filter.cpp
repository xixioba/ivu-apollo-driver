/**
 *  Copyright (C) 2020 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include <string.h>
#include <stdlib.h>

#include "../src/inno_lidar_api.h"
extern "C" {
  typedef void (*filter_log_t)(int level,
                               const char *header_simple,
                               const char *header,
                               const char *msg);
}
filter_log_t g_log_fn = NULL;
int g_debug_level = 0;

class Filter {
 public:
  explicit Filter(const char *name) {
    if (name) {
      m_name = strdup(name);
    }
  }

  ~Filter() {
    free(m_name);
    m_name = NULL;
  }

  int set_cfg(const char *cfg_name_in,
              const char *cfg_value_in) {
    if (g_log_fn) {
      g_log_fn(4, "simple", "header", cfg_name_in);
    }
    return 0;
  }

  int process_frame(inno_cframe_header* cframe,
                    inno_cframe_header*** cframes_out,
                    size_t *cframe_out_size) {
    *cframe_out_size = 2;
    *cframes_out = reinterpret_cast<inno_cframe_header**>(
        malloc(sizeof(inno_cframe_header*) * *cframe_out_size));

    // First frame is the source frame itself.
    (*cframes_out)[0] = cframe;
    size_t s = inno_get_cframe_size_from_header(cframe);

    // Second frame is the copy of the source frame
    (*cframes_out)[1] = reinterpret_cast<inno_cframe_header*>(malloc(s));
    memcpy((*cframes_out)[1], cframe, s);

    // Since the source frame is referenced in the result,
    // need to tell the caller do not free the source frame.
    // Otherwise return 0 so caller will free the source.
    return 1;
  }

 private:
  char *m_name;
};

extern "C" {
  void *open_filter(const char *name,
                    const char **filter_name,
                    const char **filter_version_str,
                    const char **filter_build_str,
                    const char **filter_cfg_prefix,
                    int *sdk_major_version,
                    int *sdk_minor_version,
                    int *sdk_dot_version) {
    *filter_name = strdup("dup_filter");
    *filter_version_str = strdup("version-sample-str");
    *filter_build_str = strdup("build-sample-str");
    *filter_cfg_prefix = strdup("dup_filter_");
    *sdk_major_version = atoi(INNO_SDK_V_MAJOR);
    *sdk_minor_version = atoi(INNO_SDK_V_MINOR);
    *sdk_dot_version = atoi(INNO_SDK_V_DOT);
    return new Filter(name);
  }

  int close_filter(void *handle) {
    delete reinterpret_cast<Filter *>(handle);
    return 0;
  }

  int set_debug_level(void *handle,
                      int debug_level) {
    g_debug_level = debug_level;
    return 0;
  }

  int set_debug(void *handle,
                  int debug_level,
                  filter_log_t log) {
    set_debug_level(handle, debug_level);
    g_log_fn = log;
    return 0;
  }

  int set_cfg(void *handle,
              const char *cfg_name_in,
              const char *cfg_value_in) {
    return reinterpret_cast<Filter *>(handle)->set_cfg(cfg_name_in,
                                                          cfg_value_in);
  }

  int process_frame(void *handle,
                    void *cframe,
                    void *cframes_out,
                    size_t *cframe_out_size) {
    return reinterpret_cast<Filter *>(handle)->process_frame(
        reinterpret_cast<inno_cframe_header*>(cframe),
        reinterpret_cast<inno_cframe_header***>(cframes_out),
        cframe_out_size);
  }

}
