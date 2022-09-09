/**
 *  Copyright (C) 2018 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

#include <inno_lidar_api.h>

FILE *writeFH = NULL;
const int MAX_INS = 10;
bool is_alive[MAX_INS];
static const double us_in_second_c = 1000000.0;
static const double hundred_us_in_second_c = 10000.0;
int inno_lidar_frame_callback(int lidar_handle, void *context,
                              struct inno_frame *frame) {
  double ts_s_start = frame->ts_us_start / us_in_second_c;
  printf("[LIDAR-%d] got frame %lu-%d: timestamp=%f-%f, %d points\r\n",
         lidar_handle, frame->idx, frame->sub_idx,
         ts_s_start,
         frame->ts_us_end/us_in_second_c, frame->points_number);

  // For performance, only print two points to stdout.
  for (unsigned int i = 0; i < 2 && i < frame->points_number; i++) {
    if (i == 1) {
      i = frame->points_number - 1;
    }
    inno_point *p = &frame->points[i];
    printf("[LIDAR-%d]   point[%5u] timestamp=%f x=%f y=%f z=%f "
           "radius=%f reflectance=%d flags=%d\r\n",
           lidar_handle, i,
           p->ts_100us / hundred_us_in_second_c + ts_s_start,
           p->x, p->y, p->z, p->radius,
           static_cast<int>(p->ref), static_cast<int>(p->flags));
  }

  // If there's a file handle, write all the points to the file.
  // Again for performance, only write the significant digits (distances
  // are accurate to cm).
  if (NULL != writeFH) {
    for (unsigned int i = 0; i < frame->points_number; i++) {
      inno_point *p = &frame->points[i];
      // frame, points, timestamp, x, y, z, radius, reflectance, flags
      fprintf(writeFH, "%lu, %5u, %f, %.2f, %.2f, %.2f, %.2f, %d, %d\r\n",
              frame->idx, i,
              p->ts_100us/hundred_us_in_second_c + ts_s_start,
              p->x, p->y, p->z, p->radius,
              static_cast<int>(p->ref), static_cast<int>(p->flags));
    }
  }

  return 0;
}

void inno_lidar_alarm_callback(int lidar_handle, void *context,
                               enum inno_alarm error_level,
                               enum inno_alarm_code error_code,
                               const char *error_message) {
  // Print alarm to stderr; consider other actions depending on the
  // level, code or message.
  fprintf(stderr, "### [LIDAR-%d] LIDAR ALARM: level=%d, code=%d, message=%s\n",
          lidar_handle, error_level, error_code, error_message);

  // For critical or fatal alarms, the lidar connection should be restarted.
  if (error_level >= INNO_ALARM_CRITICAL &&
      error_code != INNO_ALARM_CODE_LIB_VERSION_MISMATCH) {
    fprintf(stderr, "notify lidar to stop\n");
    is_alive[intptr_t(context)] = false;
  }
  return;
}

void inno_lidar_log_test_callback(enum inno_log_level level,
                                  const char *header1,
                                  const char *header2, const char *msg) {
  fprintf(stderr, "level=%d header1=%s header2=%s body=%s\n",
          level, header1, header2, msg);
}

int get_new_lidar_handle(int idx, int speed, const char *datfile,
                         const char *ip, const char *yaml, const char *model) {
  char buf[32];
  snprintf(buf, sizeof(buf), "test-lidar-%d", idx);
  int handle;
  if (ip) {
    handle = inno_lidar_open_live(buf, ip, 8001, true);
  } else {
    handle = inno_lidar_open_file(buf, datfile, speed, 0, 0);
  }
  assert(handle > 0);

  int params = inno_lidar_set_parameters(handle, model, "", yaml);
  assert(params == 0);

  int ret = inno_lidar_set_reflectance_mode(handle,
                                            REFLECTANCE_MODE_INTENSITY);
  assert(ret == 0);

  inno_lidar_set_callbacks(handle, inno_lidar_alarm_callback,
                           inno_lidar_frame_callback, NULL,
                           reinterpret_cast<void *>(idx));
  inno_lidar_set_reflectance_mode(handle, REFLECTANCE_MODE_REFLECTIVITY);
  return handle;
}

void usage(const char *argv0) {
  fprintf(stderr, "usage: %s {-f data-file | -n LIDAR_IP} [-m model] "
          "[-y yaml-file] [-s speed] [-i instance_num] "
          "[-d duration-in-sec] [-v | -b] [-w pc-output-file.csv] "
          "[-r repeat-count] [-k | -t]\n",
          argv0);
  fprintf(stderr, "If the '-f data-file' argument is used, '-y yaml-file' must"
          "also be supplied.");
  exit(1);
}

int main(int argc, char *argv[]) {
  // Variables appropriate for both live & replaying from file.
  const char *writefile = NULL;
  int repeat = 1;
  int instance = 1;
  int duration = 10000000;
  int keep_alive = 0;
  int verbose_flag = 0;
  const char *ip = NULL;
  const char *yaml = NULL;
  const char *model = "REV_E";

  // Variables needed for running from a previously saved .dat file.
  const char *datfile = NULL;
  int speed = 20;

  static struct option long_options[] = {
    /* These options set a flag. */
    {"verbose", no_argument, &verbose_flag, 1},
    {"brief",   no_argument, &verbose_flag, 0},
    {"keepalive",  no_argument, &keep_alive, 1},
    {"terminate",  no_argument, &keep_alive, 0},
    {"yaml",  required_argument, 0, 'y'},
    {"file",  required_argument, 0, 'f'},
    {"speed",  required_argument, 0, 's'},
    {"instance",  required_argument, 0, 'i'},
    {"model",  required_argument, 0, 'm'},
    {"duration",  required_argument, 0, 'd'},
    {"repeat",  required_argument, 0, 'r'},
    {"ip",  required_argument, 0, 'n'},
    {"writeframes",  required_argument, 0, 'w'},
    {0, 0, 0, 0}
  };
  /* getopt_long stores the option index here. */
  int c;

  while (1) {
    int option_index = 0;
    c = getopt_long(argc, argv, "i:y:f:r:s:vm:d:hn:w:kt:",
                    long_options, &option_index);

    /* Detect the end of the options. */
    if (c == -1) {
      break;
    }

    switch (c) {
      case 0:
        /* If this option set a flag, do nothing else now. */
        if (long_options[option_index].flag != 0) {
          break;
        }
        fprintf(stderr, "option %s", long_options[option_index].name);
        if (optarg)
          fprintf(stderr, " with arg %s", optarg);
        fprintf(stderr, "\n");
        break;

      case 'y':
        yaml = optarg;
        break;

      case 'f':
        datfile = optarg;
        break;

      case 'n':
        ip = strdup(optarg);
        break;

      case 'm':
        model = strdup(optarg);
        break;

      case 's':
        speed = atoi(optarg);
        break;

      case 'i':
        instance = atoi(optarg);
        break;

      case 'd':
        duration = atoi(optarg);
        break;

      case 'r':
        repeat = atoi(optarg);
        break;

      case 'v':
        verbose_flag = 1;
        break;

      case 'b':
        verbose_flag = 0;
        break;

      case 'w':
        writefile = optarg;
        break;

      case 'k':
        keep_alive = 1;
        break;

      case 't':
        keep_alive = 0;
        break;

      case 'h':
        usage(argv[0]);

      case '?':
        /* getopt_long already printed an error message. */
        abort();

      default:
        abort();
    }
  }

  if (!(datfile || ip) || speed <= 0 || instance <= 0 ||
      instance > MAX_INS || !model || duration <= 0 || repeat <= 0) {
    usage(argv[0]);
  }

  printf("Use Innovusion LIDAR API version=%s built_time=%s\n",
         inno_api_version(), inno_api_build_time());
  printf("LIDAR model %s\n", model);
  if (ip) {
    printf("Use live LIDAR %s\n", ip);
    if (instance != 1) {
      printf("Force instance = 1\n");
    }
    instance = 1;
    model = "";
  } else {
    printf("Process file %s\n", datfile);
    printf("Speed is %dMB/s\n", speed);
    printf("Number of LIDAR instances %d\n", instance);
  }
  if (yaml) {
    printf("Use yaml file: %s\n", yaml);
  }
  printf("Duration is %d seconds\n", duration);
  printf("Repeat %d times\n", repeat);

  if (NULL != writefile) {
    printf("Saving frame points to %s\n", writefile);
    writeFH = fopen(writefile, "w");
    if (NULL == writeFH) {
      fprintf(stderr, "Could not open %s for frame output; aborting\n",
              writefile);
      return 2;
    }
    fprintf(writeFH, "frame, points, timestamp, x, y, z, radius, reflectance,"
            " flags\r\n");
  }

  signal(SIGPIPE, SIG_IGN);
  inno_lidar_setup_sig_handler();
  inno_lidar_set_logs(-1, -1, inno_lidar_log_test_callback);
  inno_lidar_set_log_level(verbose_flag ? INNO_LOG_EVERYTHING_LEVEL :
                           INNO_LOG_INFO_LEVEL);

  int handles[MAX_INS];
  float recheck_time = 0.25;
  // Repeatedly ('repeat' times) connect to & disconnect from the lidar.
  for (int round = 0; round < repeat; round++) {
    float total_sleep = 0;
    for (int i = 0; i < instance; i++) {
      handles[i] = get_new_lidar_handle(i, speed, datfile,
                                        ip, yaml, model);
      if (handles[i] > 0) {
        inno_lidar_start(handles[i]);
        is_alive[i] = true;
      } else {
        is_alive[i] = false;
      }
    }

    while (total_sleep < duration) {
      // Periodically confirm that the sensor is still connected.
      usleep(recheck_time * 1000000);
      total_sleep += recheck_time;
      for (int i = 0; i < instance; i++) {
        if (!is_alive[i] && handles[i] > 0) {
          fprintf(stderr, "Lidar %d ended prematurely.\n", i);
          inno_lidar_stop(handles[i]);
          inno_lidar_close(handles[i]);
          handles[i] = -1;
          if (keep_alive) {
            fprintf(stderr, "Trying to restart ....\n");
            handles[i] = get_new_lidar_handle(i, speed, datfile,
                                              ip, yaml, model);
            if (handles[i] > 0) {
              is_alive[i] = true;
              inno_lidar_start(handles[i]);
              fprintf(stderr, " restart successful!\n");
            } else {
              fprintf(stderr, " could not restart, wait 1 sec\n");
              float retry_sleep = 1;
              usleep(1000000 * retry_sleep);
              total_sleep += retry_sleep;
            }
          }
        }
      }
    }

    // Clean up: stop & close the previous connection(s).
    for (int i = 0; i < instance; i++) {
      if (is_alive[i]) {
        inno_lidar_stop(handles[i]);
        inno_lidar_close(handles[i]);
        is_alive[i] = false;
        handles[i] = 0;
      }
    }
    printf("Done round %d\n", round);
  }
  if (NULL != writeFH) {
    fclose(writeFH);
  }
  return 0;
}
