/**
 *  Copyright (C) 2019 - Innovusion Inc.
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
#include <inno_lidar_api_experimental.h>

FILE *writeFH = NULL;
const int MAX_INS = 10;
bool is_alive[MAX_INS];
static const double us_in_second_c = 1000000.0;
static const double hundred_us_in_second_c = 10000.0;

int process_cframe(int lidar_handle, struct inno_cframe_header *frame) {
  double ts_s_start = frame->ts_us_start / us_in_second_c;
  printf("[LIDAR-%d] got frame %lu-%d: topic=%u "
         "timestamp=%f-%f, type=%d, %d items\n",
         lidar_handle, frame->idx, frame->sub_idx,
         frame->topic,
         ts_s_start,
         frame->ts_us_end/us_in_second_c,
         frame->type,
         frame->item_number);

  if (frame->type == INNO_CFRAME_POINT) {
    // For performance, only print two points to stdout.
    for (unsigned int i = 0; i < 2 && i < frame->item_number; i++) {
      if (i == 1) {
        i = frame->item_number - 1;
      }
      inno_point *p = &frame->points[i];
      printf("[LIDAR-%d]   point[%5u] timestamp=%f x=%f y=%f z=%f "
             "radius=%f reflectance=%d flags=%d\n",
             lidar_handle, i,
             p->ts_100us / hundred_us_in_second_c + ts_s_start,
             p->x, p->y, p->z, p->radius,
             static_cast<int>(p->ref), static_cast<int>(p->flags));
    }

    // If there's a file handle, write all the points to the file.
    // Again for performance, only write the significant digits (distances
    // are accurate to cm).
    if (NULL != writeFH) {
      for (unsigned int i = 0; i < frame->item_number; i++) {
        inno_point *p = &frame->points[i];
        // frame, points, timestamp, x, y, z, radius, reflectance, flags
        fprintf(writeFH, "%lu, %5u, %f, %.2f, %.2f, %.2f, %.2f, %d, %d\n",
                frame->idx, i,
                p->ts_100us/hundred_us_in_second_c + ts_s_start,
                p->x, p->y, p->z, p->radius,
                static_cast<int>(p->ref), static_cast<int>(p->flags));
      }
    }
  } else if (frame->type == INNO_CFRAME_ALARM) {
    printf("[LIDAR-%d] receive alarm: level=%d "
           "code=%d message=%s\n",
           lidar_handle, frame->alarm[0].level,
           frame->alarm[0].code, frame->alarm[0].message);
  } else {
    printf("[LIDAR-%d] ignore\n", lidar_handle);
  }

  return 0;
}

void inno_lidar_log_test_callback(enum inno_log_level level,
                                  const char *header1,
                                  const char *header2, const char *msg) {
  fprintf(stderr, "level=%d header1=%s header2=%s body=%s\n",
          level, header1, header2, msg);
}

int get_new_lidar_handle(int idx, int speed, const char *datfile,
                         const char *ip, const char *yaml, const char *model,
                         const char *filter) {
  char buf[32];
  snprintf(buf, sizeof(buf), "test-lidar-%d", idx);
  int handle;
  if (ip) {
    handle = inno_lidar_open_live(buf, ip, 8001, true);
  } else {
    handle = inno_lidar_open_file(buf, datfile, speed, 0, 0);
  }
  assert(handle > 0);
  // load filter
  if (filter) {
    int r = inno_lidar_load_filter(handle, filter);
    assert(r == 0);
  }
  int params = inno_lidar_set_parameters(handle, model, "", yaml);
  assert(params == 0);
  inno_lidar_set_callbacks(handle, NULL,
                           NULL, NULL,
                           NULL);
  inno_lidar_set_reflectance_mode(handle, REFLECTANCE_MODE_REFLECTIVITY);
  return handle;
}

void usage(const char *argv0) {
  fprintf(stderr, "usage: %s {-f data-file | -n LIDAR_IP} [-m model] "
          "[-y yaml-file] [-s speed] "
          "[-v | -b] [-w pc-output-file.csv] "
          "[-k | -t] [--static-perception] [-F filter-lib-path]\n",
          argv0);
  fprintf(stderr, "If the '-f data-file' argument is used, '-y yaml-file' must"
          "also be supplied.");
  exit(1);
}

int main(int argc, char *argv[]) {
  // Variables appropriate for both live & replaying from file.
  const char *writefile = NULL;
  int keep_alive = 0;
  int verbose_flag = 0;
  const char *ip = NULL;
  const char *yaml = NULL;
  const char *model = "REV_E";
  int do_perception = 0;

  // Variables needed for running from a previously saved .dat file.
  const char *datfile = NULL;
  const char *filter = NULL;
  int speed = 20;

  static struct option long_options[] = {
    /* These options set a flag. */
    {"verbose", no_argument, &verbose_flag, 1},
    {"brief",   no_argument, &verbose_flag, 0},
    {"keepalive",  no_argument, &keep_alive, 1},
    {"terminate",  no_argument, &keep_alive, 0},
    {"static-perception",  no_argument, &do_perception, 1},
    {"yaml",  required_argument, 0, 'y'},
    {"file",  required_argument, 0, 'f'},
    {"speed",  required_argument, 0, 's'},
    {"model",  required_argument, 0, 'm'},
    {"filter",  required_argument, 0, 'F'},
    {"ip",  required_argument, 0, 'n'},
    {"writeframes",  required_argument, 0, 'w'},
    {0, 0, 0, 0}
  };
  /* getopt_long stores the option index here. */
  int c;

  while (1) {
    int option_index = 0;
    c = getopt_long(argc, argv, "y:f:s:vm:hn:w:kt:F:",
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

      case 'F':
        filter = optarg;
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

  if (!(datfile || ip) || speed <= 0 || !model) {
    usage(argv[0]);
  }

  printf("Use Innovusion LIDAR API version=%s built_time=%s\n",
         inno_api_version(), inno_api_build_time());
  printf("LIDAR model %s\n", model);
  if (ip) {
    printf("Use live LIDAR %s\n", ip);
    model = "";
  } else {
    printf("Process file %s\n", datfile);
    printf("Speed is %dMB/s\n", speed);
  }
  if (yaml) {
    printf("Use yaml file: %s\n", yaml);
  }

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

  int handle;
  handle = get_new_lidar_handle(0, speed, datfile,
                                ip, yaml, model, filter);
  struct inno_cframe_header *cframe;
  int ret = inno_lidar_set_reflectance_mode(handle,
                                            REFLECTANCE_MODE_INTENSITY);
  assert(ret == 0);
  if (do_perception) {
    // ret = inno_lidar_set_perception_mode(handle, INNO_STATIC_PERCEPTION);
    assert(ret == 0);
  }
  do {
    cframe = NULL;
    ret = inno_lidar_sync_read(handle, 0.2,
                               INNO_CFRAME_POINT, &cframe);
    if (ret == 0) {
      if (cframe) {
        process_cframe(0, cframe);
        free(cframe);
      } else {
        fprintf(stderr, "impossible!!!\n");
        break;
      }
    } else if (ret == 1) {
      fprintf(stderr, "bad handle %d, stop\n", handle);
      break;
    } else if (ret == -1) {
      fprintf(stderr, "has critical error, stop\n");
      break;
    } else if (ret == -2) {
      fprintf(stderr, "timeout, continue\n");
    } else {
      fprintf(stderr, "bad return %d, stop\n", ret);
      break;
    }
  } while (1);
  inno_lidar_close(handle);

  if (NULL != writeFH) {
    fclose(writeFH);
  }
  return 0;
}
