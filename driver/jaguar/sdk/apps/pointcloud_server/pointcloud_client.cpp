/**
 *  Copyright (C) 2019 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#include <assert.h>
#include <ctype.h>
#include <libgen.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/select.h>

#include "src/inno_lidar_api.h"
#include "src/inno_lidar_api_experimental.h"

const double us_in_second_c = 1000000.0;
const double hundred_us_in_second_c = 10000.0;

/*******************
 * pointcloud server<->client communication protocol
 *
 * TCP protocol:
 * 1. client connects to the server TCP port:
 * 2. client sends a topic subscription string to specify which
 *      topics it wants to receive. See get_source_fd() for detail
 * 3. server sends version 4 bytes: "PS32" (only once)
 * 4. server sends variable-sized inno_frame_packet one by one. Please
 *      note there is no separator/delimiter between two inno_frame_packet, so the
 *      client needs to calculate the size of each inno_frame_packet to know its
 *      boundary. Each inno_frame_packet consists the following parts:
 *      a. length in bytes of c, 4 bytes, long in network order
 *      b. lidar handle, 4 bytes, long in network order
 *      c. length in bytes of e, 4 bytes, long in network order
 *      d. inno_frame_header structure
 *      e. length in bytes of g, 4 bytes, long in network order
 *      f. inno_point/inno_cpoint/inno_bbox/inno_text structures,
 *         (depended on the type in the inno_frame_header)
 *      Please note that the following conditions should always be true
 *       - ntohl(size_of_lidar_context) == 4
 *       - ntohl(size_of_inno_cframe) == sizeof(struct inno_cframe_header)
 *
 * UDP protocol is very similar to the TCP protocol, except that
 *  - it doesn't have TCP steps 1,2,3.
 *  - every UDP packet includes:
 *    = 4 bytes "PS32", plus
 *    = everything in step 4.
 *  - becuase of UDP packet size limit, one frame/subframe may be divided to
 *    multiple frame-sequences, which has the same frame/subframe id,
 *    but different sequence number.
 *
 * Bounding box inno_bbox in cframe
 *  - the detail definition of inno_bbox is in inno_lidar_api.h
 *  - when static perception is enabled, the cframe topic for inno_bbox is 110.
 *    To subscribe: send 'start/?subscribe_topics=110', see get_source_fd() for detail.
 *  - data member:
 *    = int32_t type, indicates the object type prediction,
 *        e.g. TYPE_PEDESTRIAN=1, see inno_lidar_api_experimental.h for detail
 *    = int32_t id, indicates the object unique ID for tracking purpose
 *    = int16_t region, means the region where object is located,
 *        e.g. 1-4 means in fence 1-4, 5 means in blindspot, 0 means others
 *    = int32_t longtitude, in 1/10000000 degree or cframe_geo_co_unit_c rad
 *    = int32_t latitude, in 1/10000000 degree or cframe_geo_co_unit_c rad
 *******************/

/*******************
 * network helper functions
 *******************/

/****
 *  write full buf to fd before return
 */
static int write_buffer_to_socket_full(int fd, const char *buf,
                                       ssize_t input_len) {
  ssize_t len = input_len;
  while (len > 0) {
    ssize_t written = write(fd, buf, len);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        return written;
      }
    } else if (written == 0) {
      return -1;
    }
    buf += written;
    len -= written;
  }
  return input_len;
}


/****
 *  get fd from udp address/port
 */
static int bind_udp_addr_port(const char *address,
                              uint16_t port) {
  int sockfd;
  struct sockaddr_in servaddr;
  // Creating socket file descriptor
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  memset(&servaddr, 0, sizeof(servaddr));

  // Filling server information
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(port);

  // Bind the socket with the server address
  if (bind(sockfd, (const struct sockaddr *)&servaddr,
           sizeof(servaddr)) < 0 ) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }
  return sockfd;
}


/****
 *  get fd from tcp address/port
 */
static int connect_addr_port(const char *address,
                             uint16_t port,
                             double read_timeout_sec,
                             int recv_buffer_size) {
  int fd;

  struct sockaddr_in serv_addr;

  if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    perror("Error: Could not create socket\n");
    return -1;
  }

  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  if (inet_pton(AF_INET, address, &serv_addr.sin_addr) <= 0) {
    perror("inet_pton error\n");
    return -2;
  }

  {
    int one = 1;
    socklen_t optlen = sizeof(one);
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, optlen) < 0) {
      perror("cannot setsockopt keepalive\n");
      close(fd);
      return -3;
    }
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) < 0) {
      perror("cannot setsockopt tcp_nodelay\n");
      close(fd);
      return -4;
    }
    #ifndef _QNX_
    #ifndef __QNX__
    if (setsockopt(fd, IPPROTO_TCP, TCP_QUICKACK, &one, sizeof(one)) < 0) {
      perror("cannot setsockopt tcp_quickack\n");
      close(fd);
      return -5;
    }
    #endif
    #endif
    int rcvbuff = recv_buffer_size;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuff, sizeof(rcvbuff)) < 0) {
      perror("cannot setsockopt rcvbuf\n");
      close(fd);
      return -6;
    }
  }

  struct timeval tv;
  tv.tv_sec = static_cast<int>(read_timeout_sec);
  tv.tv_usec = (read_timeout_sec - tv.tv_sec) * 1000000;

  if (read_timeout_sec != 0) {
    int ret = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                         reinterpret_cast<void *>(&tv), sizeof(tv));
    if (ret < 0) {
      perror("Error: setsockopt\n");
      close(fd);
      return -7;
    }
  }

  int flag;
  // set to non-blocking mode
  if ((flag = fcntl(fd, F_GETFL, NULL)) < 0) {
    perror("Error: fcntl F_GETFL\n");
    close(fd);
    return -8;
  }
  flag |= O_NONBLOCK;
  if (fcntl(fd, F_SETFL, flag) < 0) {
    perror("Error: fcntl F_SETFL\n");
    close(fd);
    return -9;
  }

  int res = connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  if (res < 0) {
    if (errno == EINPROGRESS) {
      do {
        fd_set myset;
        struct timeval *tp = NULL;
        if (read_timeout_sec > 0) {
          tp = &tv;
        }
        FD_ZERO(&myset);
        FD_SET(fd, &myset);
        res = select(fd+1, NULL, &myset, NULL, tp);
        if (res < 0) {
          if (errno != EINTR) {
            // cannot connect
            // do not print
            perror("Error: cannot select\n");
            close(fd);
            return -10;
          } else {
            continue;
          }
        } else if (res > 0) {
          // Socket selected for write
          socklen_t lon = sizeof(int);
          int valopt = 0;
          if (getsockopt(fd, SOL_SOCKET, SO_ERROR,
                         reinterpret_cast<void*>(&valopt), &lon) < 0) {
            perror("Error: getsockopt\n");
            close(fd);
            return -11;
          }
          // Check the value returned...
          if (valopt) {
            perror("getsockeopt\n");
            close(fd);
            return -12;
          } else {
            // connected
            break;
          }
        } else {
          // connect timeout
          perror("connect timeout\n");
          close(fd);
          return -13;
        }
      } while (1);
    } else {
      // cannot connect
      perror("cannot connect\n");
      close(fd);
      return -14;
    }
  } else {
    // connected
  }
  // set to blocking mode
  if ((flag = fcntl(fd, F_GETFL, NULL)) < 0) {
    perror("Error: fcntl F_GETFL\n");
    close(fd);
    return -15;
  }
  flag &= (~O_NONBLOCK);
  if (fcntl(fd, F_SETFL, flag) < 0) {
    perror("Error: fcntl F_SETFL\n");
    close(fd);
    return -16;
  }

  return fd;
}


/****
 *  get fd from connecting to remoter server or open local file
 *  for remote server, also sends subscribe topic string
 */
static int get_source_fd(bool use_udp, const char *ips,
                         int server_port,
                         const char *infile,
                         const char *subscribe_topics) {
  int fd = -1;
  if (ips) {
    printf("Use live LIDAR %s, port is %d\n", ips, server_port);
    if (!use_udp) {
      double read_timeout_sec = 10;
      int recv_buffer_size = 256 * 1024;
      fd = connect_addr_port(ips,
                             server_port,
                             read_timeout_sec,
                             recv_buffer_size);
      if (fd >= 0) {
        char buf[32*1024];
        // write subscribe_topic string to server
        // only subscribed topics will be sent by server
        // e.g.:
        //   'start/?subscribe_topics=110' subscribes
        //      only bounding-box topics
        //   'start/?subscribe_topics=110, 120' subscribes
        //      bounding box and text topics
        //   'start/?subscribe_topics=' subscribes
        //      all topics
        printf("subscribe topic %s\n", subscribe_topics);
        snprintf(buf, sizeof(buf),
                 "start/?subscribe_topics=%s\n",
                 subscribe_topics);
        buf[sizeof(buf) - 1] = 0;
        int r = write_buffer_to_socket_full(fd, buf,
                                            strlen(buf));
        if (r < 0) {
          close(fd);
          fd = -1;
        }
      }
    } else {
      fd = bind_udp_addr_port(ips,
                              server_port);
    }
  } else {
    printf("Read file %s\n", infile);
    fd = open(infile, O_RDONLY);
  }
  return fd;
}


/****
 * read input_len bytes from fd to buf,
 * return input_len or < 0 to indicate error
 */
static int read_buffer_from_fd_full(int fd, void *buf, ssize_t input_len) {
  ssize_t len = input_len;
  while (len > 0) {
    ssize_t r = read(fd, buf, len);
    if (r < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        return r;
      }
    } else if (r == 0) {
      return -1;
    }
    buf = (reinterpret_cast<char*>(buf)) + r;
    len -= r;
  }
  return input_len;
}


/****
 * read data from fd to *buf. It is expected that the data from fd are organized as:
 *   part A (4 bytes) store length of the part B in network order
 *   part B (variable size) store data
 * return length of part B or < 0 to indicate error,
 *   in no error cases, memory block that is
 *   big enough to hold part B is malloced and its address is
 *   stored in *buf
 */
static int read_buffer_from_socket_with_length(int fd, void **buf) {
  uint32_t len = 0;
  int ret = read_buffer_from_fd_full(fd,
                                     reinterpret_cast<void*>(&len),
                                     sizeof(len));
  if (ret >= 0) {
    len = static_cast<uint32_t>(ntohl(len));
    if (len == 0) {
      *buf = NULL;
      return len;
    }
    *buf = malloc(len);
    ret = read_buffer_from_fd_full(fd, *buf, len);
    if (ret >= 0) {
      return len;
    } else {
      free(*buf);
      *buf = NULL;
      return ret;
    }
  } else {
    return ret;
  }
}


/*******************
 * csv helper functions
 *******************/
/****
 * setup output CSV file fd
 */
static FILE *setup_csv_fd(const char *csv_filename) {
  if (csv_filename == NULL) {
    return NULL;
  }
  int csv_file_fd = open(csv_filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (csv_file_fd < 0) {
    fprintf(stderr, "cannot open %s. %s\n", csv_filename, strerror(errno));
    exit(-1);
  }
  FILE *csv_file = fdopen(csv_file_fd, "w");
  if (csv_file == NULL) {
    fprintf(stderr, "cannot fdopen %s. %s\n", csv_filename, strerror(errno));
    exit(-2);
  }
  return csv_file;
}


/****
 * write inno_point to CSV file fd
 */
static void write_points_to_csv_file(FILE *file,
                                     int lidar_handle,
                                     int topic,
                                     uint64_t idx,
                                     uint16_t sub_idx,
                                     uint16_t seq,
                                     int points_number,
                                     double ts_start_s,
                                     struct inno_point *points) {
  if (ftell(file) == 0) {
    fprintf(file, "lidar, topic, frame, sub-frame, seq, "
            "point, timestamp, x, y, z, radius, reflectance, flags, "
            "scanid, scanidx\n");
  }
  for (int i = 0; i < points_number; i++) {
    struct inno_point *p = &points[i];
    fprintf(file, "%d, %d, %lu, %u, %u, %u, %.7f, "
            "%.3f, %.3f, %.3f, %.3f, %u, %u, %u %u\n",
            lidar_handle, topic, idx, sub_idx, seq,
            i,
            p->ts_100us/hundred_us_in_second_c + ts_start_s,
            p->x, p->y, p->z,
            p->radius,
            static_cast<uint32_t>(p->ref),
            static_cast<uint32_t>(p->flags),
            static_cast<uint32_t>(p->scan_id),
            static_cast<uint32_t>(p->scan_idx));
  }
  return;
}


/****
 * write inno_cpoint to CSV file fd
 */
static void write_cpoints_to_csv_file(FILE *file,
                                      int lidar_handle,
                                      int topic,
                                      uint64_t idx,
                                      uint16_t sub_idx,
                                      uint16_t seq,
                                      double frame_ts_sec_start,
                                      int points_number,
                                      struct inno_cpoint *points) {
  if (ftell(file) == 0) {
    fprintf(file, "lidar, topic, frame, sub-frame, seq, "
            "point, timestamp, radius, h_angle, v_angle, reflectance, flags, "
            "scanid, scanidx\n");
  }
  for (int i = 0; i < points_number; i++) {
    struct inno_cpoint *p = &points[i];
    fprintf(file, "%d, %d, %lu, %u, %u, %u, %.7f, "
            "%.3f, %.3f, %.3f, %u, %u, %u, %u\n",
            lidar_handle, topic, idx, sub_idx, seq,
            i, p->ts_100us/hundred_us_in_second_c + frame_ts_sec_start,
            p->radius / static_cast<double>(100),
            p->h_angle * cpoint_angel_unit_c,
            p->v_angle * cpoint_angel_unit_c,
            static_cast<uint32_t>(p->ref),
            static_cast<uint32_t>(p->flags),
            static_cast<uint32_t>(p->scan_id),
            static_cast<uint32_t>(p->scan_idx));
  }
  return;
}


/****
 * write bbox to CSV file fd
 */
static void write_bbox_to_csv_file(FILE *file,
                                   int lidar_handle,
                                   int topic,
                                   uint64_t idx,
                                   uint16_t sub_idx,
                                   uint16_t seq,
                                   double frame_ts_sec_start,
                                   int box_count,
                                   struct inno_bbox *bbox) {
  if (ftell(file) == 0) {
    fprintf(file, "lidar, topic, frame, sub-frame, seq, bbox, "
            "timestamp, "
            "x, y, z, "
            "speed_x, speed_y, speed_z, "
            "width, height, depth, "
            "label, value, type, id, "
            "confidence, priority, "
            "longtitude, latitude, elevation, "
            "region, pt_count, cluster"
            "\n");
  }
  for (int i=0; i < box_count; ++i) {
    inno_bbox *b = &bbox[i];
    fprintf(file, "%d, %d, %lu, %u, %u, %u, "
            "%.7f, "
            "%.3f, %.3f, %.3f, "
            "%.3f, %.3f, %.3f, "
            "%.3f, %.3f, %.3f, "
            "%d, %d, %d, %d, "
            "%hd, %hd, "
            "%d, %d, %hd,"
            "%hd, %hu, %u"
            "\n",
            lidar_handle, topic,
            idx, sub_idx, seq, i,
            b->ts_100us/hundred_us_in_second_c + frame_ts_sec_start,
            b->x, b->y, b->z,
            b->speed_x, b->speed_y, b->speed_z,
            b->width, b->height, b->depth,
            b->label, b->value, b->type, b->id,
            b->confidence, b->priority,
            b->longtitude, b->latitude, b->elevation,
            b->region, b->pc_count, b->cluster_id);
  }
  return;
}


/*******************
 * functions to process frame/cframe
 *******************/

/****
 * process inno_point in cframe
 */
static int process_cframe_point(FILE *csv_file, int lidar_handle,
                                inno_cframe_header *frame,
                                struct inno_point *points) {
  double ts_start_s = frame->ts_us_start/us_in_second_c;
  for (unsigned int i = 0; i < 2 && i < frame->item_number; i++) {
    if (i == 1) {
      i = frame->item_number - 1;
    }
    inno_point *p = &points[i];
    printf("[c-%d]   *point[%5u] timestamp=%f "
           "x=%f y=%f z=%f radius=%f reflectance=%d flags=%d\r\n",
           lidar_handle, i,
           p->ts_100us/hundred_us_in_second_c+ ts_start_s,
           p->x, p->y, p->z, p->radius,
           static_cast<int>(p->ref), static_cast<int>(p->flags));
  }
  if (csv_file != NULL) {
    write_points_to_csv_file(csv_file, lidar_handle, frame->topic,
                             frame->idx, frame->sub_idx,
                             frame->sub_seq, frame->item_number,
                             ts_start_s, points);
  }
  return 0;
}


/****
 * process inno_cpoint in cframe
 */
static int process_cframe_cpoint(FILE *csv_file,
                                 int lidar_handle,
                                 inno_cframe_header *frame,
                                 struct inno_cpoint *points) {
  double ts_start_s = frame->ts_us_start/us_in_second_c;
  for (unsigned int i = 0; i < 2 && i < frame->item_number; i++) {
    if (i == 1) {
      i = frame->item_number - 1;
    }
    inno_cpoint *p = &points[i];
    printf("[c-%d]   *point[%5u] timestamp=%f "
           "radius=%f h_angle=%f v_angle=%f reflectance=%d flags=%d\r\n",
           lidar_handle, i,
           p->ts_100us / hundred_us_in_second_c + ts_start_s,
           p->radius / static_cast<double>(100),
           p->h_angle * cpoint_angel_unit_c,
           p->v_angle * cpoint_angel_unit_c,
           static_cast<int>(p->ref), static_cast<int>(p->flags));
  }
  if (csv_file != NULL) {
    write_cpoints_to_csv_file(csv_file, lidar_handle, frame->topic,
                              frame->idx, frame->sub_idx,
                              frame->sub_seq, ts_start_s,
                              frame->item_number, points);
  }
  return 0;
}


/****
 * process inno_bboxes in cframe
 */
static int process_cframe_bbox(FILE *csv_file,
                               int lidar_handle,
                               inno_cframe_header *frame,
                               struct inno_bbox *bbox) {
  double ts_start_s = frame->ts_us_start/us_in_second_c;
  for (unsigned int i = 0; i < 2 && i < frame->item_number; i++) {
    if (i == 1) {
      i = frame->item_number - 1;
    }
    inno_bbox *p = &bbox[i];
    printf("[c-%d]   *bbox[%5u] timestamp=%f "
           "x=%f y=%f z=%f label=%d value=%d type=%d id=%d "
           "confidence=%hd priority=%hd longtitude=%d latitude=%d "
           "elevation=%hd\r\n",
           lidar_handle, i,
           p->ts_100us/hundred_us_in_second_c+ ts_start_s,
           p->x, p->y, p->z, p->label, p->value, p->type,
           p->id, p->confidence, p->priority,
           p->longtitude, p->latitude, p->elevation);
  }
  if (csv_file != NULL) {
    write_bbox_to_csv_file(csv_file, lidar_handle,
                           frame->topic,
                           frame->idx, frame->sub_idx,
                           frame->sub_seq, ts_start_s,
                           frame->item_number, bbox);
  }
  return 0;
}


/****
 * process data in cframe
 */
static int process_cframe(int lidar_ctx, void *buf2,
                          void *buf3,
                          int buf3_len,
                          FILE *csv_file,
                          FILE *csv_file_bbox) {
  struct inno_cframe_header *frame = (struct inno_cframe_header *)buf2;
  printf("[c-%d] got cframe topic=%d type=%d idx=%lu-%d-%d: "
         "timestamp=%f, %u items. %s-last subframe in frame, "
         "%s-last sequence in sub-frame\r\n",
         lidar_ctx, frame->topic, frame->type,
         frame->idx, frame->sub_idx, frame->sub_seq,
         frame->ts_us_end/us_in_second_c, frame->item_number,
         (frame->flags & 2)? "not" : "is",
         (frame->flags & 1)? "not" : "is");

  int unit_size = 0;
  switch (frame->type) {
    case INNO_CFRAME_POINT:
      unit_size = sizeof(struct inno_point);
      break;
    case INNO_CFRAME_CPOINT:
      unit_size = sizeof(struct inno_cpoint);
      break;
    case INNO_CFRAME_BBOX:
      unit_size = sizeof(struct inno_bbox);
      break;
    case INNO_CFRAME_TEXT:
      unit_size = sizeof(struct inno_text);
      break;
    case INNO_CFRAME_ALARM:
      unit_size = sizeof(struct inno_alarm_message);
      break;
    default:
      fprintf(stderr, "invalid type %d\n", frame->type);
      return -1;
  }
  if (buf3_len % unit_size != 0) {
    fprintf(stderr, "invalid size %d vs %d\n", buf3_len, unit_size);
    return -2;
  }
  unsigned int unit_n = buf3_len / unit_size;
  if (unit_n != frame->item_number) {
    fprintf(stderr, "item_number mismatch type=%d %u vs %u\n",
            frame->type, unit_n, frame->item_number);
    return -1;
  }

  if (frame->type == INNO_CFRAME_CPOINT) {
    process_cframe_cpoint(csv_file, lidar_ctx, frame,
                          (struct inno_cpoint*)buf3);
  } else if (frame->type == INNO_CFRAME_POINT) {
    process_cframe_point(csv_file, lidar_ctx, frame,
                         (struct inno_point*)buf3);
  } else if (frame->type == INNO_CFRAME_BBOX) {
    process_cframe_bbox(csv_file_bbox, lidar_ctx, frame,
                        (struct inno_bbox*)buf3);
  } else if (frame->type == INNO_CFRAME_ALARM) {
    // alarm messages are sent to client after wrapped in cframe
    // one alarm message per cframe
    if (unit_n != 1) {
      fprintf(stderr, "INNO_CFRAME_ALARM invalid item number %u\n",
              unit_n);
      return -1;
    }
    struct inno_alarm_message &a = ((struct inno_alarm_message*)buf3)[0];
    printf("alarm message: n=%d sz=%d level=%d type=%d message=%s\n",
           unit_n, unit_size,
           a.level, a.code, a.message);
  } else {
    printf("ignore type %d for now.\n", frame->type);
  }
  return 0;
}

/*******************
 * functions to read frame/cframe
 *******************/

/****
 * read cframes from tcp connection
 */
static int read_cframes_tcp(int fd, FILE *csv_file, FILE *csv_file_bbox) {
  /*
    each cframe from network has:
    a. length-of-part-b, 4 bytes in network order
    b. lidar handle, 4 bytes in network order
    c. length-of-part-d, 4 bytes in network order
    d. inno_cframe_header structure
    e. length-of-part-f, 4 bytes in network order
    f. array of inno_point or array of inno_cpoint.
   */
  void *buf1 = NULL;
  void *buf2 = NULL;
  void *buf3 = NULL;
  int ret = 0;
  do {
    ret = read_buffer_from_socket_with_length(fd, &buf1);
    if (ret != 4 || buf1 == NULL) {
      if (ret == -1 || ret == 0) {
        ret = -1;
        fprintf(stderr, "end of source\n");
      } else {
        fprintf(stderr, "bad cdata ctx ret=%d\n", ret);
      }
      break;
    }
    int lidar_ctx = ntohl(*(unsigned int*)buf1);
    ret = read_buffer_from_socket_with_length(fd, &buf2);
    if (ret != sizeof(struct inno_cframe_header) || buf2 == NULL) {
      fprintf(stderr, "bad data frame ret=%d\n", ret);
      ret = -2;
      break;
    }

    ret = read_buffer_from_socket_with_length(fd, &buf3);
    if (ret < 0 || (buf3 == NULL && ret != 0)) {
      fprintf(stderr, "bad data points ret=%d\n", ret);
      break;
    }
    ret = process_cframe(lidar_ctx, buf2, buf3, ret, csv_file, csv_file_bbox);
  } while (0);

  free(buf1);
  free(buf2);
  free(buf3);

  return ret;
}


/****
 * read cframe from buff received from udp
 */
static int read_cframe_udp(char *buff, int len,
                           FILE *csv_file, FILE *csv_file_bbox) {
  void *buf1 = NULL;
  void *buf2 = NULL;
  void *buf3 = NULL;
  int ret = 0;
  unsigned point_offset = 4 + 4 + 4 + sizeof(struct inno_cframe_header) + 4;
  buf1 = buff + 4;
  buf2 = buff + 4 + 4 + 4;
  buf3 = buff + point_offset;
  unsigned int l = ntohl(*(unsigned int*)buff);
  if (4 != l) {
    fprintf(stderr, "invalid packet=%u\n", l);
    return -1;
  }
  l = ntohl(*(unsigned int*)(buff + 8));
  if (sizeof(struct inno_cframe_header) != l) {
    fprintf(stderr, "b invalid packet=%u\n", l);
    return -2;
  }
  l = ntohl(*(unsigned int*)(buff + point_offset - 4));
  if (len - point_offset != l) {
    fprintf(stderr, "c invalid packet %u %u\n", len - point_offset, l);
    return -3;
  }

  ret = process_cframe(ntohl(*(unsigned int*)buf1), buf2, buf3,
                       len - point_offset, csv_file, csv_file_bbox);
  return ret;
}


/*******************
 * tcp/udp read and process functions
 *******************/

/****
 * process data from tcp
 */
static int read_and_process_tcp(int fd, FILE *csv_file, FILE *csv_file_bbox) {
  char buff_4[4];
  if (4 != read_buffer_from_fd_full(fd, buff_4, sizeof(buff_4))) {
    fprintf(stderr, "Invalid source (%s) -- %s::%s(%d)\n", buff_4, __FILE__,
                                                           __func__,
                                                           __LINE__);
    return -1;
  }
  int ret = 0;
  if (strncmp(buff_4, "PS32", 4) == 0) {
    ret = 0;
    while (ret == 0) {
      ret = read_cframes_tcp(fd, csv_file, csv_file_bbox);
    }
    ret -= 200;
  } else {
    fprintf(stderr, "Invalid source (%s) -- %s::%s(%d)\n", buff_4, __FILE__,
                                                           __func__,
                                                           __LINE__);
    return -4;
  }
  return ret;
}


/****
 * process data from udp
 */
static int read_and_process_udp(int fd, FILE *csv_file, FILE *csv_file_bbox) {
  char buff[1500];
  struct sockaddr_in cliaddr;
  socklen_t len = sizeof(cliaddr);
  int n;
  int ret = 0;
  while (1) {
    n = recvfrom(fd, buff, sizeof(buff),
                 MSG_WAITALL, (struct sockaddr *)&cliaddr,
                 &len);
    if (n < 0) {
      perror("recvfrom");
      return -1;
    }
    if (8 >= n) {
      fprintf(stderr, "Invalid source\n");
      return -2;
    }
    if (strncmp(buff, "PS32", 4) == 0) {
      ret = read_cframe_udp(buff + 4, n - 4, csv_file, csv_file_bbox);
      if (ret < 0) {
        ret -= 300;
      }
    } else {
      fprintf(stderr, "Invalid source\n");
      return -4;
    }
  }
  return ret;
}

char *trimwhitespace(char *str) {
  char *end;
  while (isspace((unsigned char)*str)) str++;

  if (*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while (end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator character
  end[1] = '\0';

  return str;
}

static void print_basis(const char *argv0) {
  // Write SDK ver & build tag to stdout, if they can be determined....
  char app[PATH_MAX];
  strncpy(app, argv0, sizeof(app));
  char buff_filename[PATH_MAX];
  char tag_dir[PATH_MAX];
  if (strncmp(argv0, ".", 1) == 0) {
    snprintf(tag_dir, sizeof(tag_dir), "%s/../..", dirname(app));
  } else {
    snprintf(tag_dir, sizeof(tag_dir), "%s", dirname(dirname(dirname(app))));
  }
  // printf("Looking for SDK_VERSION and BUILD_TAG in %s\n", tag_dir);
  snprintf(buff_filename, sizeof(buff_filename), "%s/SDK_VERSION", tag_dir);
  char buff_sdk[128];
  bool have_sdk_ver = false;
  if (access(buff_filename, F_OK) != -1) {
    FILE * f = fopen(buff_filename, "r");
    if (f) {
      size_t bytes_read = fread(buff_sdk, 1, sizeof(buff_sdk), f);
      fclose(f);
      have_sdk_ver = bytes_read > 0;
    }
  }

  snprintf(buff_filename, sizeof(buff_filename), "%s/BUILD_TAG", tag_dir);
  char buff_tag[128];
  bool have_build_tag = false;
  if (access(buff_filename, F_OK) != -1) {
    FILE * f = fopen(buff_filename, "r");
    if (f) {
      size_t bytes_read = fread(buff_tag, 1, sizeof(buff_tag), f);
      fclose(f);
      have_build_tag = bytes_read > 0;
    }
  }
  if (have_sdk_ver || have_build_tag) {
    printf("PCC_app based on ");
    if (have_sdk_ver) {
      char* disp_sdk = trimwhitespace(buff_sdk);
      printf("SDK ver: %s ", disp_sdk);
    }
    if (have_build_tag) {
      char* disp_tag = trimwhitespace(buff_tag);
      printf("(build tag: %s)", disp_tag);
    }
    printf("\n");
  } else {
    printf("Unknown SDK version & build tag.\n");
  }
}

static void usage(const char *argv0) {
  fprintf(stderr,
          "usage:\n\t%s -n SERVER_IP "
          "[--udp] [--server-port PORT] "
          "[--output-csv CSV_FILE] "
          "[--output-csv-bbox CSV_FILE] "
          "[--subscribe-topics topic1,topic2,...] "
          "[--keep-alive]\nOR\n"
          "\t%s -f INPUT_FILE "
          "[--subscribe-topics topic1,topic2,...] "
          "[--output-csv CSV_FILE] "
          "[--output-csv-bbox CSV_FILE]"
          "\n",
          argv0, argv0);
  exit(1);
}

/*******************
 * main function
 *******************/
int main(int argc, char *argv[]) {
  const char *ips = NULL;
  const char *infile = NULL;
  int server_port = 8001;
  bool use_udp = false;
  const char *csv_filename = NULL;
  const char *csv_filename_bbox = NULL;
  bool keep_alive = false;
  // subscribe all static topics, bbox-topic, text-topic, alarm-topic
  // const char *subscribe_topics = "-2,101,110,120";
  // empty string means subscribe everything
  const char *subscribe_topics = "";

  static struct option long_options[] = {
    /* These options set a flag. */
    {"file",  required_argument, 0, 'f'},
    {"output-csv",  required_argument, 0, 'o'},
    {"output-csv-bbox",  required_argument, 0, 'b'},
    {"ip",  required_argument, 0, 'n'},
    {"udp",  no_argument, 0, 'u'},
    {"server-port",  required_argument, 0, 'p'},
    {"subscribe-topics",  required_argument, 0, 's'},
    {"keep-alive",  no_argument, 0, 'k'},
    {"help",  no_argument, 0, 'h'},
    {0, 0, 0, 0}
  };

  /* getopt_long stores the option index here. */
  int c;

  while (1) {
    int option_index = 0;
    c = getopt_long(argc, argv, "vf:n:p:o:b:u:k:hs:",
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
        printf("option %s", long_options[option_index].name);
        if (optarg)
          printf(" with arg %s", optarg);
        printf("\n");
        break;

      case 'n':
        ips = strdup(optarg);
        break;

      case 'f':
        infile = strdup(optarg);
        break;

      case 'p':
        server_port = atoi(optarg);
        break;

      case 'o':
        csv_filename = optarg;
        break;

      case 'b':
        csv_filename_bbox = optarg;
        break;

      case 's':
        subscribe_topics = strdup(optarg);
        break;

      case 'u':
        use_udp = true;
        break;

      case 'k':
        printf("Setting keep-alive to true; retry server if disconnected\n");
        keep_alive = true;
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

  print_basis(argv[0]);
  if (ips == NULL && infile == NULL) {
    usage(argv[0]);
    return -1;
  }

  signal(SIGPIPE, SIG_IGN);
  FILE *csv_file = setup_csv_fd(csv_filename);
  FILE *csv_file_bbox = setup_csv_fd(csv_filename_bbox);

  int fd = -1;
  int process_ret = -1;
  int retry = 1;
  do {
    fd = get_source_fd(use_udp, ips, server_port,
                       infile, subscribe_topics);
    if (fd >= 0) {
      if (use_udp) {
        process_ret = read_and_process_udp(fd, csv_file, csv_file_bbox);
      } else {
        process_ret = read_and_process_tcp(fd, csv_file, csv_file_bbox);
      }
      close(fd);
      fd = -1;
    } else {
      fprintf(stderr, "cannot open source!\n");
      process_ret = -1;
    }
    if (keep_alive) {
      printf("Disconnected (%d) -- restart %d ***** ***** ***** ***** *****\n",
             process_ret, retry);
      ++retry;
    }
  } while (keep_alive);
  if (process_ret == -201) {
    return 0;
  }
  return process_ret;
}
