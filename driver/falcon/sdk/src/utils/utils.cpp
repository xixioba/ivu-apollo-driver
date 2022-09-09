/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "utils/utils.h"

#include <fcntl.h>
#include <pthread.h>
#ifndef __MINGW64__
#include <netinet/in.h>
#else
#include <winsock2.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#if defined(__i386__) || defined(__x86_64__)
#ifndef  __MINGW64__
#include <x86intrin.h>
#else
#include <intrin.h>
#endif
#endif

#include <vector>

#include "utils/log.h"
#include "utils/net_manager.h"

namespace innovusion {

const char *InnoUtils::white_space_ = "\t\n\v\f\r ";
const char* InnoUtils::kInnoSeparator = "\a\a";

void InnoUtils::set_self_thread_priority(int priority) {
#ifndef __MINGW64__
  struct sched_param params;
  struct sched_param current_params;
  int policy;
  int current_policy;
  pthread_t this_thread = pthread_self();

  int ret = pthread_getschedparam(this_thread, &current_policy,
                                  &current_params);
  if (ret) {
    inno_log_error_errno("getschedparam %d", ret);
    return;
  } else {
    inno_log_trace("thread current priority is %d (%d), target is %d",
                   current_params.sched_priority, current_policy,
                   priority);
  }
  if (priority == 0) {
    return;
  } else if (priority > 0) {
    policy = SCHED_FIFO;
    params.sched_priority = current_params.sched_priority + priority;
  } else {
    policy = SCHED_IDLE;
    params.sched_priority = 0;
  }
  if (params.sched_priority > 99) {
    params.sched_priority = 99;
  }
  if (params.sched_priority < 0) {
    params.sched_priority = 0;
  }
  ret = pthread_setschedparam(this_thread, policy, &params);
  if (ret != 0) {
    inno_log_warning_errno("setschedparam(%d)", params.sched_priority);
    return;
  }
  ret = pthread_getschedparam(this_thread, &current_policy,
                              &current_params);
  if (ret) {
    inno_log_error_errno("getschedparam 2 %d", ret);
  } else {
    if (current_params.sched_priority != params.sched_priority) {
      inno_log_error("current priority=%d (%d), target is %d",
                     current_params.sched_priority, current_policy,
                     params.sched_priority);
    } else {
      inno_log_info("set thread priority to %d (%d)",
                    current_params.sched_priority, current_policy);
    }
  }
#endif  // __MINGW64__
  return;
}

#ifdef __MINGW64__
__inline static uint32_t __attribute__((__gnu_inline__,
                                        __always_inline__,
                                        __artificial__))
_mm_crc32_u8(uint32_t __C, uint8_t __V) {
  return __builtin_ia32_crc32qi(__C, __V);
}

__inline static uint32_t __attribute__((__gnu_inline__,
                                        __always_inline__,
                                        __artificial__))
_mm_crc32_u16(uint32_t __C, uint16_t __V) {
  return __builtin_ia32_crc32hi(__C, __V);
}

__inline static uint32_t __attribute__((__gnu_inline__,
                                        __always_inline__,
                                        __artificial__))
_mm_crc32_u32(uint32_t __C, uint32_t __V) {
  return __builtin_ia32_crc32si(__C, __V);
}
#endif

uint32_t InnoUtils::crc32_do(uint32_t crc, const void*
                             const buf, const size_t buf_len) {
  const uint8_t* p = reinterpret_cast<const uint8_t *>(buf);
  register uint32_t l = crc;

#if (defined(__i386__) || defined(__x86_64__)) && (!defined(__MINGW64__dd))
  for (size_t i = 0; i < (buf_len / sizeof(uint32_t)); ++i) {
    l = _mm_crc32_u32(l, *(const uint32_t *)p);
    p += sizeof(uint32_t);
  }
  if (buf_len & sizeof(uint16_t)) {
    l = _mm_crc32_u16(l, *(const uint16_t *)p);
    p += sizeof(uint16_t);
  }
  if (buf_len & sizeof(uint8_t)) {
    l = _mm_crc32_u8(l, *(const uint8_t *)p);
  }
#else

#if defined(__aarch64__)
  // from https://www.programmersought.com/article/13506713080/
#define CRC32X(crc, value) __asm__("crc32x %w[c], %w[c], %x[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32W(crc, value) __asm__("crc32w %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32H(crc, value) __asm__("crc32h %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32B(crc, value) __asm__("crc32b %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CX(crc, value) __asm__("crc32cx %w[c], %w[c], %x[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CW(crc, value) __asm__("crc32cw %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CH(crc, value) __asm__("crc32ch %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
#define CRC32CB(crc, value) __asm__("crc32cb %w[c], %w[c], %w[v]":[c]"+r"(crc):[v]"r"(value))  // NOLINT
  // Use local variables, use register optimization
  register size_t len = buf_len;

#define STEP1 do {                                              \
    CRC32CB(l, *p++);                                           \
    len--;                                                      \
} while (0)

#define STEP2 do {                                              \
    CRC32CH(l, *(uint16_t *)p);                                 \
    p += 2;                                                     \
    len -= 2;                                                   \
} while (0)

#define STEP4 do {                                              \
    CRC32CW(l, *(uint32_t *)p);                                 \
    p += 4;                                                     \
    len -= 4;                                                   \
} while (0)

#define STEP8 do {                                              \
    CRC32CX(l, *(uint64_t *)p);                                 \
    p += 8;                                                     \
    len -= 8;                                                   \
} while (0)

  // 512 way loop inline expansion
  while (len >= 512) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }
  // Use if to judge directly, the effect will be higher
  if (len >= 256) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 128) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 64) {
    STEP8; STEP8; STEP8; STEP8;
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 32) {
    STEP8; STEP8; STEP8; STEP8;
  }

  if (len >= 16) {
    STEP8; STEP8;
  }

  if (len >= 8) {
    STEP8;
  }

  if (len >= 4) {
    STEP4;
  }

  if (len >= 2) {
    STEP2;
  }

  if (len >= 1) {
    STEP1;
  }
#undef STEP8
#undef STEP4
#undef STEP2
#undef STEP1

#else
//  for (size_t i = 0; i < buf_len; ++i) {
//    l = (l << 5) + l + p[i];
//  }
#error "unsupported ARCH"

#endif  // ARM
#endif
  return l;
}

uint32_t InnoUtils::calculate_http_crc32(const char* buffer,
                                         uint32_t length,
                                         bool append) {
  uint32_t crc = InnoUtils::crc32_start();
  crc = InnoUtils::crc32_do(crc, buffer, length);
  if (append) {
    // add calc BEL 0x07 for separator
    crc = crc32_do(crc, kInnoSeparator, 2);
  }
  return crc32_end(crc);
}

int InnoUtils::verify_http_crc32(const char* buffer,
                                 const char* url) {
  std::string recv_buf(buffer);
  std::string recv_crc32;
  std::string context;
  uint32_t calc_crc32 = crc32_start();
  char ch_crc32[20] = {0};

  uint32_t buf_len = strlen(buffer);
  size_t pos = recv_buf.find("X-INNO-CRC32");
  if (pos != recv_buf.npos) {
    recv_crc32 = recv_buf.substr(pos + kInnoStrCRC32len,
                                 kInnoCRC32ValueLen);
    pos = recv_buf.find("\r\n\r\n");
    if (pos != recv_buf.npos) {
      context = recv_buf.substr(pos + 4, buf_len - pos - 4);
      if (url != NULL) {
        calc_crc32 = crc32_do(calc_crc32, url, strlen(url));
        // add calc BEL 0x07 for separator
        calc_crc32 = crc32_do(calc_crc32, kInnoSeparator, 2);
      }
      if (context.size() > 0) {
        calc_crc32 = crc32_do(calc_crc32, context.c_str(),
                              context.size());
      }
      calc_crc32 = crc32_end(calc_crc32);
      snprintf(ch_crc32, sizeof(ch_crc32), "%8x", calc_crc32);
      if (strcmp(ch_crc32, recv_crc32.c_str()) != 0) {
        inno_log_warning("CRC check failed. "
                         "calc_crc32 %s != recv_crc32 %s",
                         ch_crc32, recv_crc32.c_str());
        return -1;
      } else {
        return 0;
      }
    } else {
      inno_log_warning("Can't find \r\n\r\n");
      return -1;
    }
  } else {
    return 1;   // not check crc32 for Web
  }
}


/**
 * @Brief : check if the ip is valid
 * @param  ip           
 * @return true  ip valid            
 * @return false ip invalid          
 */
bool InnoUtils::check_ip_valid(const char *ip) {
  std::string ip_address(ip);
  if (ip_address.empty() || ip_address == kInvalidIpAddress) {
    return false;
  }

  if (ip_address == kDefaultInterface)
    return true;

  struct in_addr new_ip;
  if (NetManager::inno_inet_pton(ip_address.c_str(), &new_ip) <= 0) {
    return false;
  }

  return true;
}

int InnoUtils::open_file(const char *filename, int flag_in, int mode) {
  int flag = flag_in;
#ifdef  __MINGW64__
  flag |= O_BINARY;
#endif
  int file_fd = open(filename, flag, mode);
  if (file_fd < 0) {
    inno_log_info("cannot open %s", filename);
    const char *ppwd = getenv("PWD");
    if (ppwd != NULL) {
      std::string pwd = ppwd;
      std::string file_fullpath = pwd + "/" + filename;
      file_fd = open(file_fullpath.c_str(), mode);
      if (file_fd < 0) {
        inno_log_error_errno("cannot open %s",
                             file_fullpath.c_str());
        return -1;
      } else {
        // pwd+file opened
        inno_log_info("open %s", file_fullpath.c_str());
        return file_fd;
      }
    } else {
      inno_log_error("cannot get pwd, cannot open file");
      return -3;
    }
  } else {
    inno_log_info("open %s", filename);
  }
  return file_fd;
}

#ifdef __MINGW64__
bool InnoUtils::is_socket_fd(int fd) {
  struct sockaddr_in addr;
  int ilen = sizeof(addr);
  int status = getsockname(fd, (struct sockaddr*)&addr, &ilen);
  if (status == 0) {
    return true;
  } else {
    return false;
  }
}
#endif

int InnoUtils::close_fd(int fd) {
#ifndef __MINGW64__
  return close(fd);
#else
  if (is_socket_fd(fd)) {
    inno_log_info("close socket, fd = %d", fd);
    return closesocket(fd);
  } else {
    inno_log_info("close file, fd = %d", fd);
    return close(fd);
  }
#endif
}

int InnoUtils::list_file(const std::string &path,
                         const std::string &pattern,
                         std::vector<std::string> *ret) {
  DIR *dp = ::opendir(path.c_str());
  if (dp == nullptr) {
    inno_log_error("open dir %s error", path.c_str());
    return -1;
  }
  struct dirent *entry;
  while ((entry = ::readdir(dp)) != nullptr) {
    if (strstr(entry->d_name, pattern.c_str())) {
      std::string entry_full_name = path +
        (InnoUtils::ends_with(path.c_str(), "/") ? "" : "/") + entry->d_name;
      ret->push_back(entry_full_name);
    }
  }
  ::closedir(dp);
  return 0;
}

std::string InnoUtils::get_current_time_str(const std::string& format) {
  time_t rawtime;
  struct tm *info;
  char temp[80];
  struct tm result_time;

  time(&rawtime);
#ifndef __MINGW64__
  info = localtime_r(&rawtime, &result_time);
#else
  info = localtime_s(&result_time, &rawtime) == 0 ?
         &result_time : NULL;
#endif
  strftime(temp, sizeof(temp), format.c_str(), info);
  return std::string(temp);
}

/**
 * 1. Create a SOCK_DGRAM socket(AF_INET, SOCK_DGRAM, 0)
 * 2. set reuse addr
 * 3. bind to input port
 * 4. set input opts
 * @param port
 * @param opts
 * @return -1 if failed. socket_fd if success.
 */
int InnoUdpHelper::bind(uint16_t port,
                        const std::vector<InnoUdpOpt> &opts) {
  int socket_fd = -1;
  struct sockaddr_in udp_listener_addr;
  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    inno_log_error_errno("udp listener socket creation error %d", socket_fd);
    return -1;
  }

#ifndef  __MINGW64__
  int reuse = 1;
#else
  char reuse = 1;
#endif
  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR,
                 &reuse, sizeof(reuse)) < 0) {
    inno_log_error("udp listener set reuse address error");
    close(socket_fd);
    return -1;
  }

  memset(&udp_listener_addr, 0, sizeof(udp_listener_addr));
  udp_listener_addr.sin_family = AF_INET;
  udp_listener_addr.sin_addr.s_addr = INADDR_ANY;
  udp_listener_addr.sin_port = htons(port);
  if (::bind(socket_fd, (const sockaddr*)&udp_listener_addr,
           sizeof(udp_listener_addr)) < 0) {
    inno_log_error_errno("failed to bind to port:%d", port);
    close(socket_fd);
    return -1;
  }

  for (auto & opt : opts) {
    int rs = setsockopt(socket_fd, opt.level, opt.optname,
                        reinterpret_cast<const char *>(opt.optval),
                        opt.optlen);
    if (rs < 0) {
      inno_log_info("opt: %d, %d, %p, %u, %s",
                    opt.level, opt.optname,
                    opt.optval, opt.optlen, opt.optname_str);
      inno_log_error_errno("setsockopt %s error %hu", opt.optname_str, port);
      close(socket_fd);
      return -1;
    }
  }

  return socket_fd;
}
}  // namespace innovusion
