/**
 * \file	3dmgx2.cc
 * \copyright Copyright (C) 2008-20010  Willow Garage. All rights reserved.
 * \section LICENSE
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>
#include "poll.h"
#include <lib_atlas/macros.h>
#include <lib_atlas/exceptions.h>
#include <lib_atlas/io/formatter.h>
#include "imu_driver.h"

#define CMD_ACCEL_ANGRATE_MAG_ORIENT_REP_LEN 79
#define CMD_RAW_ACCEL_ANGRATE_LEN 31

namespace provider_imu {

namespace details {

//! Code to swap bytes since IMU is big endian
static inline unsigned short bswap_16(unsigned short x) {
  return (x >> 8) | (x << 8);
}

//! Code to swap bytes since IMU is big endian
static inline unsigned int bswap_32(unsigned int x) {
  return (bswap_16(x & 0xffff) << 16) | (bswap_16(x >> 16));
}

//! Code to extract a floating point number from the IMU
static float extract_float(uint8_t *addr) {
  float tmp;

  *((unsigned char *) (&tmp) + 3) = *(addr);
  *((unsigned char *) (&tmp) + 2) = *(addr + 1);
  *((unsigned char *) (&tmp) + 1) = *(addr + 2);
  *((unsigned char *) (&tmp)) = *(addr + 3);

  return tmp;
}

//! Helper function to get system time in nanoseconds.
static unsigned long long time_helper() {
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (unsigned long long)(curtime.tv_sec) * 1000000000 +
         (unsigned long long)(curtime.tv_nsec);
#else
  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  return (unsigned long long) (timeofday.tv_sec) * 1000000000 +
      (unsigned long long) (timeofday.tv_usec) * 1000;
#endif
}

} // namespace details

// Some systems (e.g., OS X) require explicit externing of static class
// members.
extern constexpr double ImuDriver::G;
extern constexpr double ImuDriver::KF_K_1;
extern constexpr double ImuDriver::KF_K_2;

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
ImuDriver::ImuDriver() : file_descriptor_(-1), continuous_(false), is_gx3_(false) {}

//------------------------------------------------------------------------------
//
ImuDriver::~ImuDriver() { ClosePort(); }

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void ImuDriver::OpenPort(const char *port_name) {
  ClosePort();  // In case it was previously open, try to close it first.

  // Open the port
  file_descriptor_ = open(port_name, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY,
                          S_IRUSR | S_IWUSR);
  if (file_descriptor_ < 0) {
    const char *extra_msg = "";
    switch (errno) {
      case EACCES:
        extra_msg =
            "You probably don't have premission to open the port for reading "
            "and writing.";
        break;
      case ENOENT:
        extra_msg =
            "The requested port does not exist. Is the IMU connected? Was the "
            "port name misspelled?";
        break;
    }

    ATLAS_THROW(std::runtime_error, atlas::Format("Unable to open serial port [{0}]. {1}. {2}", port_name,
               strerror(errno), extra_msg));
  }

  // Lock the port
  struct flock fl;
  fl.l_type = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 0;
  fl.l_pid = getpid();

  if (fcntl(file_descriptor_, F_SETLK, &fl) != 0)
    ATLAS_THROW(std::runtime_error,
                atlas::Format("Device {0} is already locked. Try 'lsof | grep {1}' to find "
               "other processes that currently have the port open.",
               port_name, port_name));

  // Change port settings
  struct termios term;
  if (tcgetattr(file_descriptor_, &term) < 0)
    ATLAS_THROW(std::runtime_error,
                atlas::Format("Unable to get serial port attributes. The port you specified "
               "({0}) may not be a serial port.",
               port_name));

  cfmakeraw(&term);
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  if (tcsetattr(file_descriptor_, TCSAFLUSH, &term) < 0)
    ATLAS_THROW(std::runtime_error,
                atlas::Format("Unable to set serial port attributes. The port you specified "
               "({0}) may not be a serial port.",
               port_name));  /// @todo tcsetattr returns true if at least one
  /// attribute was set. Hence, we might not have set
  /// everything on success.

  // Stop continuous mode
  StopContinuous();

  // Make sure queues are empty before we begin
  if (tcflush(file_descriptor_, TCIOFLUSH) != 0)
    ATLAS_THROW(std::runtime_error,
               "Tcflush failed. Please report this error if you see it.");
}

//------------------------------------------------------------------------------
//
void ImuDriver::ClosePort() {
  if (file_descriptor_ != -1) {
    if (continuous_) {
      try {
        // ROS_DEBUG("stopping continuous");
        StopContinuous();

      } catch (std::runtime_error &e) {
        // std::runtime_errors here are fine since we are closing anyways
      }
    }

    if (close(file_descriptor_) != 0)
      ATLAS_THROW(std::runtime_error, atlas::Format("Unable to close serial port; [{0}]",
                 strerror(errno)));
    file_descriptor_ = -1;
  }
}

//------------------------------------------------------------------------------
//
void ImuDriver::InitTime(double fix_off) {
  wraps_ = 0;

  uint8_t cmd[1];
  uint8_t rep[31];
  cmd[0] = CMD_RAW;

  Transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);
  start_time_ = details::time_helper();

  int k = 25;
  offset_ticks_ = details::bswap_32(*(uint32_t *)(rep + k));
  last_ticks_ = offset_ticks_;

  // reset kalman filter state
  offset_ = 0;
  d_offset_ = 0;
  sum_meas_ = 0;
  counter_ = 0;

  // fixed offset
  fixed_offset_ = fix_off;
}

//------------------------------------------------------------------------------
//
void ImuDriver::InitGyros(double *bias_x, double *bias_y, double *bias_z) {
  wraps_ = 0;

  uint8_t cmd[5];
  uint8_t rep[19];

  cmd[0] = CMD_CAPTURE_GYRO_BIAS;
  cmd[1] = CMD_ACCEL_ANGRATE_MAG_ORIENT;
  cmd[2] = 0x29;
  *(unsigned short *)(&cmd[3]) = details::bswap_16(10000);

  Transact(cmd, sizeof(cmd), rep, sizeof(rep), 30000);

  if (bias_x) *bias_x = details::extract_float(rep + 1);

  if (bias_y) *bias_y = details::extract_float(rep + 5);

  if (bias_z) *bias_z = details::extract_float(rep + 9);
}

//------------------------------------------------------------------------------
//
bool ImuDriver::SetContinuous(cmd command) {
  uint8_t cmd[4];
  uint8_t rep[8];

  cmd[0] = CMD_CONTINUOUS;
  cmd[1] = 0xC1;  // Confirms user intent
  cmd[2] = 0x29;  // Confirms user intent
  cmd[3] = command;

  Transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

  // Verify that continuous mode is set on correct command:
  if (rep[1] != command) {
    return false;
  }

  continuous_ = true;
  return true;
}

//------------------------------------------------------------------------------
//
void ImuDriver::StopContinuous() {
  uint8_t cmd[3];

  cmd[0] = CMD_STOP_CONTINUOUS;

  cmd[1] = 0x75;  // gx3 - confirms user intent

  cmd[2] = 0xb4;  // gx3 - confirms user intent

  Send(cmd, sizeof(cmd));

  Send(cmd, is_gx3_ ? 3 : 1);

  usleep(1000000);

  if (tcflush(file_descriptor_, TCIOFLUSH) != 0) {
    ATLAS_THROW(std::runtime_error, "Tcflush failed");
  }

  continuous_ = false;
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveAccelAngrateMag(uint64_t *time, double *accel,
                                       double *angrate, double *mag) {
  int i, k;
  uint8_t rep[43];

  uint64_t sys_time;
  uint64_t imu_time;

  // ROS_DEBUG("About to do receive.");
  Receive(CMD_ACCEL_ANGRATE_MAG, rep, sizeof(rep), 1000, &sys_time);
  // ROS_DEBUG("Receive finished.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++) {
    accel[i] = details::extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++) {
    angrate[i] = details::extract_float(rep + k);
    k += 4;
  }

  // Read the magnetometer reading.
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = details::extract_float(rep + k);
    k += 4;
  }

  imu_time = ExtractTime(rep + 37);
  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveAccelAngrateOrientation(uint64_t *time, double *accel,
                                               double *angrate,
                                               double *orientation) {
  int i, k;
  uint8_t rep[67];

  uint64_t sys_time;
  uint64_t imu_time;

  // ROS_DEBUG("About to do receive.");
  Receive(CMD_ACCEL_ANGRATE_ORIENT, rep, sizeof(rep), 1000, &sys_time);
  // ROS_DEBUG("Finished receive.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++) {
    accel[i] = details::extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++) {
    angrate[i] = details::extract_float(rep + k);
    k += 4;
  }

  // Read the orientation matrix
  k = 25;
  for (i = 0; i < 9; i++) {
    orientation[i] = details::extract_float(rep + k);
    k += 4;
  }

  imu_time = ExtractTime(rep + 61);
  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveAccelAngrate(uint64_t *time, double *accel,
                                    double *angrate) {
  int i, k;
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;

  Receive(CMD_ACCEL_ANGRATE, rep, sizeof(rep), 1000, &sys_time);

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++) {
    accel[i] = details::extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++) {
    angrate[i] = details::extract_float(rep + k);
    k += 4;
  }

  imu_time = ExtractTime(rep + 25);
  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveDelvelDelang(uint64_t *time, double *delvel,
                                    double *delang) {
  int i, k;
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;

  Receive(CMD_DELVEL_DELANG, rep, sizeof(rep), 1000, &sys_time);

  // Read the delta angles:
  k = 1;
  for (i = 0; i < 3; i++) {
    delang[i] = details::extract_float(rep + k);
    k += 4;
  }

  // Read the delta velocities
  k = 13;
  for (i = 0; i < 3; i++) {
    delvel[i] = details::extract_float(rep + k) * G;
    k += 4;
  }

  imu_time = ExtractTime(rep + 25);
  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveEuler(uint64_t *time, double *roll, double *pitch,
                             double *yaw) {
  uint8_t rep[19];

  uint64_t sys_time;
  uint64_t imu_time;

  Receive(CMD_EULER, rep, sizeof(rep), 1000, &sys_time);

  *roll = details::extract_float(rep + 1);
  *pitch = details::extract_float(rep + 5);
  *yaw = details::extract_float(rep + 9);

  imu_time = ExtractTime(rep + 13);
  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
bool ImuDriver::GetDeviceIdentifierString(id_string type, char *id) {
  uint8_t cmd[2];
  uint8_t rep[20];

  cmd[0] = CMD_DEV_ID_STR;
  cmd[1] = type;

  Transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);

  if (cmd[0] != CMD_DEV_ID_STR || cmd[1] != type) return false;

  id[16] = 0;
  memcpy(id, rep + 2, 16);

  if (type == ID_DEVICE_NAME) {
    is_gx3_ = (strstr(id, "GX3") != NULL);
  }

  return true;
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveAccelAngrateMagOrientation(uint64_t *time,
                                                  double *accel,
                                                  double *angrate,
                                                  double *mag,
                                                  double *orientation) {
  uint8_t rep[CMD_ACCEL_ANGRATE_MAG_ORIENT_REP_LEN];

  int k, i;
  uint64_t sys_time;
  uint64_t imu_time;

  Receive(CMD_ACCEL_ANGRATE_MAG_ORIENT, rep, sizeof(rep), 1000, &sys_time);

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++) {
    accel[i] = details::extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++) {
    angrate[i] = details::extract_float(rep + k);
    k += 4;
  }

  // Read the magnetic field matrix
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = details::extract_float(rep + k);
    k += 4;
  }

  // Read the orientation matrix
  k = 37;
  for (i = 0; i < 9; i++) {
    orientation[i] = details::extract_float(rep + k);
    k += 4;
  }
  imu_time = ExtractTime(rep + 73);

  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
void ImuDriver::ReceiveRawAccelAngrate(uint64_t *time, double *accel,
                                       double *angrate) {
  int i, k;
  uint8_t rep[CMD_RAW_ACCEL_ANGRATE_LEN];

  uint64_t sys_time;
  uint64_t imu_time;

  Receive(ImuDriver::CMD_RAW, rep, sizeof(rep), 1000, &sys_time);

  // Read the accelerator AD register values 0 - 65535 given as float
  k = 1;
  for (i = 0; i < 3; i++) {
    accel[i] = details::extract_float(rep + k);
    k += 4;
  }

  // Read the angular rates AD registor values 0 - 65535 (given as float
  k = 13;
  for (i = 0; i < 3; i++) {
    angrate[i] = details::extract_float(rep + k);
    k += 4;
  }

  imu_time = ExtractTime(rep + 25);
  *time = FilterTime(imu_time, sys_time);
}

//------------------------------------------------------------------------------
//
uint64_t ImuDriver::ExtractTime(uint8_t *addr) {
  uint32_t ticks = details::bswap_32(*(uint32_t *)(addr));

  if (ticks < last_ticks_) {
    wraps_ += 1;
  }

  last_ticks_ = ticks;

  uint64_t all_ticks = ((uint64_t) wraps_ << 32) - offset_ticks_ + ticks;

  return start_time_ +
         (is_gx3_
              ? (uint64_t)(all_ticks * (1000000000.0 / TICKS_PER_SEC_GX3))
              : (uint64_t)(all_ticks * (1000000000.0 /
                                        TICKS_PER_SEC_GX2)));  // syntax a bit
}

//------------------------------------------------------------------------------
//
int ImuDriver::Transact(void *cmd, int cmd_len, void *rep, int rep_len,
                        int timeout) {
  Send(cmd, cmd_len);

  return Receive(*(uint8_t *) cmd, rep, rep_len, timeout);
}

//------------------------------------------------------------------------------
//
int ImuDriver::Send(void *cmd, int cmd_len) {
  int bytes;

  // Write the data to the port
  bytes = write(file_descriptor_, cmd, cmd_len);
  if (bytes < 0) {
    ATLAS_THROW(std::runtime_error, atlas::Format("error writing to IMU [{0}]", strerror(errno)));
  }

  if (bytes != cmd_len) {
    ATLAS_THROW(std::runtime_error, "whole message not written to IMU");
  }

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  if (tcdrain(file_descriptor_) != 0) {
    ATLAS_THROW(std::runtime_error, "tcdrain failed");
  }

  return bytes;
}

//------------------------------------------------------------------------------
//
static int read_with_timeout(int fd, void *buff, size_t count, int timeout) {
  ssize_t nbytes;
  int retval;

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;

  if (timeout == 0)
    timeout = -1;  // For compatibility with former behavior, 0 means no
  // timeout. For poll, negative means no timeout.

  if ((retval = poll(ufd, 1, timeout)) < 0) {
    ATLAS_THROW(std::runtime_error, atlas::Format("poll failed  [{0}]", strerror(errno)));
  }

  if (retval == 0) {
    ATLAS_THROW(std::runtime_error, "timeout reached");
  }

  nbytes = read(fd, (uint8_t *)buff, count);

  if (nbytes < 0) {
    ATLAS_THROW(std::runtime_error, atlas::Format("read failed  [{0}]", strerror(errno)));
  }

  return nbytes;
}

//------------------------------------------------------------------------------
//
int ImuDriver::Receive(uint8_t command, void *rep, int rep_len, int timeout,
                       uint64_t *sys_time) {
  int nbytes, bytes, skippedbytes;

  skippedbytes = 0;

  struct pollfd ufd[1];
  ufd[0].fd = file_descriptor_;
  ufd[0].events = POLLIN;

  // Skip everything until we find our "header"
  *(uint8_t *)(rep) = 0;

  while (*(uint8_t *)(rep) != command && skippedbytes < MAX_BYTES_SKIPPED) {
    read_with_timeout(file_descriptor_, rep, 1, timeout);

    skippedbytes++;
  }

  if (sys_time != NULL) *sys_time = details::time_helper();

  // We now have 1 byte
  bytes = 1;

  // Read the rest of the message:
  while (bytes < rep_len) {
    nbytes =
        read_with_timeout(file_descriptor_, (uint8_t *)rep + bytes, rep_len - bytes, timeout);

    if (nbytes < 0) {
      ATLAS_THROW(std::runtime_error, atlas::Format("read failed  [{0}]", strerror(errno)));
    }

    bytes += nbytes;
  }

  // Checksum is always final 2 bytes of transaction

  uint16_t checksum = 0;
  for (int i = 0; i < rep_len - 2; i++) {
    checksum += ((uint8_t *)rep)[i];
  }

  // If wrong throw std::runtime_error
  uint16_t correct_cks = details::bswap_16(*(uint16_t *)((uint8_t *)rep + rep_len - 2));
  if (checksum != correct_cks)
    ATLAS_THROW(atlas::CorruptedDataException,
               "invalid checksum.\n Make sure the IMU sensor is connected to "
               "this computer.");

  return bytes;
}

//------------------------------------------------------------------------------
//
uint64_t ImuDriver::FilterTime(uint64_t imu_time, uint64_t sys_time) {
  // first calculate the sum of KF_NUM_SUM measurements
  if (counter_ < KF_NUM_SUM) {
    counter_++;
    sum_meas_ += (ToDouble(imu_time) - ToDouble(sys_time));
  }
  // update kalman filter with fixed innovation
  else {
    // system update
    offset_ += d_offset_;

    // measurement update
    double meas_diff = (sum_meas_ / KF_NUM_SUM) - offset_;
    offset_ += KF_K_1 * meas_diff;
    d_offset_ += KF_K_2 * meas_diff;

    // reset counter and average
    counter_ = 0;
    sum_meas_ = 0;
  }
  return imu_time - ToUint64(offset_) + ToUint64(fixed_offset_);
}

//------------------------------------------------------------------------------
//
double ImuDriver::ToDouble(uint64_t time) {
  double res = trunc(time / 1e9);
  res += (((double)time) / 1e9) - res;
  return res;
}

//------------------------------------------------------------------------------
//
uint64_t ImuDriver::ToUint64(double time) { return (uint64_t)(time * 1e9); }

//------------------------------------------------------------------------------
//
void ImuDriver::Reset() {
  uint8_t cmd[2];

  cmd[0] = CMD_RESET_IMU;
  cmd[1] = 0x9E;  // Confirms user intent
  cmd[2] = 0x3A;  // Confirms user intent

  Send(cmd, sizeof(cmd));
}

//------------------------------------------------------------------------------
//
std::string ImuDriver::GetFirmware() {
  uint8_t cmd[1];
  uint8_t rep[7];
  uint32_t version;
  std::string firm_version;

  cmd[0] = CMD_FIRMWARE_VERSION;
  Transact(cmd, sizeof(cmd), rep, sizeof(rep), 100);

  uint16_t checksum = 0;
  for (int i = 0; i < sizeof(rep) - 2; i++) {
    checksum += ((uint8_t *)rep)[i];
  }

  if (rep[0] != CMD_FIRMWARE_VERSION) {
    ATLAS_THROW(std::runtime_error, "Wrong command receive from the IMU");
  } else if (checksum !=
             details::bswap_16(*(uint16_t *)((uint8_t *)rep + sizeof(rep) - 2))) {
    ATLAS_THROW(atlas::CorruptedDataException,
               "invalid checksum.\n Something went wrong.");
  } else {
    version = rep[1];
    version = version << 8;
    version = version | rep[2];
    version = version << 8;
    version = version | rep[3];
    version = version << 8;
    version = version | rep[4];

    firm_version = std::to_string(version);

    std::stringstream ss;
    ss << firm_version.substr(0, 1);
    ss << ".";
    ss << firm_version.substr(1, 1);
    ss << ".";
    ss << firm_version.substr(2, 2);

    firm_version = ss.str();
  }

  return firm_version;
}

}  // namespace provider_imu
