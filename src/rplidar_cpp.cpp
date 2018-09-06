#include "rplidar_cpp.h"
#include "rplidar.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

using namespace rp::standalone::rplidar;

static constexpr double DEG2RAD(double x) {
  return (x)*3.14159265358979323846 / 180.;
}

std::ostream &operator<<(std::ostream &os, const RPLidarScan &scan) {
  std::streamsize ss = os.precision();

  os << "Seq: " << scan.seq << "\n";
  os << "Frame ID: " << scan.frame_id << "\n";
  os << "Timestamp [ns]: " << std::fixed << scan.timestamp << "\n";
  os << "Scan Time [ns]: " << std::fixed << scan.time_increment << "\n";
  
  double dtheta = (scan.angle_max-scan.angle_min)/(scan.ranges.size()-1);
  for (int i=0; i<scan.ranges.size(); ++i) {
    os << dtheta*i << "\t";
    os << "[" 
       << scan.ranges[i] 
       << ", "
       << std::setprecision(0)
       << scan.intensities[i]
       << "]\n"
       << std::setprecision(ss);
  }
  return os;
}

RPLidar::RPLidar(const std::string &serial_port) {
  drv = RPlidarDriver::CreateDriver(
      rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

  if (!drv) {
    throw std::runtime_error("Error: Failed to create RPLidar driver.");
  }

  if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
    RPlidarDriver::DisposeDriver(drv);
    throw std::runtime_error(
        "Error: Cannot bind RPLidar to the specified serial port " +
        serial_port + ".");
  }

  if (!getDeviceInfo()) {
    RPlidarDriver::DisposeDriver(drv);
    throw std::runtime_error("Error: Failed to get RPLidar device info.");
  }

  if (!checkHealth()) {
    RPlidarDriver::DisposeDriver(drv);
    throw std::runtime_error("Error: Failed to get RPLidar device health.");
  }

  drv->startMotor();

  u_result op_result;

  RplidarScanMode current_scan_mode;
  if (scan_mode.empty()) {
    op_result =
        drv->startScan(false /* not force scan */,
                       true /* use typical scan mode */, 0, &current_scan_mode);
  } else {
    std::vector<RplidarScanMode> allSupportedScanModes;
    op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

    if (IS_OK(op_result)) {
      _u16 selectedScanMode = _u16(-1);
      for (std::vector<RplidarScanMode>::iterator iter =
               allSupportedScanModes.begin();
           iter != allSupportedScanModes.end(); iter++) {
        if (iter->scan_mode == scan_mode) {
          selectedScanMode = iter->id;
          break;
        }
      }

      if (selectedScanMode == _u16(-1)) {
        //printf("scan mode `%s' is not supported by lidar, supported modes:",
        //       scan_mode.c_str());
        for (std::vector<RplidarScanMode>::iterator iter =
                 allSupportedScanModes.begin();
             iter != allSupportedScanModes.end(); iter++) {
          //printf("\t%s: max_distance: %.1f m, Point number: %.1fK",
          //       iter->scan_mode, iter->max_distance,
          //       (1000 / iter->us_per_sample));
        }
        op_result = RESULT_OPERATION_FAIL;
      } else {
        op_result =
            drv->startScanExpress(false /* not force scan */, selectedScanMode,
                                  0, &current_scan_mode);
      }
    }
  }

  if (IS_OK(op_result)) {
    max_distance = current_scan_mode.max_distance;
    printf("current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , ",
           current_scan_mode.scan_mode, current_scan_mode.max_distance,
           (1000 / current_scan_mode.us_per_sample));
  } else {
    throw std::runtime_error("Error: Failed to start RPLidar scan.");
  }
}

RPLidar::~RPLidar() {
  drv->stop();
  drv->stopMotor();
  RPlidarDriver::DisposeDriver(drv);
}

RPLidar &RPLidar::setBaudrate(int baudrate) {
  serial_baudrate = baudrate;
  return *this;
}

RPLidar &RPLidar::setFrameID(const std::string &id) {
  frame_id = id;
  return *this;
}

RPLidar &RPLidar::setMaxDistance(double max_distance) {
  this->max_distance = max_distance;
  return *this;
}

RPLidar &RPLidar::setScanMode(const std::string &scan_mode) {
  this->scan_mode = scan_mode;
  return *this;
}

float RPLidar::getAngle(const rplidar_response_measurement_node_hq_t &node) {
  return node.angle_z_q14 * 90.f / 16384.f;
}

bool RPLidar::getDeviceInfo() {
  u_result op_result;
  rplidar_response_device_info_t devinfo;

  op_result = drv->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      std::cout << "Error, operation timed out. RESULT_OPERATION_TIMEOUT! "
                << std::endl;
    } else {
      std::cout << "Error, unexpected error, code: " << std::hex
                << static_cast<int>(op_result) << std::endl;
    }
    return false;
  }

  // print out the device serial number, firmware and hardware version number..
  //printf("RPLIDAR S/N: ");
  //for (int pos = 0; pos < 16; ++pos) {
  //  printf("%02X", devinfo.serialnum[pos]);
  //}
  //printf("\n");
  //printf("Firmware Ver: %d.%02d\n", devinfo.firmware_version >> 8,
  //       devinfo.firmware_version & 0xFF);
  //printf("Hardware Rev: %d\n", (int)devinfo.hardware_version);

  return true;
}

bool RPLidar::checkHealth() {
  u_result op_result;
  rplidar_response_device_health_t healthinfo;

  op_result = drv->getHealth(healthinfo);
  if (IS_OK(op_result)) {
    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
      throw std::runtime_error("Error, rplidar internal error detected. Please reboot the device to retry.");
    }
    return true;
  } else {
    throw std::runtime_error("Error, Failed to retrieve RPLidar health status.");
  }
  return false;
}

bool RPLidar::stopMotor() {
  if (!drv) {
    return false;
  }
  drv->stop();
  drv->stopMotor();
  return true;
}

bool RPLidar::startMotor() {
  if (!drv) {
    return false;
  }
  drv->startMotor();
  drv->startScan(0, 1);
  return true;
}

std::optional<RPLidarScan> RPLidar::poll() {
  using namespace std::chrono;

  rplidar_response_measurement_node_hq_t nodes[360 * 8];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);

  auto start_scan_time = steady_clock::now();
  if (drv->grabScanDataHq(nodes, count) != RESULT_OK) {
    return std::nullopt;
  }
  auto end_scan_time = steady_clock::now();

  if (drv->ascendScanData(nodes, count) != RESULT_OK) {
    return std::nullopt;
  }

  int i = 0;
  int start_node = 0, end_node = 0;
  // find the first valid node and last valid node
  while (nodes[i++].dist_mm_q2 == 0)
    ;
  start_node = i - 1;
  i = count - 1;
  while (nodes[i--].dist_mm_q2 == 0)
    ;
  end_node = i + 1;

  double angle_min = DEG2RAD(getAngle(nodes[start_node]));
  double angle_max = DEG2RAD(getAngle(nodes[end_node]));

  RPLidarScan scan;
  scan.seq = scan_seq++;
  scan.frame_id = frame_id;

  scan.timestamp =
      duration_cast<nanoseconds>(start_scan_time.time_since_epoch()).count();
  scan.scan_time =
      duration_cast<nanoseconds>(end_scan_time - start_scan_time).count();
  scan.time_increment = scan.scan_time / (double)(count - 1);

  scan.angle_min = angle_min;
  scan.angle_max = angle_max;
  scan.angle_increment =
      (scan.angle_max - scan.angle_min) / (double)(count - 1);

  scan.range_min = 0.15;
  scan.range_max = max_distance;

  for (int i = 0; i < count; ++i) {
    float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
    if (read_value == 0.0) {
      read_value = std::numeric_limits<float>::infinity();
    }
    scan.ranges.push_back(read_value);
    scan.intensities.push_back(nodes[i].quality >> 2);
  }

  return std::optional<RPLidarScan>{scan};
}
