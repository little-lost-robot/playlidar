/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <time.h>
#include <cmath>
#include <chrono>
#include <sstream>

#include "tinyosc.h"
#include "laserscan.h"
#include "rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
  while (ms>=1000){
    usleep(1000*1000);
    ms-=1000;
  };
  if (ms!=0)
    usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;
using namespace std::chrono;

#define PORT 8888
//RPlidarDriver * drv = NULL;

void publish_scan(rplidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, milliseconds start,
                  milliseconds scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  int frame_id){

    LaserScan scan_msg;
    //scan_msg.header.stamp = start;
    //scan_msg.header.frame_id = frame_id;
    //scan_count++;

    bool reversed = (angle_max > angle_min);
    if ( reversed ) {
      scan_msg.angle_min =  M_PI - angle_max;
      scan_msg.angle_max =  M_PI - angle_min;
    } else {
      scan_msg.angle_min =  M_PI - angle_min;
      scan_msg.angle_max =  M_PI - angle_max;
    }
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = max_distance;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
            scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count-1-i] = read_value;
            scan_msg.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
        }
    }


    //publish scan_msg;

}


bool stop_motor(RPlidarDriver * drv){
  if(!drv)
    return false;
  drv->stop();
  drv->stopMotor();
  return true;
}

bool start_motor(RPlidarDriver * drv){
  if(!drv)
    return false;
  drv->startMotor();
  drv->startScan(0,1);
  return true;
}

bool getRPLIDARDeviceInfo(RPlidarDriver * drv)
{
  u_result     op_result;
  rplidar_response_device_info_t devinfo;

  op_result = drv->getDeviceInfo(devinfo);
  if (IS_FAIL(op_result)) {
    if (op_result == RESULT_OPERATION_TIMEOUT) {
      printf("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
    } else {
      printf("Error, unexpected error, code: %x",op_result);
    }
    return false;
  }

  // print out the device serial number, firmware and hardware version number..
  printf("RPLIDAR S/N: ");
  for (int pos = 0; pos < 16 ;++pos) {
    printf("%02X", devinfo.serialnum[pos]);
  }
  printf("\n");
  printf("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
  printf("Hardware Rev: %d",(int)devinfo.hardware_version);
  return true;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
  u_result     op_result;
  rplidar_response_device_health_t healthinfo;
  op_result = drv->getHealth(healthinfo);
  bool is_ok = false;

  if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
    printf("RPLidar health status : ");
    switch (healthinfo.status) {
      case RPLIDAR_STATUS_OK:
        printf("OK.");
        is_ok=true;
      case RPLIDAR_STATUS_WARNING:
        printf("Warning.");
        is_ok=true;
      case RPLIDAR_STATUS_ERROR:
        is_ok=false;
        printf("Error.");
    }
    printf(" (errorcode: %d)\n", healthinfo.error_code);
  } else {
    fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
    is_ok=false;
  }

  if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
    fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
    is_ok=false;
    // enable the following code if you want rplidar to be reboot by software
    // drv->reset();
  }
  return is_ok;
}

bool ctrl_c_pressed;
void ctrlc(int){
  ctrl_c_pressed = true;
}

static float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
  return node.angle_z_q14 * 90.f / 16384.f;
}

milliseconds timeNow(){
  milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  return ms;
}

int main(int argc, const char * argv[]) {
  std::string serial_port;
  int serial_baudrate = 115200;
  bool inverted = false;
  bool angle_compensate = true;
  float max_distance = 8.0;
  int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
  std::string scan_mode;

  char buffer[1024] = {0};
  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);
  int socket_fd=0, valread;

  if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
    printf("\n Socket creation error \n");
    return -1;
  }else{
    printf("\n Socket created \n");
  }
  if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0){
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }
  if (connect(socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
    printf("\nConnection Failed boo\n");
    return -1;
  }
  else{
    printf("\nConnected: %d \n", PORT);
  }

  u_result     op_result;

#ifdef _WIN32
  serial_port = "\\\\.\\com57";
#elif __APPLE__
  serial_port = "/dev/tty.SLAB_USBtoUART";
#else
  serial_port = "/dev/ttyUSB0";
#endif

  RPlidarDriver *drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);

  if(!drv) {
    printf("Create Driver fail, exit");
    exit(-2);
  }

  if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
    printf("Error, cannot bind to the specified serial port %s.",serial_port.c_str());
    RPlidarDriver::DisposeDriver(drv);
    return -1;
  }

  if(!getRPLIDARDeviceInfo(drv)) {
    return -1;
  }

  if(!checkRPLIDARHealth(drv)) {
    return -1;
  }

  signal(SIGINT, ctrlc);

  start_motor(drv);

  RplidarScanMode current_scan_mode;
  //TODO: do we need other scan modes?
  op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);

  if(IS_OK(op_result)){
    //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
    angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
    if(angle_compensate_multiple < 1){
      angle_compensate_multiple = 1;
    }
    max_distance = current_scan_mode.max_distance;
    printf("current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d",  current_scan_mode.scan_mode,
           current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);

  }else{
    printf("Can not start scan: %08x!", op_result);
    return -1;
  }

  milliseconds start_scan_time;
  milliseconds end_scan_time;
  milliseconds scan_duration;

  // fetch result and print it out...
  int frame_id=0;
  while (1) {
    rplidar_response_measurement_node_hq_t nodes[360*8];
    size_t   count = _countof(nodes);

    start_scan_time = timeNow();
    // Rank the scan data from grabScanData() as the angle inscreases  ascendScanData()
    op_result = drv->grabScanDataHq(nodes, count);
    end_scan_time = timeNow();
    scan_duration = (end_scan_time - start_scan_time);

    if (IS_OK(op_result)) {
      drv->ascendScanData(nodes, count);
      float angle_min = DEG2RAD(0.0f);
      float angle_max = DEG2RAD(359.0f);
      if (angle_compensate) {
        //const int angle_compensate_multiple = 1;
        const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
        int angle_compensate_offset = 0;
        rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
        memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

        for(int i=0 ; i < count; i++ ) {
          if (nodes[i].dist_mm_q2 != 0) {
            float angle = getAngle(nodes[i]);
            int angle_value = (int)(angle * angle_compensate_multiple);
            if ((angle_value - angle_compensate_offset) < 0){
              angle_compensate_offset = angle_value;
            }
            for (int j = 0; j < angle_compensate_multiple; j++) {
              angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
            }
          }
        }
        publish_scan(angle_compensate_nodes, angle_compensate_nodes_count,
                     start_scan_time, scan_duration, inverted,
                     angle_min, angle_max, max_distance,
                     frame_id);


      }else{
        int start_node = 0, end_node = 0;
        int i = 0;
        // find the first valid node and last valid node
        while (nodes[i++].dist_mm_q2 == 0);
        start_node = i-1;
        i = count -1;
        while (nodes[i--].dist_mm_q2 == 0);
        end_node = i+1;

        angle_min = DEG2RAD(getAngle(nodes[start_node]));
        angle_max = DEG2RAD(getAngle(nodes[end_node]));

        publish_scan(&nodes[start_node], end_node-start_node+1,
                     start_scan_time, scan_duration, inverted,
                     angle_min, angle_max, max_distance,
                     frame_id);
      }


      // for (int pos = 0; pos < (int)count ; ++pos) {
      //   // declare a buffer for writing the OSC packet into
      //   char oscbuffer[1024];
      //   int len = tosc_writeMessage(
      //                               buffer, sizeof(buffer),
      //                               "/ping", // the address
      //                               "fsi",   // the format; 'f':32-bit float, 's':ascii string, 'i':32-bit integer
      //                               1.0f, "hello", 2);
      //   send(socket_fd, buffer, len, 0);
      //
      //   printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
      //          (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
      //          (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
      //          nodes[pos].dist_mm_q2/4.0f,
      //          nodes[pos].quality);
      // }
      //}
    }
    else if (op_result == RESULT_OPERATION_FAIL) {
      // All the data is invalid, just publish them
      float angle_min = DEG2RAD(0.0f);
      float angle_max = DEG2RAD(359.0f);
    }

    if (ctrl_c_pressed){
      break;
    }
    frame_id = (frame_id+1) % 1000000;
  }

  stop_motor(drv);
  // done!
on_finished:
  RPlidarDriver::DisposeDriver(drv);
  drv = NULL;
  return 0;
}
