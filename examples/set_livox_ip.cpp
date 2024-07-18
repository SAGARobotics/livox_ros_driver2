//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <cstddef>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>

std::string current_ip, desired_ip;

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (response == nullptr) {
    return;
  }
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) {
    return;
  }
  if (response->ret_code == 0 && response->error_key == 0) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}


void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  }

  if(info->lidar_ip == current_ip)
  {
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    printf("Livox Lidar found: %s\n", info->lidar_ip);

    LivoxLidarIpInfo lidar_ip_info;
    strcpy(lidar_ip_info.ip_addr, desired_ip.c_str());
    strcpy(lidar_ip_info.net_mask, "255.255.255.0");
    strcpy(lidar_ip_info.gw_addr, "192.168.0.1");
    SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
  }
}

int main(int argc, const char *argv[]) {
  if (argc != 3) {
    printf("Params Invalid, must input config path.\n");
    return -1;
  }
  std::string config = "mid360_config.json";
  current_ip = argv[1];
  desired_ip = argv[2];

  // REQUIRED, to init Livox SDK2
  if (!LivoxLidarSdkInit(config.c_str())) {
    printf("Livox Init Failed\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  sleep(5);

  LivoxLidarSdkUninit();
  printf("Livox IP reconfigured!\n");
  return 0;
}
