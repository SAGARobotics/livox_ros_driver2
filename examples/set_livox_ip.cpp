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
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <arpa/inet.h>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <stdio.h>
#include <string.h>
#include <string>
#include <chrono>

std::string current_ip, desired_ip;
std::atomic<bool> request_started{false};
std::mutex result_mutex;
std::condition_variable result_ready;
bool finished = false;
bool succeeded = false;

void Finish(bool success) {
  {
    std::lock_guard<std::mutex> lock(result_mutex);
    finished = true;
    succeeded = success;
  }
  result_ready.notify_one();
}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
  if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
    fprintf(stderr, "Lidar reboot failed: status=%d, ret_code=%d. IP may already be stored; verify the device before retrying.\n",
            status, response ? response->ret_code : -1);
    Finish(false);
    return;
  }
  printf("Lidar acknowledged reboot to apply %s -> %s.\n", current_ip.c_str(), desired_ip.c_str());
  Finish(true);
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (status != kLivoxLidarStatusSuccess || response == nullptr ||
      response->ret_code != 0 || response->error_key != 0) {
    fprintf(stderr, "Set lidar IP failed: status=%d, ret_code=%d, error_key=%d.\n",
            status, response ? response->ret_code : -1, response ? response->error_key : -1);
    Finish(false);
    return;
  }
  livox_status reboot_status = LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  if (reboot_status != kLivoxLidarStatusSuccess) {
    fprintf(stderr, "Could not request lidar reboot: status=%d. IP may already be stored.\n", reboot_status);
    Finish(false);
  }
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) {
    printf("lidar info change callback failed, the info is nullptr.\n");
    return;
  }

  if (current_ip == info->lidar_ip && !request_started.exchange(true))
  {
    printf("Livox lidar found: %s, SN: %s, device type: %u\n",
           info->lidar_ip, info->sn, info->dev_type);

    LivoxLidarIpInfo lidar_ip_info{};
    strcpy(lidar_ip_info.ip_addr, desired_ip.c_str());
    strcpy(lidar_ip_info.net_mask, "255.255.255.0");
    strcpy(lidar_ip_info.gw_addr, "192.168.0.1");
    livox_status status = SetLivoxLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
    if (status != kLivoxLidarStatusSuccess) {
      fprintf(stderr, "Could not request lidar IP change: status=%d.\n", status);
      Finish(false);
    }
  }
}

bool IsValidUnicastIp(const char* text) {
  in_addr address{};
  if (strlen(text) >= sizeof(LivoxLidarIpInfo{}.ip_addr) ||
      inet_pton(AF_INET, text, &address) != 1) {
    return false;
  }
  uint32_t ip = ntohl(address.s_addr);
  return (ip >> 24) != 0 && (ip >> 24) != 127 && (ip >> 24) < 224 &&
         (ip & 0xff) != 0 && (ip & 0xff) != 255;
}

int main(int argc, const char *argv[]) {
  if (argc != 3 && argc != 4) {
    fprintf(stderr, "Usage: %s <current_ip> <desired_ip> [config.json]\n"
                    "Default config: examples/mid360_config.json; applies mask 255.255.255.0 and gateway 192.168.0.1.\n",
            argv[0]);
    return 1;
  }
  if (!IsValidUnicastIp(argv[1]) || !IsValidUnicastIp(argv[2]) ||
      strcmp(argv[1], argv[2]) == 0) {
    fprintf(stderr, "Expected distinct usable unicast IPv4 addresses.\n");
    return 1;
  }

  current_ip = argv[1];
  desired_ip = argv[2];
  std::string config = argc == 4 ? argv[3] :
      ament_index_cpp::get_package_share_directory("livox_ros_driver2") + "/examples/mid360_config.json";
  std::ifstream config_file(config);
  if (!config_file) {
    fprintf(stderr, "Cannot read SDK config: %s\n", config.c_str());
    return 1;
  }
  config_file.close();

  // Config-free discovery crashes in this SDK; the default config covers both models.
  if (!LivoxLidarSdkInit(config.c_str())) {
    fprintf(stderr, "Livox SDK init failed; check host IP and that no driver owns port 56000.\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  bool success = false;
  {
    std::unique_lock<std::mutex> lock(result_mutex);
    if (!result_ready.wait_for(lock, std::chrono::seconds(30), [] { return finished; })) {
      fprintf(stderr, "Timed out changing %s -> %s (%s). Verify both addresses before retrying.\n",
              current_ip.c_str(), desired_ip.c_str(),
              request_started.load() ? "request sent" : "target not discovered");
    } else {
      success = succeeded;
    }
  }

  LivoxLidarSdkUninit();
  return success ? 0 : 1;
}
