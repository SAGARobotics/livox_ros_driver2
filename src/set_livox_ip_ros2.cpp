#include <ament_index_cpp/get_package_share_directory.hpp>
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace
{
std::string current_ip;
std::string desired_ip;
std::string subnet_mask;
std::string gateway_ip;
std::atomic<bool> request_started{false};
std::atomic<bool> operation_finished{false};
std::atomic<int> operation_result{1};

void RebootCallback(livox_status, uint32_t handle, LivoxLidarRebootResponse *response, void *)
{
  if (response == nullptr)
  {
    std::fprintf(stderr, "Lidar reboot response is null (handle %u).\n", handle);
    operation_finished = true;
    return;
  }

  std::printf("Lidar reboot response: ret_code=%u\n", response->ret_code);
  operation_result = response->ret_code == 0 ? 0 : 1;
  operation_finished = true;
}

void SetIpInfoCallback(
  livox_status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *)
{
  if (response == nullptr)
  {
    std::fprintf(stderr, "SetLivoxLidarIp response is null (handle %u).\n", handle);
    operation_finished = true;
    return;
  }

  std::printf("SetLivoxLidarIp response: ret_code=%u error_key=%u\n",
    response->ret_code, response->error_key);
  if (response->ret_code != 0 || response->error_key != 0)
  {
    operation_finished = true;
    return;
  }

  std::printf("IP updated; rebooting lidar to apply the change.\n");
  LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo *info, void *)
{
  if (info == nullptr || operation_finished || request_started)
  {
    return;
  }

  if (current_ip != info->lidar_ip)
  {
    return;
  }

  request_started = true;
  std::printf("Found lidar at %s (handle %u); setting IP to %s.\n",
    current_ip.c_str(), handle, desired_ip.c_str());
  std::printf("Network settings: mask=%s gateway=%s\n",
    subnet_mask.c_str(), gateway_ip.c_str());

  LivoxLidarIpInfo ip_info{};
  std::strncpy(ip_info.ip_addr, desired_ip.c_str(), sizeof(ip_info.ip_addr) - 1);
  std::strncpy(ip_info.net_mask, subnet_mask.c_str(), sizeof(ip_info.net_mask) - 1);
  std::strncpy(ip_info.gw_addr, gateway_ip.c_str(), sizeof(ip_info.gw_addr) - 1);
  SetLivoxLidarIp(handle, &ip_info, SetIpInfoCallback, nullptr);
}

bool is_ipv4_address(const std::string &address)
{
  in_addr parsed{};
  return inet_pton(AF_INET, address.c_str(), &parsed) == 1;
}

std::string default_gateway_for(const std::string &address)
{
  in_addr parsed{};
  if (inet_pton(AF_INET, address.c_str(), &parsed) != 1)
  {
    return {};
  }

  reinterpret_cast<unsigned char *>(&parsed.s_addr)[3] = 1;
  char gateway[INET_ADDRSTRLEN]{};
  if (inet_ntop(AF_INET, &parsed, gateway, sizeof(gateway)) == nullptr)
  {
    return {};
  }
  return gateway;
}
}  // namespace

int main(int argc, char *argv[])
{
  if (argc < 3 || argc > 5)
  {
    std::fprintf(stderr,
      "Usage: %s <current_lidar_ip> <desired_lidar_ip> [subnet_mask] [gateway]\n",
      argv[0]);
    return 2;
  }

  current_ip = argv[1];
  desired_ip = argv[2];
  subnet_mask = argc >= 4 ? argv[3] : "255.255.255.0";
  gateway_ip = argc >= 5 ? argv[4] : default_gateway_for(desired_ip);
  if (!is_ipv4_address(current_ip) || !is_ipv4_address(desired_ip) ||
    !is_ipv4_address(subnet_mask) || !is_ipv4_address(gateway_ip))
  {
    std::fprintf(stderr, "Current IP, desired IP, subnet mask, and gateway must be valid IPv4 addresses.\n");
    return 2;
  }

  const auto package_share = ament_index_cpp::get_package_share_directory("livox_ros_driver2");
  const auto config = package_share + "/config/mixed_MID360s_MID360_config.json";
  if (!LivoxLidarSdkInit(config.c_str()))
  {
    std::fprintf(stderr, "Livox SDK initialization failed using %s.\n", config.c_str());
    LivoxLidarSdkUninit();
    return 1;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  constexpr int timeout_seconds = 30;
  for (int i = 0; i < timeout_seconds * 10 && !operation_finished; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!operation_finished)
  {
    std::fprintf(stderr, "Timed out waiting for lidar at %s.\n", current_ip.c_str());
  }
  else if (!request_started)
  {
    std::fprintf(stderr, "No lidar at %s was found.\n", current_ip.c_str());
  }

  LivoxLidarSdkUninit();
  return operation_result;
}