#include "call_back/livox_lidar_callback.h"

#include <cstddef>
#include <iostream>

namespace {

#pragma pack(push, 1)
struct VersionParameter {
  uint16_t key;
  uint16_t length;
  uint8_t value[4];
};

struct VersionResponse {
  uint8_t ret_code;
  uint16_t param_num;
  VersionParameter parameters[2];
};
#pragma pack(pop)

static_assert(offsetof(VersionResponse, parameters) ==
              offsetof(LivoxLidarDiagInternalInfoResponse, data), "SDK response layout changed");

bool CheckVersions(const livox_ros::LidarDeviceInfo& info,
                   const std::string& firmware, const std::string& hardware) {
  if (info.fw_version != firmware || info.hw_version != hardware) {
    std::cerr << "Unexpected metadata versions: firmware=" << info.fw_version
              << ", hardware=" << info.hw_version << std::endl;
    return false;
  }
  return true;
}

}  // namespace

int main() {
  using namespace livox_ros;
  LdsLidar* lds = LdsLidar::GetInstance(10.0);
  uint32_t handle = IpStringToNum("192.168.0.11");
  uint8_t index = 0;
  if (lds->cache_index_.GetFreeIndex(kLivoxLidarType, handle, index) != 0) {
    std::cerr << "Could not allocate test lidar" << std::endl;
    return 1;
  }
  LidarDeviceInfo& info = lds->lidars_[index].info;
  VersionResponse response{0, 2, {
      {kKeyVersionHardware, 4, {10, 20, 30, 40}},
      {kKeyVersionApp, 4, {255, 128, 1, 0}},
  }};
  auto* sdk_response = reinterpret_cast<LivoxLidarDiagInternalInfoResponse*>(&response);
  auto query = [&] {
    LivoxLidarCallback::QueryInternalInfoCallback(
        kLivoxLidarStatusSuccess, handle, sdk_response, lds);
  };
  auto check_unchanged = [&] { return CheckVersions(info, "255.128.1.0", "10.20.30.40"); };

  query();
  if (!check_unchanged()) return 1;

  LivoxLidarCallback::QueryInternalInfoCallback(kLivoxLidarStatusTimeout, handle, nullptr, lds);
  if (!check_unchanged()) return 1;
  LivoxLidarCallback::QueryInternalInfoCallback(kLivoxLidarStatusSuccess, handle, nullptr, lds);
  if (!check_unchanged()) return 1;

  response.ret_code = 32;
  query();
  if (!check_unchanged()) return 1;
  response.ret_code = 0;

  response.param_num = 3;
  query();
  if (!check_unchanged()) return 1;
  response.param_num = 2;

  response.parameters[1].length = 3;
  query();
  if (!check_unchanged()) return 1;
  response.parameters[1].length = 4;

  response.parameters[1].key = kKeyProductInfo;
  query();
  if (!check_unchanged()) return 1;
  response.parameters[1].key = kKeyVersionApp;

  response.param_num = 1;
  query();
  if (!CheckVersions(info, "", "10.20.30.40")) return 1;

  response.param_num = 0;
  query();
  if (!CheckVersions(info, "", "")) return 1;

  response.param_num = 2;
  response.parameters[0] = {kKeyVersionApp, 4, {35, 1, 1, 8}};
  response.parameters[1] = {kKeyVersionHardware, 4, {0, 0, 0, 0}};
  query();
  if (!CheckVersions(info, "35.1.1.8", "0.0.0.0")) return 1;

  std::cout << "PASS: version ordering, unsigned bytes, device-reported zero versions, "
               "missing fields, and failed/malformed responses" << std::endl;
  return 0;
}
