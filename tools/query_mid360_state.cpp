#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace {

bool g_enable_point_send = false;
bool g_reboot = false;

template <typename T>
T ReadValue(const uint8_t* data) {
  T value{};
  std::memcpy(&value, data, sizeof(value));
  return value;
}

void QueryCallback(livox_status status, uint32_t handle,
                   LivoxLidarDiagInternalInfoResponse* response, void*) {
  std::cout << "query_status=" << status << " handle=" << handle;
  if (response == nullptr) {
    std::cout << " response=null\n";
    return;
  }
  std::cout << " ret_code=" << static_cast<unsigned>(response->ret_code)
            << " param_num=" << response->param_num << '\n';

  uint16_t offset = 0;
  for (uint16_t i = 0; i < response->param_num; ++i) {
    auto* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(&response->data[offset]);
    const uint8_t* value = kv->value;
    std::cout << "key=0x" << std::hex << std::setw(4) << std::setfill('0')
              << kv->key << std::dec << " len=" << kv->length;
    switch (kv->key) {
      case kKeyPointSendEn:
        std::cout << " point_send_en=" << static_cast<unsigned>(value[0]);
        break;
      case kKeyLidarPointDataHostIpCfg:
        std::cout << " point_host=" << static_cast<unsigned>(value[0]) << '.'
                  << static_cast<unsigned>(value[1]) << '.'
                  << static_cast<unsigned>(value[2]) << '.'
                  << static_cast<unsigned>(value[3])
                  << " host_port=" << ReadValue<uint16_t>(value + 4)
                  << " lidar_port=" << ReadValue<uint16_t>(value + 6);
        break;
      case kKeyLidarImuHostIpCfg:
        std::cout << " imu_host=" << static_cast<unsigned>(value[0]) << '.'
                  << static_cast<unsigned>(value[1]) << '.'
                  << static_cast<unsigned>(value[2]) << '.'
                  << static_cast<unsigned>(value[3])
                  << " host_port=" << ReadValue<uint16_t>(value + 4)
                  << " lidar_port=" << ReadValue<uint16_t>(value + 6);
        break;
      case kKeyWorkMode:
        std::cout << " work_target=" << static_cast<unsigned>(value[0]);
        break;
      case kKeyCurWorkState:
        std::cout << " current_work_state=" << static_cast<unsigned>(value[0]);
        break;
      case kKeyStatusCode:
        std::cout << " status_hex=";
        for (uint16_t j = 0; j < kv->length; ++j) {
          std::cout << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<unsigned>(value[j]);
        }
        std::cout << std::dec;
        break;
      case kKeyHmsCode:
        std::cout << " hms=";
        for (uint16_t j = 0; j + sizeof(uint32_t) <= kv->length;
             j += sizeof(uint32_t)) {
          std::cout << " 0x" << std::hex << std::setw(8) << std::setfill('0')
                    << ReadValue<uint32_t>(value + j) << std::dec;
        }
        break;
      default:
        break;
    }
    std::cout << '\n';
    offset += sizeof(uint16_t) * 2 + kv->length;
  }
}

void EnablePointCallback(livox_status status, uint32_t handle,
                         LivoxLidarAsyncControlResponse* response, void*) {
  std::cout << "enable_point_send status=" << status << " handle=" << handle;
  if (response != nullptr) {
    std::cout << " ret_code=" << static_cast<unsigned>(response->ret_code)
              << " error_key=" << response->error_key;
  }
  std::cout << '\n';
}

void RebootCallback(livox_status status, uint32_t handle,
                    LivoxLidarRebootResponse* response, void*) {
  std::cout << "reboot status=" << status << " handle=" << handle;
  if (response != nullptr) {
    std::cout << " ret_code=" << static_cast<unsigned>(response->ret_code);
  }
  std::cout << '\n';
}

void InfoChanged(const uint32_t handle, const LivoxLidarInfo* info, void*) {
  std::cout << "detected handle=" << handle
            << " ip=" << (info ? info->lidar_ip : "unknown")
            << " sn=" << (info ? info->sn : "unknown") << '\n';
  QueryLivoxLidarInternalInfo(handle, QueryCallback, nullptr);
  if (g_enable_point_send) {
    EnableLivoxLidarPointSend(handle, EnablePointCallback, nullptr);
  }
  if (g_reboot) {
    LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
  }
}

void PushInfo(const uint32_t handle, const uint8_t, const char* info, void*) {
  std::cout << "push handle=" << handle << " " << (info ? info : "null") << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: query_mid360_state CONFIG.json "
                 "[--enable-point-send|--reboot]\n";
    return 2;
  }
  g_enable_point_send = argc == 3 && std::string(argv[2]) == "--enable-point-send";
  g_reboot = argc == 3 && std::string(argv[2]) == "--reboot";
  if (!LivoxLidarSdkInit(argv[1])) {
    std::cerr << "LivoxLidarSdkInit failed\n";
    return 1;
  }
  SetLivoxLidarInfoCallback(PushInfo, nullptr);
  SetLivoxLidarInfoChangeCallback(InfoChanged, nullptr);
  std::this_thread::sleep_for(std::chrono::seconds(8));
  LivoxLidarSdkUninit();
  return 0;
}
