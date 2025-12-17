#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <vector>
#include <chrono>
#include <mutex>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#define TOPIC_HIGHSTATE "rt/sportmodestate"
using namespace unitree::common;
using namespace unitree::robot;

int udp_socket;
std::atomic<bool> client_ready(false);
sockaddr_in client_addr;
socklen_t client_addr_len = sizeof(client_addr);

void WaitForNextClient() {
    std::thread([] {
        uint8_t buf[1];
        sockaddr_in latest_client;
        socklen_t latest_client_len = sizeof(latest_client);

        while (true) {
            int n = recvfrom(udp_socket, buf, sizeof(buf), 0,
                             (sockaddr*)&latest_client, &latest_client_len);
            if (n > 0) {
                // Update the global client address atomically
                std::memcpy(&client_addr, &latest_client, sizeof(sockaddr_in));
                client_addr_len = latest_client_len;
                if (!client_ready.exchange(true)) {
                    printf("[UDP] First client connected from %s:%d\n",
                           inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                } else {
                    printf("[UDP] Switched to new client %s:%d\n",
                           inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                }
            }
        }
    }).detach();
}

void InitUDPServer(uint16_t port) {
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        perror("socket creation failed");
        return;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(udp_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(udp_socket);
        return;
    }
    printf("UDPServer initialized on port %d\n", port);
    WaitForNextClient();
}

void sendtoClient(const uint8_t* data, size_t len) {
    if (client_ready.load()) {
        sendto(udp_socket, data, len, 0, (sockaddr*)&client_addr, client_addr_len);
    }
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
    if (!data || data->data_type != kLivoxLidarCartesianCoordinateHighData) return;
    LivoxLidarCartesianHighRawPoint* pts = (LivoxLidarCartesianHighRawPoint*)data->data;

    static thread_local uint16_t seq_id = 0;
    constexpr size_t MAX_UDP_PAYLOAD = 512;
    uint8_t packet[MAX_UDP_PAYLOAD];
    memcpy(&packet[0], &seq_id, sizeof(seq_id));
    size_t offset = sizeof(seq_id);

    for (uint32_t i = 0; i < data->dot_num; ++i) {
        int16_t x = (int16_t)(pts[i].x);
        int16_t y = (int16_t)(pts[i].y);
        int16_t z = (int16_t)(pts[i].z);
        if (z > -80 && z < 80) {
            if (offset + 6 > MAX_UDP_PAYLOAD) {
                sendtoClient(packet, offset);
                seq_id++;
                offset = sizeof(seq_id);
                memcpy(&packet[0], &seq_id, sizeof(seq_id));
            }
            memcpy(&packet[offset], &x, 2);
            memcpy(&packet[offset + 2], &y, 2);
            memcpy(&packet[offset + 4], &z, 2);
            offset += 6;
        }
    }

    if (offset > sizeof(seq_id)) {
        sendtoClient(packet, offset);
        seq_id++;
    }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle, 
    LivoxLidarDiagInternalInfoResponse* response, void* client_data) {
    if (status != kLivoxLidarStatusSuccess) {
        printf("Query lidar internal info failed.\n");
        // QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
        return;
    }

    if (response == nullptr) {
        return;
    }

    uint8_t host_point_ipaddr[4] {0};
    uint16_t host_point_port = 0;
    uint16_t lidar_point_port = 0;

    uint8_t host_imu_ipaddr[4] {0};
    uint16_t host_imu_data_port = 0;
    uint16_t lidar_imu_data_port = 0;

    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i) {
        LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
        if (kv->key == kKeyLidarPointDataHostIpCfg) {
            memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
        } else if (kv->key == kKeyLidarImuHostIpCfg) {
            memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }

    printf("Host point cloud ip addr:%u.%u.%u.%u, host point cloud port:%u, lidar point cloud port:%u.\n",
        host_point_ipaddr[0], host_point_ipaddr[1], host_point_ipaddr[2], host_point_ipaddr[3], host_point_port, lidar_point_port);

    printf("Host imu ip addr:%u.%u.%u.%u, host imu port:%u, lidar imu port:%u.\n",
        host_imu_ipaddr[0], host_imu_ipaddr[1], host_imu_ipaddr[2], host_imu_ipaddr[3], host_imu_data_port, lidar_imu_data_port);

}

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr) {
        return;
    }
    printf("WorkModeCallack, status:%u, handle:%u, ret_code:%u, error_key:%u",
        status, handle, response->ret_code, response->error_key);
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
    if (info == nullptr) {
        printf("lidar info change callback failed, the info is nullptr.\n");
        return;
    } 
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    
    // set the work mode to kLivoxLidarNormal, namely start the lidar
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);
  
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
}

// void _HighStateHandler(const void *message) {
//     using namespace std::chrono;
//     static auto last_send_time = steady_clock::now();
//     auto now = steady_clock::now();
//     if (duration_cast<milliseconds>(now - last_send_time).count() < 20) {
//         return;  // Skip if less than 20ms since last send
//     }
//     last_send_time = now;

//     const auto* high_state = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);

//     float data[7];
//     data[0] = high_state->position()[0];
//     data[1] = high_state->position()[1];
//     data[2] = high_state->position()[2];
//     data[3] = high_state->imu_state().quaternion()[0];
//     data[4] = high_state->imu_state().quaternion()[1];
//     data[5] = high_state->imu_state().quaternion()[2];
//     data[6] = high_state->imu_state().quaternion()[3];

//     // printf("Position: %f, %f, %f\n", data[0], data[1], data[2]);
//     // printf("IMU quaternion: %f, %f, %f, %f\n", data[3], data[4], data[5], data[6]);

//     // sendto(udp_socket, data, sizeof(data), 0, (sockaddr*)&target_addr, sizeof(target_addr));
//     uint8_t buffer[1 + sizeof(data)];
//     buffer[0] = MSG_HIGHSTATE;
//     memcpy(buffer + 1, data, sizeof(data));
//     sendtoClient(buffer, sizeof(buffer));
// };

int main(int argc, const char *argv[]) {
    if (argc != 2) {
        printf("Params Invalid, must input config path.\n");
        return -1;
    }
    const std::string path = argv[1];

    // REQUIRED, to init Livox SDK2
    if (!LivoxLidarSdkInit(path.c_str())) {
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        return -1;
    } else {
        printf("Livox Init Success\n");
    }

    InitUDPServer(8888);
    
    // REQUIRED, to get point cloud data via 'PointCloudCallback'
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    
    // // OPTIONAL, to get imu data via 'ImuDataCallback'
    // // some lidar types DO NOT contain an imu component
    // SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    
    // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    
    // // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LivoxLidarSdkUninit();
    printf("Livox Quick Start Demo End!\n");
    return 0;
}