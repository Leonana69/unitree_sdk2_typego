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
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <errno.h>
#include <sys/resource.h>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#define TOPIC_HIGHSTATE "rt/sportmodestate"
using namespace unitree::common;
using namespace unitree::robot;

// Statistics for PointCloudCallback
static std::atomic<uint64_t> pointcloud_callback_count(0);
static std::atomic<std::chrono::steady_clock::time_point> last_log_time(std::chrono::steady_clock::now());

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

    // Optimize socket for low latency and high throughput
    int sendbuf_size = 4 * 1024 * 1024;  // 4MB send buffer
    if (setsockopt(udp_socket, SOL_SOCKET, SO_SNDBUF, &sendbuf_size, sizeof(sendbuf_size)) < 0) {
        perror("setsockopt SO_SNDBUF failed");
    }
    
    // Set IP_TOS for low latency (DSCP EF - Expedited Forwarding)
    int tos = IPTOS_LOWDELAY | 0xE0;  // Low delay + DSCP EF (0xE0)
    if (setsockopt(udp_socket, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) < 0) {
        perror("setsockopt IP_TOS failed");
    }
    
    // Note: UDP doesn't have Nagle's algorithm (that's TCP-only), so no need to disable it

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(udp_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("bind failed");
        close(udp_socket);
        return;
    }
    
    int actual_sendbuf = 0;
    socklen_t len = sizeof(actual_sendbuf);
    getsockopt(udp_socket, SOL_SOCKET, SO_SNDBUF, &actual_sendbuf, &len);
    printf("UDPServer initialized on port %d (send buffer: %d bytes)\n", port, actual_sendbuf);
    WaitForNextClient();
}

// Optimized send function with error handling
inline void sendtoClient(const uint8_t* data, size_t len) {
    if (client_ready.load()) {
        ssize_t sent = sendto(udp_socket, data, len, MSG_DONTWAIT, 
                              (sockaddr*)&client_addr, client_addr_len);
        if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            // Only log persistent errors, not temporary buffer full conditions
            static int error_count = 0;
            if (++error_count % 1000 == 0) {
                fprintf(stderr, "UDP send error: %s (count: %d)\n", strerror(errno), error_count);
            }
        }
    }
}

// Optimized point structure (xyz only, int16)
#pragma pack(push, 1)
struct OptimizedPoint {
    int16_t x;  // mm, converted from int32
    int16_t y;  // mm, converted from int32
    int16_t z;  // mm, converted from int32
};

// Packet header structure for optimized point cloud transmission
struct PointCloudPacketHeader {
    uint16_t sequence_id;
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
    uint32_t handle;
    uint8_t dev_type;
    uint8_t data_type;
    uint16_t point_count;
    // OptimizedPoint[] follows immediately after header
};

// IMU data structure (packed for network transmission)
struct ImuData {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

// Packet header structure for IMU data transmission
struct ImuPacketHeader {
    uint16_t sequence_id;
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
    uint32_t handle;
    uint8_t dev_type;
    uint8_t data_type;  // Will contain IMU data type from Livox SDK
    uint16_t imu_count;  // Number of IMU samples in this packet
    // ImuData[] follows immediately after header
};
#pragma pack(pop)

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
    // Increment callback counter
    ++pointcloud_callback_count;
    
    // Check if 1 second has passed since last log
    auto log_now = std::chrono::steady_clock::now();
    auto last_log = last_log_time.load();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_log).count();
    
    if (elapsed >= 1000) {
        // Try to update the last log time (only one thread will succeed)
        if (last_log_time.compare_exchange_weak(last_log, log_now)) {
            uint64_t count = pointcloud_callback_count.exchange(0);  // Get and reset counter atomically
            printf("[PointCloudCallback] Called %lu times in the last second\n", count);
        }
    }
    
    if (!data || !client_ready.load()) return;
    
    // Support multiple data types for full data transmission
    if (data->data_type != kLivoxLidarCartesianCoordinateHighData &&
        data->data_type != kLivoxLidarCartesianCoordinateLowData) {
        return;
    }
    
    // Use MTU size for optimal network performance (Ethernet MTU - IP/UDP headers = 1472 bytes)
    constexpr size_t MAX_UDP_PAYLOAD = 1472;
    constexpr size_t HEADER_SIZE = sizeof(PointCloudPacketHeader);
    constexpr size_t OPTIMIZED_POINT_SIZE = sizeof(OptimizedPoint);  // 6 bytes (xyz only, int16)
    constexpr size_t MAX_POINTS_PER_PACKET = (MAX_UDP_PAYLOAD - HEADER_SIZE) / OPTIMIZED_POINT_SIZE;
    
    static thread_local uint16_t seq_id = 0;
    uint8_t packet[MAX_UDP_PAYLOAD];
    
    uint32_t total_points = data->dot_num;
    LivoxLidarCartesianHighRawPoint* src_pts = (LivoxLidarCartesianHighRawPoint*)data->data;
    
    // Use the LiDAR's own hardware timestamp for precise timing
    uint64_t lidar_timestamp;
    memcpy(&lidar_timestamp, data->timestamp, sizeof(uint64_t));
    uint32_t ts_sec  = (uint32_t)(lidar_timestamp / 1000000000ULL);
    uint32_t ts_nsec = (uint32_t)(lidar_timestamp % 1000000000ULL);

    uint32_t remaining_points = total_points;
    uint32_t point_offset = 0;
    
    // Send all points in multiple packets if needed (NO FILTERING - full data transmission!)
    while (remaining_points > 0) {
        uint32_t points_in_packet = (remaining_points > MAX_POINTS_PER_PACKET) 
                                     ? MAX_POINTS_PER_PACKET 
                                     : remaining_points;
        
        // Build packet header
        PointCloudPacketHeader* header = (PointCloudPacketHeader*)packet;
        header->sequence_id = htons(++seq_id);
        header->timestamp_sec = htonl(ts_sec);
        header->timestamp_nsec = htonl(ts_nsec);
        header->handle = htonl(handle);
        header->dev_type = dev_type;
        header->data_type = data->data_type;
        header->point_count = htons(points_in_packet);
        
        // Convert and pack optimized points (xyz only, int32->int16)
        OptimizedPoint* opt_pts = (OptimizedPoint*)(packet + HEADER_SIZE);
        for (uint32_t i = 0; i < points_in_packet; ++i) {
            const LivoxLidarCartesianHighRawPoint& src = src_pts[point_offset + i];
            opt_pts[i].x = (int16_t)(src.x);  // Direct cast (user confirmed int16 is sufficient)
            opt_pts[i].y = (int16_t)(src.y);
            opt_pts[i].z = (int16_t)(src.z);
            // Reflectivity and tag are omitted for bandwidth optimization
        }
        
        // Send packet immediately (non-blocking, low latency)
        size_t packet_size = HEADER_SIZE + (points_in_packet * OPTIMIZED_POINT_SIZE);
        sendtoClient(packet, packet_size);
        
        point_offset += points_in_packet;
        remaining_points -= points_in_packet;
    }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr || !client_ready.load()) {
        return;
    }
    
    // Use MTU size for optimal network performance
    constexpr size_t MAX_UDP_PAYLOAD = 1472;
    constexpr size_t HEADER_SIZE = sizeof(ImuPacketHeader);
    constexpr size_t IMU_DATA_SIZE = sizeof(ImuData);
    constexpr size_t MAX_IMU_PER_PACKET = (MAX_UDP_PAYLOAD - HEADER_SIZE) / IMU_DATA_SIZE;
    
    static thread_local uint16_t imu_seq_id = 0;
    uint8_t packet[MAX_UDP_PAYLOAD];
    
    uint32_t total_imu_samples = data->dot_num;
    LivoxLidarImuRawPoint* src_imu = (LivoxLidarImuRawPoint*)data->data;
    
    // Use the LiDAR's own hardware timestamp for precise timing
    uint64_t lidar_timestamp;
    memcpy(&lidar_timestamp, data->timestamp, sizeof(uint64_t));
    uint32_t ts_sec  = (uint32_t)(lidar_timestamp / 1000000000ULL);
    uint32_t ts_nsec = (uint32_t)(lidar_timestamp % 1000000000ULL);

    uint32_t remaining_samples = total_imu_samples;
    uint32_t imu_offset = 0;
    
    // Send all IMU samples in multiple packets if needed
    while (remaining_samples > 0) {
        uint32_t samples_in_packet = (remaining_samples > MAX_IMU_PER_PACKET) 
                                     ? MAX_IMU_PER_PACKET 
                                     : remaining_samples;
        
        // Build IMU packet header
        ImuPacketHeader* header = (ImuPacketHeader*)packet;
        header->sequence_id = htons(++imu_seq_id);
        header->timestamp_sec = htonl(ts_sec);
        header->timestamp_nsec = htonl(ts_nsec);
        header->handle = htonl(handle);
        header->dev_type = dev_type;
        header->data_type = data->data_type;  // Preserve original IMU data type
        header->imu_count = htons(samples_in_packet);
        
        // Copy IMU data (convert from LivoxLidarImuRawPoint to ImuData)
        ImuData* imu_data = (ImuData*)(packet + HEADER_SIZE);
        for (uint32_t i = 0; i < samples_in_packet; ++i) {
            const LivoxLidarImuRawPoint& src = src_imu[imu_offset + i];
            imu_data[i].acc_x = src.acc_x;
            imu_data[i].acc_y = src.acc_y;
            imu_data[i].acc_z = src.acc_z;
            imu_data[i].gyro_x = src.gyro_x;
            imu_data[i].gyro_y = src.gyro_y;
            imu_data[i].gyro_z = src.gyro_z;
        }
        
        // Send packet immediately (non-blocking, low latency)
        size_t packet_size = HEADER_SIZE + (samples_in_packet * IMU_DATA_SIZE);
        sendtoClient(packet, packet_size);
        
        imu_offset += samples_in_packet;
        remaining_samples -= samples_in_packet;
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

int main(int argc, const char *argv[]) {
    if (argc != 2) {
        printf("Params Invalid, must input config path.\n");
        return -1;
    }
    const std::string path = argv[1];

    // Set high priority for real-time performance
    setpriority(PRIO_PROCESS, 0, -10);  // Higher priority (may require root)
    
    // Set CPU affinity to a dedicated core if available (optional optimization)
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // CPU_SET(0, &cpuset);  // Use CPU 0
    // pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

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
    // This callback will transmit ALL points without filtering
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
    
    // OPTIONAL, to get imu data via 'ImuDataCallback'
    // some lidar types DO NOT contain an imu component
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
    
    // SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    
    // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

    printf("Full point cloud service ready. Transmitting all lidar data with minimal latency.\n");
    printf("Packet format: [Header: seq|timestamp|handle|dev_type|data_type|point_count] + [Points]\n");

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LivoxLidarSdkUninit();
    printf("Livox Full Service End!\n");
    return 0;
}