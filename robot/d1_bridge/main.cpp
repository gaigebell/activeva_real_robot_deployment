// d1_bridge：D1 驱动桥
//   本地 TCP(JSON line)  <->  DDS(rt/arm_Command / current_servo_angle)
// 每臂一个实例；双臂按官方方案一（绑网卡）或方案二（改 topic 后缀）区分（docs/protocol_d1.md）。
//
// 用法:
//   d1_bridge --port 5500 [--topic rt/arm_Command]
//             [--feedback-topic current_servo_angle] [--interface eth0] [--address 1]
//
// TODO(现场):
//   - 核对 PubServoInfo_ 字段名与 7 值映射（参考 d1_sdk/src/get_arm_joint_angle.cpp）
//   - 夹爪语义（NQ5）：angle6 单位与范围
//   - 指令频率上限实测；必要时加节流/平滑（mode 0/1）
//   - 安全：无客户端时是否自动卸力（默认不动作，仅转发）
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

using namespace unitree::robot;
using unitree_arm::msg::dds_::ArmString_;
using unitree_arm::msg::dds_::PubServoInfo_;

static std::atomic<int> g_seq{1};
static ChannelPublisher<ArmString_>* g_pub = nullptr;

// ---- 最新反馈（臂官方上传 10 Hz）----
static float g_angles[7] = {0};
static bool g_has_feedback = false;
static std::mutex g_fb_mutex;

void on_feedback(const void* msg) {
    const auto* fb = static_cast<const PubServoInfo_*>(msg);
    std::lock_guard<std::mutex> lk(g_fb_mutex);
    // TODO(现场): 核对字段名（servo0_data()~servo6_data()）
    g_angles[0] = fb->servo0_data_();
    g_angles[1] = fb->servo1_data_();
    g_angles[2] = fb->servo2_data_();
    g_angles[3] = fb->servo3_data_();
    g_angles[4] = fb->servo4_data_();
    g_angles[5] = fb->servo5_data_();
    g_angles[6] = fb->servo6_data_();
    g_has_feedback = true;
}

// ---- DDS 下发 ----
void send_json(const std::string& json) {
    ArmString_ msg{};
    msg.data_() = json;
    g_pub->Write(msg);
}

// funcode 2：全部关节角度控制（mode 0=10Hz 小平滑 / 1=轨迹大平滑）
void cmd_target(int address, const float angles[7], int mode) {
    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"seq\":%d,\"address\":%d,\"funcode\":2,\"data\":{\"mode\":%d,"
        "\"angle0\":%.4f,\"angle1\":%.4f,\"angle2\":%.4f,\"angle3\":%.4f,"
        "\"angle4\":%.4f,\"angle5\":%.4f,\"angle6\":%.4f}}",
        g_seq.fetch_add(1), address, mode,
        angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]);
    send_json(buf);
}

// funcode 5（使能）/ 6（供电）/ 7（归零）
void cmd_funcode(int address, int funcode, const char* data = "") {
    char buf[512];
    if (data == nullptr || data[0] == '\0')
        snprintf(buf, sizeof(buf), "{\"seq\":%d,\"address\":%d,\"funcode\":%d}",
                 g_seq.fetch_add(1), address, funcode);
    else
        snprintf(buf, sizeof(buf), "{\"seq\":%d,\"address\":%d,\"funcode\":%d,\"data\":%s}",
                 g_seq.fetch_add(1), address, funcode, data);
    send_json(buf);
}

// ---- TCP 服务（JSON line）----
static std::vector<int> g_clients;
static std::mutex g_clients_mutex;

void broadcast(const std::string& line) {
    std::lock_guard<std::mutex> lk(g_clients_mutex);
    for (auto it = g_clients.begin(); it != g_clients.end();) {
        if (send(*it, line.c_str(), line.size(), MSG_NOSIGNAL) <= 0) {
            close(*it);
            it = g_clients.erase(it);
        } else {
            ++it;
        }
    }
}

// 简易 JSON 解析（避免额外依赖）；TODO(现场): 可换 rapidjson
static bool parse_target(const std::string& line, float angles[7], int& mode) {
    size_t p = line.find("\"angles\"");
    if (p == std::string::npos) return false;
    p = line.find('[', p);
    if (p == std::string::npos) return false;
    for (int i = 0; i < 7; ++i) {
        char* end = nullptr;
        angles[i] = strtof(line.c_str() + p + 1, &end);
        if (end == nullptr || *end == '\0') return false;
        p = end - line.c_str();
    }
    p = line.find("\"mode\"");
    mode = 0;
    if (p != std::string::npos)
        mode = atoi(line.c_str() + p + 7);
    return true;
}

void handle_line(const std::string& line, int address) {
    if (line.find("\"target\"") != std::string::npos) {
        float angles[7];
        int mode = 0;
        if (parse_target(line, angles, mode))
            cmd_target(address, angles, mode);
    } else if (line.find("\"enable\"") != std::string::npos) {
        cmd_funcode(address, 5, "{\"mode\":1}");
    } else if (line.find("\"disable\"") != std::string::npos) {
        cmd_funcode(address, 5, "{\"mode\":0}");
    } else if (line.find("\"power_on\"") != std::string::npos) {
        cmd_funcode(address, 6, "{\"power\":1}");
    } else if (line.find("\"power_off\"") != std::string::npos) {
        cmd_funcode(address, 6, "{\"power\":0}");
    } else if (line.find("\"zero\"") != std::string::npos) {
        cmd_funcode(address, 7);
    }
}

void client_thread(int fd, int address) {
    std::string buf;
    char chunk[1024];
    while (true) {
        ssize_t n = recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0) break;
        buf.append(chunk, n);
        size_t pos;
        while ((pos = buf.find('\n')) != std::string::npos) {
            handle_line(buf.substr(0, pos), address);
            buf.erase(0, pos + 1);
        }
    }
    close(fd);
}

void feedback_broadcast_loop() {
    // 反馈到达即转发（官方 10 Hz）
    while (true) {
        usleep(100000);  // 100 ms 轮询
        float angles[7];
        bool has;
        {
            std::lock_guard<std::mutex> lk(g_fb_mutex);
            has = g_has_feedback;
            memcpy(angles, g_angles, sizeof(angles));
        }
        if (!has) continue;
        char buf[512];
        snprintf(buf, sizeof(buf),
            "{\"type\":\"feedback\",\"angles\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}\n",
            angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]);
        broadcast(buf);
    }
}

int main(int argc, char** argv) {
    int port = 5500;
    int address = 1;
    std::string topic = "rt/arm_Command";
    std::string feedback_topic = "current_servo_angle";
    std::string interface = "";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (a == name && i + 1 < argc) return argv[++i];
            return nullptr;
        };
        if (const char* v = next("--port")) port = atoi(v);
        else if (const char* v = next("--address")) address = atoi(v);
        else if (const char* v = next("--topic")) topic = v;
        else if (const char* v = next("--feedback-topic")) feedback_topic = v;
        else if (const char* v = next("--interface")) interface = v;
        else if (a == "--help") {
            printf("用法: d1_bridge --port 5500 [--topic rt/arm_Command] "
                   "[--feedback-topic current_servo_angle] [--interface eth0] [--address 1]\n");
            return 0;
        }
    }

    // DDS 初始化（方案一：绑定网卡）
    if (interface.empty())
        ChannelFactory::Instance()->Init(0);
    else
        ChannelFactory::Instance()->Init(0, interface);

    g_pub = new ChannelPublisher<ArmString_>(topic);
    g_pub->InitChannel();

    auto* sub = new ChannelSubscriber<PubServoInfo_>(feedback_topic);
    sub->InitChannel(on_feedback, 10);

    // TCP 服务
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(listen_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }
    listen(listen_fd, 4);
    printf("[d1_bridge] interface=%s topic=%s feedback=%s, 监听 TCP :%d, address=%d\n",
           interface.empty() ? "<auto>" : interface.c_str(), topic.c_str(),
           feedback_topic.c_str(), port, address);

    std::thread(feedback_broadcast_loop).detach();

    while (true) {
        int fd = accept(listen_fd, nullptr, nullptr);
        if (fd < 0) continue;
        {
            std::lock_guard<std::mutex> lk(g_clients_mutex);
            g_clients.push_back(fd);
        }
        std::thread(client_thread, fd, address).detach();
    }
    return 0;
}
