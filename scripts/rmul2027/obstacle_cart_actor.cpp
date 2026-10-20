#include <ignition/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/entity_factory.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/contacts.pb.h>
#include <google/protobuf/util/json_util.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

int main(int argc, char **argv)
{
    if (argc != 3) return 2;
    const std::string sdfPath = argv[1], record = argv[2];
    ignition::transport::Node node;
    std::mutex mutex;
    std::atomic<double> sim{0.0};
    std::atomic<unsigned long> frames{0}, contacts{0}, probeContacts{0};
    std::atomic<bool> calibrating{false};
    std::ofstream raw(record + "/contacts.jsonl"), poses(record + "/gazebo-poses.jsonl");
    double lastPose = -1.0, x = 15.2, y = 23.0;
    auto health = [&]() {
        std::ofstream f(record + "/contact-health.json");
        f << "{\"sim\":" << std::setprecision(15) << sim.load()
          << ",\"frames\":" << frames.load() << ",\"navigation_contacts\":" << contacts.load()
          << ",\"probe_contacts\":" << probeContacts.load() << "}";
    };
    node.Subscribe("/world/default/clock", std::function<void(const ignition::msgs::Clock &)>(
        [&](const ignition::msgs::Clock &msg) {
            sim = msg.sim().sec() + msg.sim().nsec() * 1e-9;
        }));
    std::function<void(const ignition::msgs::Contacts &)> contactCallback = [&](const ignition::msgs::Contacts &msg) {
            ++frames;
            if (!msg.contact_size()) return;
            std::lock_guard<std::mutex> lock(mutex);
            bool probe = calibrating.load();
            for (const auto &c : msg.contact()) {
                if (c.collision1().name().find("c020_contact_probe") != std::string::npos ||
                    c.collision2().name().find("c020_contact_probe") != std::string::npos) probe = true;
            }
            if (probe) ++probeContacts; else ++contacts;
            std::string json;
            google::protobuf::util::MessageToJsonString(msg, &json);
            raw << "{\"sim\":" << std::setprecision(15) << sim.load()
                << ",\"probe\":" << (probe ? "true" : "false") << ",\"message\":" << json << "}\n";
            raw.flush();
        };
    node.Subscribe("/c020/cart_contacts", contactCallback);
    node.Subscribe("/world/default/model/c020_opponent_cart/link/cart/sensor/cart_contact/contact", contactCallback);
    node.Subscribe("/world/default/pose/info", std::function<void(const ignition::msgs::Pose_V &)>(
        [&](const ignition::msgs::Pose_V &msg) {
            std::lock_guard<std::mutex> lock(mutex);
            double now = sim.load();
            if (now - lastPose < .05) return;
            lastPose = now;
            for (const auto &p : msg.pose()) {
                if (p.name() != "c020_opponent_cart" && p.name() != "red_standard_robot1") continue;
                poses << std::setprecision(15) << "{\"sim\":" << now << ",\"name\":\"" << p.name()
                      << "\",\"id\":" << p.id() << ",\"xyz\":[" << p.position().x() << ','
                      << p.position().y() << ',' << p.position().z() << "]}\n";
            }
            poses.flush();
        }));
    auto request = [&](const std::string &service, const auto &req, unsigned timeout = 5000) {
        ignition::msgs::Boolean reply;
        bool result = false;
        return node.Request(service, req, timeout, reply, result) && result && reply.data();
    };
    ignition::msgs::EntityFactory bootstrap;
    bootstrap.set_sdf("<sdf version='1.7'><model name='c020_contact_system'><static>true</static><link name='system'/><plugin filename='libignition-gazebo-contact-system.so' name='ignition::gazebo::systems::Contact'/></model></sdf>");
    if (!request("/world/default/create", bootstrap, 10000)) return 5;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ignition::msgs::EntityFactory factory;
    factory.set_sdf_filename(sdfPath);
    if (!request("/world/default/create", factory, 10000)) {
        std::cout << "{\"event\":\"create_failed\"}" << std::endl;
        return 3;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    std::cout << "{\"event\":\"created\"}" << std::endl;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string command;
        input >> command;
        if (command == "QUIT") break;
        if (command == "POSE") {
            double mx, my, yaw;
            if (!(input >> mx >> my >> yaw)) return 4;
            x = mx - 4.8; y = my + 3.0;
            ignition::msgs::Pose pose;
            pose.set_name("c020_opponent_cart");
            pose.mutable_position()->set_x(x); pose.mutable_position()->set_y(y);
            pose.mutable_position()->set_z(0.0);
            pose.mutable_orientation()->set_z(std::sin(yaw / 2));
            pose.mutable_orientation()->set_w(std::cos(yaw / 2));
            bool ok = request("/world/default/set_pose", pose);
            std::cout << std::setprecision(15) << "{\"event\":\"pose\",\"ok\":" << (ok ? "true" : "false")
                      << ",\"sim\":" << sim.load() << ",\"map_xy\":[" << mx << ',' << my << "]}" << std::endl;
        } else if (command == "PROBE") {
            calibrating = true;
            std::ostringstream sdf;
            sdf << std::setprecision(15) << "<sdf version='1.7'><model name='c020_contact_probe'><pose>"
                << x << ' ' << y << " 0.27 0 0 0</pose><link name='probe'><gravity>false</gravity>"
                << "<inertial><mass>0.01</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz>"
                << "</inertia></inertial><collision name='probe_collision'><geometry><box><size>0.1 0.1 0.1</size>"
                << "</box></geometry></collision></link></model></sdf>";
            ignition::msgs::EntityFactory probe;
            probe.set_sdf(sdf.str());
            bool made = request("/world/default/create", probe);
            std::this_thread::sleep_for(std::chrono::milliseconds(700));
            ignition::msgs::Entity entity;
            entity.set_name("c020_contact_probe"); entity.set_type(ignition::msgs::Entity::MODEL);
            bool removed = request("/world/default/remove", entity);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            calibrating = false;
            health();
            std::cout << "{\"event\":\"probe\",\"created\":" << (made ? "true" : "false")
                      << ",\"removed\":" << (removed ? "true" : "false") << ",\"contact_messages\":"
                      << probeContacts.load() << ",\"other_contacts\":" << contacts.load() << "}" << std::endl;
        } else if (command == "HEALTH") {
            health();
            std::cout << "{\"event\":\"health\",\"frames\":" << frames.load()
                      << ",\"contacts\":" << contacts.load() << ",\"probe_contacts\":" << probeContacts.load() << "}" << std::endl;
        }
    }
    health();
    ignition::msgs::Entity entity;
    entity.set_name("c020_opponent_cart"); entity.set_type(ignition::msgs::Entity::MODEL);
    request("/world/default/remove", entity);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    entity.set_name("c020_contact_system");
    request("/world/default/remove", entity);
    return 0;
}
