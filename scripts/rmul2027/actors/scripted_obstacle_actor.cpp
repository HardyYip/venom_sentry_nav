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
    std::atomic<bool> calibrating{false}, motionProbeActive{false};
    std::atomic<unsigned long> motionProbeContacts{0};
    std::atomic<double> motionFirstContact{-1.0};
    std::ofstream raw(record + "/contacts.jsonl"), poses(record + "/gazebo-poses.jsonl");
    std::ofstream moveLog(record + "/move.jsonl");
    double lastPose = -1.0, x = 0.0, y = 0.0;
    std::atomic<bool> moveActive{false}, moveStop{false};
    std::atomic<double> moveProgress{0.0}, moveCmdX{0.0}, moveCmdY{0.0};
    std::thread mover;
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
            bool currentMotionProbe = false;
            for (const auto &c : msg.contact()) {
                if (c.collision1().name().find("c020_contact_probe") != std::string::npos ||
                    c.collision2().name().find("c020_contact_probe") != std::string::npos) probe = true;
                if (c.collision1().name().find("c020_contact_probe::motion_probe::") != std::string::npos ||
                    c.collision2().name().find("c020_contact_probe::motion_probe::") != std::string::npos) currentMotionProbe = true;
            }
            if (probe) ++probeContacts; else ++contacts;
            if (currentMotionProbe && motionProbeActive.load()) {
                ++motionProbeContacts;
                if (motionFirstContact.load() < 0.0) motionFirstContact = sim.load();
            }
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
        if (command == "MOTIONPROBE") {
            calibrating = true; motionFirstContact = -1.0; motionProbeContacts = 0;
            const double originalX=x, originalY=y;
            std::ostringstream sdf;
            sdf << std::setprecision(15) << "<sdf version='1.7'><model name='c020_contact_probe'><pose>"
                << originalX+.8 << ' ' << originalY << " .27 0 0 0</pose><link name='motion_probe'><gravity>false</gravity>"
                << "<inertial><mass>0.01</mass><inertia><ixx>.0001</ixx><iyy>.0001</iyy><izz>.0001</izz></inertia></inertial>"
                << "<collision name='probe_collision'><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model></sdf>";
            ignition::msgs::EntityFactory factory;factory.set_sdf(sdf.str());
            bool made=request("/world/default/create",factory);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const double begin=sim.load();
            motionProbeActive=true;
            auto setModelPose=[&](double px) {
                ignition::msgs::Pose pose;pose.set_name("c020_opponent_cart");
                pose.mutable_position()->set_x(px);pose.mutable_position()->set_y(originalY);
                pose.mutable_position()->set_z(0.0);pose.mutable_orientation()->set_w(1.0);
                return request("/world/default/set_pose",pose,1000);
            };
            bool moved=true;double next=begin;
            while(sim.load()-begin<2.0 && moved) {
                const double now=sim.load();
                if(now<next){std::this_thread::sleep_for(std::chrono::milliseconds(5));continue;}
                moved=setModelPose(originalX+.5*(now-begin));next=now+.033;
                moveLog << "{\"phase\":\"motion_probe\",\"sim\":" << std::setprecision(15) << now
                        << ",\"cmd_world\":[" << originalX+.5*(now-begin) << ',' << originalY << "]}\n";moveLog.flush();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            motionProbeActive=false;
            ignition::msgs::Entity entity;entity.set_name("c020_contact_probe");entity.set_type(ignition::msgs::Entity::MODEL);
            bool removed=request("/world/default/remove",entity);
            moved=setModelPose(originalX)&&moved;
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            calibrating=false;health();
            const auto hits=motionProbeContacts.load();
            const bool ok=made&&removed&&moved&&hits>0&&motionFirstContact.load()>begin+.4;
            std::cout << std::setprecision(15) << "{\"event\":\"motion_probe\",\"ok\":" << (ok?"true":"false")
                      << ",\"created\":" << (made?"true":"false") << ",\"removed\":" << (removed?"true":"false")
                      << ",\"contact_messages\":" << hits << ",\"begin_sim\":" << begin
                      << ",\"first_contact_sim\":" << motionFirstContact.load() << ",\"speed_mps\":0.5,\"distance_m\":1.0}" << std::endl;
        } else if (command == "MOVE") {
            double mx0, my0, mx1, my1, speed;
            if (!(input >> mx0 >> my0 >> mx1 >> my1 >> speed)) return 4;
            if (moveActive.load() || speed <= 0.0) {
                std::cout << "{\"event\":\"move\",\"ok\":false,\"reason\":\"busy_or_bad_speed\"}" << std::endl;
                continue;
            }
            if (mover.joinable()) mover.join();
            double wx0 = mx0 - 4.8, wy0 = my0 + 3.0, wx1 = mx1 - 4.8, wy1 = my1 + 3.0;
            x = wx0; y = wy0;
            moveStop = false; moveProgress = 0.0; moveCmdX = wx0; moveCmdY = wy0;
            moveActive = true;
            mover = std::thread([&, wx0, wy0, wx1, wy1, speed]() {
                ignition::transport::Node moverNode;
                auto setPose = [&](double px, double py) {
                    ignition::msgs::Pose pose;
                    pose.set_name("c020_opponent_cart");
                    pose.mutable_position()->set_x(px); pose.mutable_position()->set_y(py);
                    pose.mutable_position()->set_z(0.0);
                    pose.mutable_orientation()->set_w(1.0);
                    ignition::msgs::Boolean reply;
                    bool result = false;
                    return moverNode.Request("/world/default/set_pose", pose, 1000, reply, result) && result && reply.data();
                };
                const double total = std::hypot(wx1 - wx0, wy1 - wy0);
                const double t0 = sim.load();
                const double duration = total / speed;
                double next = t0;
                setPose(wx0, wy0);
                while (!moveStop.load()) {
                    double now = sim.load();
                    if (now < next) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
                    double f = (now - t0) / duration;
                    if (f > 1.0) f = 1.0;
                    double px = wx0 + (wx1 - wx0) * f, py = wy0 + (wy1 - wy0) * f;
                    if (!setPose(px, py)) { moveStop = true; break; }
                    moveCmdX = px; moveCmdY = py; moveProgress = f;
                    moveLog << std::setprecision(15) << "{\"sim\":" << now << ",\"progress\":" << f
                            << ",\"cmd_world\":[" << px << ',' << py << "]}\n";
                    moveLog.flush();
                    if (f >= 1.0) break;
                    next = now + 0.033;
                }
                moveActive = false;
            });
            std::cout << std::setprecision(15) << "{\"event\":\"move\",\"ok\":true,\"sim\":" << sim.load()
                      << ",\"start_map\":[" << mx0 << ',' << my0 << "],\"end_map\":[" << mx1 << ',' << my1
                      << "],\"speed\":" << speed << ",\"distance\":" << std::hypot(mx1 - mx0, my1 - my0)
                      << ",\"duration_s\":" << std::hypot(mx1 - mx0, my1 - my0) / speed << "}" << std::endl;
        } else if (command == "MOVESTATUS") {
            std::cout << std::setprecision(15) << "{\"event\":\"move_status\",\"active\":"
                      << (moveActive.load() ? "true" : "false") << ",\"sim\":" << sim.load()
                      << ",\"progress\":" << moveProgress.load() << ",\"cmd_world\":["
                      << moveCmdX.load() << ',' << moveCmdY.load() << "]}" << std::endl;
        } else if (command == "STOPMOVE") {
            moveStop = true;
            if (mover.joinable()) mover.join();
            std::cout << std::setprecision(15) << "{\"event\":\"move_stopped\",\"sim\":" << sim.load()
                      << ",\"progress\":" << moveProgress.load() << ",\"cmd_world\":["
                      << moveCmdX.load() << ',' << moveCmdY.load() << "]}" << std::endl;
        } else if (command == "POSE") {
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
        } else if (command == "PROBE" || command == "PROBE2") {
            calibrating = true;
            std::ostringstream sdf;
            sdf << std::setprecision(15) << "<sdf version='1.7'><model name='c020_contact_probe'><pose>"
                << (x + (command == "PROBE2" ? 3.6 : 0.0)) << ' ' << (y + (command == "PROBE2" ? 5.3 : 0.0)) << " 0.27 0 0 0</pose><link name='probe'><gravity>false</gravity>"
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
    moveStop = true;
    if (mover.joinable()) mover.join();
    health();
    ignition::msgs::Entity entity;
    entity.set_name("c020_opponent_cart"); entity.set_type(ignition::msgs::Entity::MODEL);
    request("/world/default/remove", entity);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    entity.set_name("c020_contact_system");
    request("/world/default/remove", entity);
    return 0;
}
