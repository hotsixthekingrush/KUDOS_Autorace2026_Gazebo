#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/string.hpp>

namespace kudos_arena {
class TrafficLightPlugin : public gazebo::ModelPlugin {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    link_ = model_->GetLink("signal_link");
    if (!link_) { gzerr << "Traffic light signal_link missing\n"; return; }
    ros_ = gazebo_ros::Node::Get(sdf);
    transport_.reset(new gazebo::transport::Node());
    transport_->Init(model_->GetWorld()->Name());
    visual_pub_ = transport_->Advertise<gazebo::msgs::Visual>("~/visual");
    state_pub_ = ros_->create_publisher<std_msgs::msg::String>(
      "/traffic_light/state", rclcpp::QoS(1).transient_local());
    command_sub_ = ros_->create_subscription<std_msgs::msg::String>(
      "/traffic_light/command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        int state = -1;
        for (int i = 0; i < 3; ++i) if (msg->data == names_[i]) state = i;
        if (state < 0) { RCLCPP_WARN(ros_->get_logger(), "Use RED, YELLOW or GREEN"); return; }
        requested_.store(state);
        sequence_.fetch_add(1);
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TrafficLightPlugin::Update, this));
    RCLCPP_INFO(ros_->get_logger(), "Signal ready: RED; waiting for separate controller");
  }
 private:
  void Update() {
    const double now = model_->GetWorld()->SimTime().Double();
    if (now < last_time_) {
      active_ = 0; last_command_ = -100.; last_publish_ = -1.;
      seen_ = sequence_.load();
    }
    last_time_ = now;
    const auto sequence = sequence_.load();
    if (sequence != seen_) {
      active_ = requested_.load(); last_command_ = now; seen_ = sequence;
    }
    // Controller heartbeats cease on exit: return to red after 1 s of sim time.
    if (now - last_command_ > 1.0) active_ = 0;
    if (now - last_publish_ < 0.1) return;
    last_publish_ = now;
    for (int i = 0; i < 3; ++i) {
      const std::string visual_name = "lamp_" + names_[i];
      gazebo::msgs::Visual visual;
      for (const auto &entry : link_->Visuals()) {
        const auto &candidate = entry.second;
        const auto &name = candidate.name();
        const std::string suffix = "::" + visual_name;
        if (name == visual_name || (name.size() >= suffix.size() &&
            name.compare(name.size()-suffix.size(), suffix.size(), suffix) == 0)) {
          visual = candidate;
          break;
        }
      }
      // Reuse server-assigned visual identity for GUI AND sensor render scenes.
      if (visual.name().empty()) continue;
      auto *material = visual.mutable_material();
      const auto &rgb = colors_[i];
      const double gain = (i == active_) ? 1.0 : 0.06;
      ignition::math::Color color(rgb[0]*gain, rgb[1]*gain, rgb[2]*gain, 1);
      gazebo::msgs::Set(material->mutable_ambient(), color);
      gazebo::msgs::Set(material->mutable_diffuse(), color);
      gazebo::msgs::Set(material->mutable_emissive(),
        i == active_ ? color : ignition::math::Color(0,0,0,1));
      visual_pub_->Publish(visual);
    }
    std_msgs::msg::String msg; msg.data = names_[active_]; state_pub_->publish(msg);
  }
  const std::array<std::string,3> names_{{"RED","YELLOW","GREEN"}};
  const std::array<std::array<double,3>,3> colors_{{{{1.,0.01,0.01}},{{1.,0.65,0.01}},{{0.01,1.,0.01}}}};
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr visual_pub_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr ros_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  std::atomic<int> requested_{0};
  std::atomic<unsigned long> sequence_{0};
  unsigned long seen_{0};
  int active_{0};
  double last_time_{0}, last_command_{-100}, last_publish_{-1};
};
GZ_REGISTER_MODEL_PLUGIN(TrafficLightPlugin)
}
