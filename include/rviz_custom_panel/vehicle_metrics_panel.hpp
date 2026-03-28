#ifndef RVIZ_CUSTOM_PANEL__VEHICLE_METRICS_PANEL_HPP_
#define RVIZ_CUSTOM_PANEL__VEHICLE_METRICS_PANEL_HPP_

#include <QLabel>
#include <QProgressBar>
#include <QDial>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPixmap>
#include <QTimer>
#include <deque>
#include <memory>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <cmath> 
#include <chrono> // Added for robust, Gazebo-immune time tracking

#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialized_message.hpp"
#include "gae_msgs/msg/gae_telemetry.hpp"
#include "gae_msgs/msg/gae_control_cmd.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace rviz_custom_panel {

// ==========================================
// --- PANEL 1: MAIN METRICS ---
// ==========================================
class VehicleMetricsPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit VehicleMetricsPanel(QWidget* parent = nullptr);
  void onInitialize() override;

Q_SIGNALS:
  void onTelemetryReceived();
  void onStateReceived();

private Q_SLOTS:
  void updateUI();
  void updateStateUI();

private:
  void setupUI();
  QString plannerStateToString(int state);

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<gae_msgs::msg::GaeTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
  rclcpp::Subscription<gae_msgs::msg::GaeControlCmd>::SharedPtr control_sub_;

  QProgressBar* battery_bar_;
  QLabel* battery_label_;
  QLabel* battery_percent_label_;
  QDial* speed_dial_;
  QLabel* speed_label_;
  QLabel* state_label_;

  float current_speed_ = 0.0f;
  float current_voltage_ = 0.0f;
  int current_planner_id_ = 0;
  int mode_auto_ = 0; 
  bool received_control_data_ = false;

  const float V_MIN = 47.0f;
  const float V_MAX = 57.0f;
};

// ==========================================
// --- PANEL 2: EXTRAS (Localization & Steering) ---
// ==========================================
class VehicleExtrasPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit VehicleExtrasPanel(QWidget* parent = nullptr);
  void onInitialize() override;

Q_SIGNALS:
  void onDataReceived();

private Q_SLOTS:
  void updateUI();

private:
  void setupUI();
  void drawSteeringWheel(float angle); 

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_1;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_2;
  rclcpp::Subscription<gae_msgs::msg::GaeControlCmd>::SharedPtr control_sub_;

  QLabel* std_x_label_navsat1_;
  QLabel* std_y_label_navsat1_;
  QLabel* std_x_label_navsat2_;
  QLabel* std_y_label_navsat2_;
  QLabel* steering_wheel_label_;
  QLabel* steering_angle_label_;

  double current_cov_0_navsat1 = 0.0;
  double current_cov_1_navsat1 = 0.0;
  double current_cov_0_navsat2 = 0.0;
  double current_cov_1_navsat2 = 0.0;   
  uint16_t current_steering_ = 1800;
};

// ==========================================
// --- PANEL 3: DRIVERS / MODULES HEALTH ---
// ==========================================
class VehicleDriversPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit VehicleDriversPanel(QWidget* parent = nullptr);
  ~VehicleDriversPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void updateUI();

private:
  struct TopicMonitor {
    QString name;
    float orange_thresh; 
    float green_thresh;  
    std::deque<std::chrono::steady_clock::time_point> timestamps; // Uses CPU time, not Sim time
    QLabel* status_label;
    rclcpp::GenericSubscription::SharedPtr sub; 
    float displayed_hz = 0.0f; 
    std::mutex mutex;          
  };

  std::vector<std::shared_ptr<TopicMonitor>> monitors_;
  QTimer* update_timer_;
  QVBoxLayout* main_layout_;
  QGridLayout* grid_layout_;

  // Removed isolated_context_ completely.
  rclcpp::Node::SharedPtr monitor_node_; 
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_; 
  
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  void createMonitor(const QString& name, const std::string& topic, const std::string& msg_type, float orange_th, float green_th);
};

} // namespace rviz_custom_panel

#endif // RVIZ_CUSTOM_PANEL__VEHICLE_METRICS_PANEL_HPP_