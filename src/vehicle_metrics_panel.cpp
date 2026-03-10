#include "rviz_custom_panel/vehicle_metrics_panel.hpp"
#include "rviz_common/display_context.hpp"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPainter>
#include <QTransform>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace rviz_custom_panel {

// ==========================================
// VEHICLE METRICS PANEL (PANEL 1)
// ==========================================

VehicleMetricsPanel::VehicleMetricsPanel(QWidget* parent) : rviz_common::Panel(parent) {
    setupUI();
    connect(this, &VehicleMetricsPanel::onTelemetryReceived, this, &VehicleMetricsPanel::updateUI);
    connect(this, &VehicleMetricsPanel::onStateReceived, this, &VehicleMetricsPanel::updateStateUI);
}

void VehicleMetricsPanel::onInitialize() {
    auto node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!node_ptr) return;
    ros_node_ = node_ptr->get_raw_node();

    telemetry_sub_ = ros_node_->create_subscription<gae_msgs::msg::GaeTelemetry>(
        "/vehicle/telemetry", 10, [this](const gae_msgs::msg::GaeTelemetry::SharedPtr msg) {
            current_speed_ = msg->motor_velocity;
            current_voltage_ = msg->bus_voltage;
            Q_EMIT onTelemetryReceived();
        });

    state_sub_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
        "/vehicle_state", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
            current_planner_id_ = msg->data;
            Q_EMIT onStateReceived();
        });

    control_sub_ = ros_node_->create_subscription<gae_msgs::msg::GaeControlCmd>(
        "/vehicle/control", 10, [this](const gae_msgs::msg::GaeControlCmd::SharedPtr msg) {
            mode_auto_ = msg->mode_auto;
            received_control_data_ = true;
            Q_EMIT onStateReceived();
        });
}

void VehicleMetricsPanel::updateUI() {
    speed_dial_->setValue(static_cast<int>(current_speed_));
    speed_label_->setText(QString::number(current_speed_, 'f', 2) + " m/s");

    float percentage = ((current_voltage_ - V_MIN) / (V_MAX - V_MIN)) * 100.0f;
    percentage = std::clamp(percentage, 0.0f, 100.0f);
    battery_bar_->setValue(static_cast<int>(percentage * 100.0f));
    battery_label_->setText(QString::number(current_voltage_, 'f', 1) + " V");
    battery_percent_label_->setText("%" + QString::number(percentage, 'f', 1));

    if (percentage < 20.0f) battery_bar_->setStyleSheet("QProgressBar::chunk { background-color: #e74c3c; }");
    else if (percentage < 50.0f) battery_bar_->setStyleSheet("QProgressBar::chunk { background-color: #f1c40f; }");
    else battery_bar_->setStyleSheet("QProgressBar::chunk { background-color: #27ae60; }");
}

void VehicleMetricsPanel::updateStateUI() {
    QString text;
    QString style = "font-size: 14pt; font-weight: bold; padding: 8px; border-radius: 6px; ";
    if (!received_control_data_) {
        text = "NO CONTROL DATA";
        style += "color: #7f8c8d; background-color: #f2f3f4;";
    } else if (mode_auto_ == 1) {
        text = "AUTO: " + plannerStateToString(current_planner_id_);
        style += "color: #ffffff; background-color: #27ae60; border: 2px solid #1e8449;";
    } else {
        text = "MANUAL CONTROL";
        style += "color: #ffffff; background-color: #e67e22; border: 2px solid #d35400;";
    }
    state_label_->setText(text);
    state_label_->setStyleSheet(style);
}

QString VehicleMetricsPanel::plannerStateToString(int s) {
    switch (s) {
        case 1: return "CENTERLINE"; case 2: return "LATTICE";
        case 3: return "ASTAR"; case 4: return "PARKING";
        default: return "ACTIVE";
    }
}

void VehicleMetricsPanel::setupUI() {
    auto* layout = new QVBoxLayout(this);
    auto* speed_group = new QGroupBox("Drivetrain");
    auto* s_layout = new QVBoxLayout();
    speed_dial_ = new QDial(); speed_dial_->setRange(0, 15); speed_dial_->setNotchesVisible(true);
    speed_label_ = new QLabel("0.00 m/s"); speed_label_->setAlignment(Qt::AlignCenter);
    s_layout->addWidget(speed_dial_); s_layout->addWidget(speed_label_);
    speed_group->setLayout(s_layout);

    auto* batt_group = new QGroupBox("Power System");
    auto* b_layout = new QVBoxLayout();
    battery_bar_ = new QProgressBar(); battery_bar_->setRange(0, 10000); battery_bar_->setTextVisible(false);
    auto* b_info_layout = new QHBoxLayout();
    battery_label_ = new QLabel("0.0 V"); battery_percent_label_ = new QLabel("%0.0");
    b_info_layout->addWidget(battery_label_); b_info_layout->addStretch(); b_info_layout->addWidget(battery_percent_label_);
    b_layout->addWidget(battery_bar_); b_layout->addLayout(b_info_layout);
    batt_group->setLayout(b_layout);

    auto* state_group = new QGroupBox("Status");
    auto* st_layout = new QVBoxLayout();
    state_label_ = new QLabel("WAITING..."); state_label_->setAlignment(Qt::AlignCenter);
    st_layout->addWidget(state_label_);
    state_group->setLayout(st_layout);

    layout->addWidget(speed_group); layout->addWidget(batt_group); layout->addWidget(state_group);
}

// ==========================================
// VEHICLE EXTRAS PANEL (PANEL 2)
// ==========================================

VehicleExtrasPanel::VehicleExtrasPanel(QWidget* parent) : rviz_common::Panel(parent) {
    setupUI();
    connect(this, &VehicleExtrasPanel::onDataReceived, this, &VehicleExtrasPanel::updateUI);
}

void VehicleExtrasPanel::onInitialize() {
    auto node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!node_ptr) return;
    ros_node_ = node_ptr->get_raw_node();

    nav_sat_1 = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/sensing/gps_front/fix", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            current_cov_0_navsat1 = msg->position_covariance[0];
            current_cov_1_navsat1 = msg->position_covariance[4];
            Q_EMIT onDataReceived();
        });

    nav_sat_2 = ros_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/sensing/gps_rear/fix", 10, [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
            current_cov_0_navsat2 = msg->position_covariance[0];
            current_cov_1_navsat2 = msg->position_covariance[4];
            Q_EMIT onDataReceived();
        });

    control_sub_ = ros_node_->create_subscription<gae_msgs::msg::GaeControlCmd>(
        "/vehicle/control", 10, [this](const gae_msgs::msg::GaeControlCmd::SharedPtr msg) {
            current_steering_ = msg->steering;
            Q_EMIT onDataReceived();
        });
}

void VehicleExtrasPanel::updateUI() {
    // Math: Standard Deviation (mm) = sqrt(Variance (m^2)) * 1000
    double std_x_1 = std::sqrt(std::max(0.0, current_cov_0_navsat1)) * 1000.0;
    double std_y_1 = std::sqrt(std::max(0.0, current_cov_1_navsat1)) * 1000.0;
    double std_x_2 = std::sqrt(std::max(0.0, current_cov_0_navsat2)) * 1000.0;
    double std_y_2 = std::sqrt(std::max(0.0, current_cov_1_navsat2)) * 1000.0;

    // Output formatted to 1 decimal place (e.g., "14.5")
    std_x_label_navsat1_->setText(QString::number(std_x_1, 'f', 1));
    std_y_label_navsat1_->setText(QString::number(std_y_1, 'f', 1));
    std_x_label_navsat2_->setText(QString::number(std_x_2, 'f', 1));
    std_y_label_navsat2_->setText(QString::number(std_y_2, 'f', 1));

    int text_angle = static_cast<int>(current_steering_);
    float visual_angle = (static_cast<float>(current_steering_) - 1800.0f) / 36.0f;
    visual_angle = std::clamp(visual_angle, -50.0f, 50.0f);

    steering_angle_label_->setText(QString::number(text_angle) + "°");
    drawSteeringWheel(visual_angle);
}

void VehicleExtrasPanel::drawSteeringWheel(float angle) {
    QPixmap pixmap(180, 180);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    
    painter.translate(90, 90);
    painter.rotate(-angle);
    
    painter.setPen(QPen(QColor(33, 37, 41), 14)); 
    painter.drawEllipse(-80, -80, 160, 160);
    
    painter.setPen(QPen(QColor(52, 58, 64), 10));
    painter.drawLine(-70, 0, 70, 0); 
    painter.drawLine(0, 0, 0, 70);   
    
    painter.setBrush(QColor(33, 37, 41));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(-15, -15, 30, 30);
    
    painter.end();
    
    steering_wheel_label_->setPixmap(pixmap);
}

void VehicleExtrasPanel::setupUI() {
    auto* layout = new QVBoxLayout(this);
    
    // --- STEERING GROUP ---
    auto* steer_group = new QGroupBox("Steering Visualization");
    auto* steer_layout = new QVBoxLayout();
    
    steering_wheel_label_ = new QLabel();
    steering_wheel_label_->setAlignment(Qt::AlignCenter);
    steering_wheel_label_->setFixedSize(180, 180); 
    
    steering_angle_label_ = new QLabel("1800°");
    steering_angle_label_->setAlignment(Qt::AlignCenter);
    steering_angle_label_->setStyleSheet("font-size: 13pt; font-weight: bold; color: #2c3e50;");

    steer_layout->addWidget(steering_wheel_label_);
    steer_layout->addWidget(steering_angle_label_);
    steer_group->setLayout(steer_layout);

    drawSteeringWheel(0.0f);

    // --- GNSS GROUP ---
    auto* cov_group = new QGroupBox("GNSS Std. Deviation");
    auto* v_layout = new QVBoxLayout();
    
    // FRONT GPS HEADER
    v_layout->addWidget(new QLabel("<b>Front GPS:</b>"));
    
    // FRONT GPS ROWS (Added immediately after header)
    auto* row1 = new QHBoxLayout();
    row1->addWidget(new QLabel("Std X (mm):"));
    std_x_label_navsat1_ = new QLabel("0.0");
    std_x_label_navsat1_->setStyleSheet("font-family: monospace; font-weight: bold; color: #16a085; font-size: 11pt;");
    row1->addStretch(); row1->addWidget(std_x_label_navsat1_);
    v_layout->addLayout(row1);
    
    auto* row2 = new QHBoxLayout();
    row2->addWidget(new QLabel("Std Y (mm):"));
    std_y_label_navsat1_ = new QLabel("0.0");
    std_y_label_navsat1_->setStyleSheet("font-family: monospace; font-weight: bold; color: #16a085; font-size: 11pt;");
    row2->addStretch(); row2->addWidget(std_y_label_navsat1_);
    v_layout->addLayout(row2);
    
    // Adds a 10-pixel vertical gap for clarity
    v_layout->addSpacing(10); 
    
    // REAR GPS HEADER
    v_layout->addWidget(new QLabel("<b>Rear GPS:</b>"));
    
    // REAR GPS ROWS (Added immediately after header)
    auto* row3 = new QHBoxLayout();
    row3->addWidget(new QLabel("Std X (mm):"));
    std_x_label_navsat2_ = new QLabel("0.0");
    std_x_label_navsat2_->setStyleSheet("font-family: monospace; font-weight: bold; color: #2980b9; font-size: 11pt;");
    row3->addStretch(); row3->addWidget(std_x_label_navsat2_);
    v_layout->addLayout(row3);
    
    auto* row4 = new QHBoxLayout();
    row4->addWidget(new QLabel("Std Y (mm):"));
    std_y_label_navsat2_ = new QLabel("0.0");
    std_y_label_navsat2_->setStyleSheet("font-family: monospace; font-weight: bold; color: #2980b9; font-size: 11pt;");
    row4->addStretch(); row4->addWidget(std_y_label_navsat2_);
    v_layout->addLayout(row4);

    cov_group->setLayout(v_layout);

    layout->addWidget(steer_group);
    layout->addWidget(cov_group);
    layout->addStretch();
}

// ==========================================
// DRIVERS HEALTH PANEL (PANEL 3)
// ==========================================

VehicleDriversPanel::VehicleDriversPanel(QWidget* parent) : rviz_common::Panel(parent) {
    main_layout_ = new QVBoxLayout(this);
    auto* group = new QGroupBox("Driver / Topic Health (Hz)");
    grid_layout_ = new QGridLayout();
    
    auto* name_header = new QLabel("<b>Topic / Sensor</b>");
    auto* status_header = new QLabel("<b>Status (Frequency)</b>");
    status_header->setAlignment(Qt::AlignCenter);
    grid_layout_->addWidget(name_header, 0, 0);
    grid_layout_->addWidget(status_header, 0, 1);

    group->setLayout(grid_layout_);
    main_layout_->addWidget(group);
    main_layout_->addStretch();

    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &VehicleDriversPanel::updateUI);
}

VehicleDriversPanel::~VehicleDriversPanel() {
    running_ = false;
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    if (isolated_context_ && isolated_context_->is_valid()) {
        isolated_context_->shutdown("Panel destroyed");
    }
}

void VehicleDriversPanel::createMonitor(const QString& name, const std::string& topic, const std::string& msg_type, float orange_th, float green_th) {
    auto monitor = std::make_shared<TopicMonitor>();
    monitor->name = name;
    monitor->orange_thresh = orange_th;
    monitor->green_thresh = green_th;
    
    int row = monitors_.size() + 1; 
    grid_layout_->addWidget(new QLabel(name), row, 0);
    
    monitor->status_label = new QLabel("WAITING...");
    monitor->status_label->setAlignment(Qt::AlignCenter);
    monitor->status_label->setStyleSheet("font-weight: bold; background-color: #95a5a6; color: white; border-radius: 4px; padding: 4px;");
    grid_layout_->addWidget(monitor->status_label, row, 1);

    rclcpp::QoS qos_profile = rclcpp::SensorDataQoS().keep_last(5);

    monitor->sub = monitor_node_->create_generic_subscription(
        topic, msg_type, qos_profile,
        [this, monitor](std::shared_ptr<rclcpp::SerializedMessage> /*msg*/) {
            std::lock_guard<std::mutex> lock(monitor->mutex);
            monitor->timestamps.push_back(this->monitor_node_->now());
        });

    monitors_.push_back(monitor);
}

void VehicleDriversPanel::onInitialize() {
    rclcpp::InitOptions init_options;
    isolated_context_ = std::make_shared<rclcpp::Context>();
    isolated_context_->init(0, nullptr, init_options);

    rclcpp::NodeOptions node_options;
    node_options.context(isolated_context_);
    
    std::string node_name = "rviz_health_monitor_" + std::to_string(rand() % 10000);
    monitor_node_ = std::make_shared<rclcpp::Node>(node_name, node_options);
    
    rclcpp::ExecutorOptions exec_options;
    exec_options.context = isolated_context_;
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(exec_options);
    executor_->add_node(monitor_node_);
    
    running_ = true;
    spin_thread_ = std::thread([this]() {
        while (running_ && rclcpp::ok(isolated_context_)) {
            executor_->spin_some(std::chrono::milliseconds(5)); 
        }
    });

    createMonitor("Ouster LiDAR", "/ouster/points", "sensor_msgs/msg/PointCloud2", 12.0, 18.0);
    createMonitor("Velodyne LiDAR", "/velodyne_points", "sensor_msgs/msg/PointCloud2", 8.0, 9.0);
    createMonitor("SBG 1 IMU", "/sbg_device1/imu/data", "sensor_msgs/msg/Imu", 160.0, 180.0);
    createMonitor("SBG 2 IMU", "/sbg_device2/imu/data", "sensor_msgs/msg/Imu", 160.0, 180.0);
    createMonitor("SBG 1 NavSat", "/sbg_device1/imu/nav_sat_fix", "sensor_msgs/msg/NavSatFix", 4.0, 4.5);
    createMonitor("SBG 2 NavSat", "/sbg_device2/imu/nav_sat_fix", "sensor_msgs/msg/NavSatFix", 4.0, 4.5);
    createMonitor("ZED Left Cam", "/zed2_left_camera/image_raw", "sensor_msgs/msg/Image", 14.0, 18.0);
    createMonitor("NTRIP 1 RTCM", "/sbg_device2/ntrip_client/rtcm", "std_msgs/msg/UInt8MultiArray", 1.8, 2.4);
    createMonitor("NTRIP 2 RTCM", "/sbg_device2/ntrip_client/rtcm2", "std_msgs/msg/UInt8MultiArray", 1.8, 2.4);

    update_timer_->start(500); 
}

void VehicleDriversPanel::updateUI() {
    rclcpp::Time now = monitor_node_->now();

    for (auto& m : monitors_) {
        float raw_hz = 0.0f;

        { 
            std::lock_guard<std::mutex> lock(m->mutex);

            while (!m->timestamps.empty()) {
                double age_in_seconds = (now - m->timestamps.front()).seconds();
                if (age_in_seconds > 3.0) {
                    m->timestamps.pop_front();
                } else {
                    break; 
                }
            }

            int count = m->timestamps.size();
            
            if (count > 0) {
                double window_size = (now - m->timestamps.front()).seconds();
                if (window_size > 0.2) { 
                    double effective_window = std::min(window_size, 3.0);
                    raw_hz = static_cast<float>(count) / effective_window;
                }
            }
        } 

        if (m->displayed_hz == 0.0f || raw_hz == 0.0f) {
            m->displayed_hz = raw_hz; 
        } else {
            m->displayed_hz = (raw_hz * 0.2f) + (m->displayed_hz * 0.8f);
        }

        QString text = QString::number(m->displayed_hz, 'f', 1) + " Hz";
        QString style = "font-weight: bold; padding: 4px; border-radius: 4px; color: white; ";

        if (m->displayed_hz >= m->green_thresh) {
            style += "background-color: #27ae60;"; // Green
        } else if (m->displayed_hz >= m->orange_thresh) {
            style += "background-color: #f39c12;"; // Orange
        } else {
            style += "background-color: #c0392b;"; // Red
            text = "FAIL (" + text + ")";
        }

        m->status_label->setText(text);
        m->status_label->setStyleSheet(style);
    }
}

} // namespace rviz_custom_panel

PLUGINLIB_EXPORT_CLASS(rviz_custom_panel::VehicleMetricsPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(rviz_custom_panel::VehicleExtrasPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(rviz_custom_panel::VehicleDriversPanel, rviz_common::Panel)