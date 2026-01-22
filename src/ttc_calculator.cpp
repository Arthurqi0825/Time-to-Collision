#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>

// Custom message for TTC data
struct TTCData {
    double ttc_true;
    double ttc_estimated;
    double distance_true;
    double distance_estimated;
    double position_error;
    ros::Time timestamp;
    std::string warning_level;  // "SAFE", "WARNING", "CRITICAL", "COLLISION"
};

// Vehicle state structure
struct VehicleState {
    double x, y, z;
    double vx, vy;
    double bbox_length, bbox_width, bbox_height;
    ros::Time timestamp;
    bool pose_received;
    bool velocity_received;
    bool bbox_received;
    
    VehicleState() : x(0), y(0), z(0), vx(0), vy(0), 
                     bbox_length(0), bbox_width(0), bbox_height(0),
                     pose_received(false), velocity_received(false), bbox_received(false) {}
};

// Point structure for 2D calculations
struct Point2D {
    double x, y;
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

class TTCCalculator {
public:
    TTCCalculator(ros::NodeHandle& nh) : nh_(nh) {
        // Initialize random seed
        generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        
        // Load parameters
        loadParameters();
        
        // Initialize subscribers
        initializeSubscribers();
        
        // Initialize publishers
        initializePublishers();
        
        // Create timer for periodic updates
        double update_rate;
        nh_.param<double>("update_rate", update_rate, 500.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), 
                                 &TTCCalculator::timerCallback, this);
        
        ROS_INFO("TTC Calculator Node Started");
        ROS_INFO("  Noise std deviation: %.3f meters", noise_std_);
        ROS_INFO("  Communication delay: %.3f seconds", comm_delay_);
        ROS_INFO("  Warning threshold: %.1f seconds", ttc_warning_threshold_);
        ROS_INFO("  Critical threshold: %.1f seconds", ttc_critical_threshold_);
    }
    
    ~TTCCalculator() {}

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;
    
    // Vehicle states
    VehicleState static_vehicle_;
    VehicleState manual_vehicle_true_;
    VehicleState manual_vehicle_noisy_;
    
    // ROS Subscribers
    ros::Subscriber static_pose_sub_;
    ros::Subscriber static_vel_sub_;
    ros::Subscriber static_bbox_sub_;
    ros::Subscriber manual_pose_sub_;
    ros::Subscriber manual_vel_sub_;
    ros::Subscriber manual_bbox_sub_;
    
    // ROS Publishers
    ros::Publisher ttc_true_pub_;
    ros::Publisher ttc_estimated_pub_;
    ros::Publisher distance_true_pub_;
    ros::Publisher distance_estimated_pub_;
    ros::Publisher position_error_pub_;
    ros::Publisher warning_level_pub_;
    ros::Publisher manual_true_pose_pub_;
    ros::Publisher manual_noisy_pose_pub_;
    
    // TF Broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Timer
    ros::Timer timer_;
    
    // Parameters
    double noise_std_;
    double comm_delay_;  // Communication delay in seconds
    double ttc_warning_threshold_;
    double ttc_critical_threshold_;
    
    // Random number generator
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
    
    // Warning state with delay
    struct DelayedWarning {
        std::string level;
        ros::Time trigger_time;
        bool active;
        
        DelayedWarning() : level("SAFE"), active(false) {}
    };
    
    DelayedWarning current_warning_;
    
    // Load parameters from ROS parameter server
    void loadParameters() {
        nh_.param<double>("noise_std", noise_std_, 0.5);
        nh_.param<double>("comm_delay", comm_delay_, 0.1);  // Default 100ms delay
        nh_.param<double>("ttc_warning_threshold", ttc_warning_threshold_, 5.0);
        nh_.param<double>("ttc_critical_threshold", ttc_critical_threshold_, 1.6);
        
        // Update noise distribution
        distribution_ = std::normal_distribution<double>(0.0, noise_std_);
    }
    
    // Initialize all subscribers
    void initializeSubscribers() {
        static_pose_sub_ = nh_.subscribe("/vehicle/static/pose", 10, 
                                         &TTCCalculator::staticPoseCallback, this);
        static_vel_sub_ = nh_.subscribe("/vehicle/static/velocity", 10, 
                                        &TTCCalculator::staticVelocityCallback, this);
        static_bbox_sub_ = nh_.subscribe("/vehicle/static/bounding_box", 10, 
                                         &TTCCalculator::staticBBoxCallback, this);
        manual_pose_sub_ = nh_.subscribe("/vehicle/manual/pose", 10, 
                                         &TTCCalculator::manualPoseCallback, this);
        manual_vel_sub_ = nh_.subscribe("/vehicle/manual/velocity", 10, 
                                        &TTCCalculator::manualVelocityCallback, this);
        manual_bbox_sub_ = nh_.subscribe("/vehicle/manual/bounding_box", 10, 
                                         &TTCCalculator::manualBBoxCallback, this);
    }
    
    // Initialize all publishers
    void initializePublishers() {
        ttc_true_pub_ = nh_.advertise<std_msgs::Float64>("/ttc/true", 10);
        ttc_estimated_pub_ = nh_.advertise<std_msgs::Float64>("/ttc/estimated", 10);
        distance_true_pub_ = nh_.advertise<std_msgs::Float64>("/ttc/distance_true", 10);
        distance_estimated_pub_ = nh_.advertise<std_msgs::Float64>("/ttc/distance_estimated", 10);
        position_error_pub_ = nh_.advertise<std_msgs::Float64>("/ttc/position_error", 10);
        warning_level_pub_ = nh_.advertise<std_msgs::String>("/ttc/warning_level", 10);
        manual_true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vehicle/manual/pose_true", 10);
        manual_noisy_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vehicle/manual/pose_estimated", 10);
    }
    
    // Callbacks
    void staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        static_vehicle_.x = msg->pose.position.x;
        static_vehicle_.y = msg->pose.position.y;
        static_vehicle_.z = msg->pose.position.z;
        static_vehicle_.timestamp = msg->header.stamp;
        static_vehicle_.pose_received = true;
    }
    
    void staticVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        static_vehicle_.vx = msg->twist.linear.x;
        static_vehicle_.vy = msg->twist.linear.y;
        static_vehicle_.velocity_received = true;
    }
    
    void staticBBoxCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        static_vehicle_.bbox_length = msg->scale.x;
        static_vehicle_.bbox_width = msg->scale.y;
        static_vehicle_.bbox_height = msg->scale.z;
        static_vehicle_.bbox_received = true;
    }
    
    void manualPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Store true position
        manual_vehicle_true_.x = msg->pose.position.x;
        manual_vehicle_true_.y = msg->pose.position.y;
        manual_vehicle_true_.z = msg->pose.position.z;
        manual_vehicle_true_.timestamp = msg->header.stamp;
        manual_vehicle_true_.pose_received = true;
        
        // Generate noisy position with Gaussian noise
        manual_vehicle_noisy_.x = manual_vehicle_true_.x + distribution_(generator_);
        manual_vehicle_noisy_.y = manual_vehicle_true_.y + distribution_(generator_);
        manual_vehicle_noisy_.z = manual_vehicle_true_.z;
        manual_vehicle_noisy_.timestamp = msg->header.stamp;
        manual_vehicle_noisy_.pose_received = true;
        
        // Publish both true and noisy poses
        geometry_msgs::PoseStamped true_pose_msg = *msg;
        manual_true_pose_pub_.publish(true_pose_msg);
        
        geometry_msgs::PoseStamped noisy_pose_msg = *msg;
        noisy_pose_msg.pose.position.x = manual_vehicle_noisy_.x;
        noisy_pose_msg.pose.position.y = manual_vehicle_noisy_.y;
        manual_noisy_pose_pub_.publish(noisy_pose_msg);
    }
    
    void manualVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        manual_vehicle_true_.vx = msg->twist.linear.x;
        manual_vehicle_true_.vy = msg->twist.linear.y;
        manual_vehicle_true_.velocity_received = true;
        
        manual_vehicle_noisy_.vx = msg->twist.linear.x;
        manual_vehicle_noisy_.vy = msg->twist.linear.y;
        manual_vehicle_noisy_.velocity_received = true;
    }
    
    void manualBBoxCallback(const visualization_msgs::Marker::ConstPtr& msg) {
        manual_vehicle_true_.bbox_length = msg->scale.x;
        manual_vehicle_true_.bbox_width = msg->scale.y;
        manual_vehicle_true_.bbox_height = msg->scale.z;
        manual_vehicle_true_.bbox_received = true;
        
        manual_vehicle_noisy_.bbox_length = msg->scale.x;
        manual_vehicle_noisy_.bbox_width = msg->scale.y;
        manual_vehicle_noisy_.bbox_height = msg->scale.z;
        manual_vehicle_noisy_.bbox_received = true;
    }
    
    // Helper: Calculate 2D distance
    double calculateDistance2D(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Helper: Check if manual vehicle is approaching static vehicle
    bool isApproaching(const VehicleState& manual_vehicle) const {
        double rel_vx = manual_vehicle.vx - static_vehicle_.vx;
        double rel_vy = manual_vehicle.vy - static_vehicle_.vy;
        double dx = static_vehicle_.x - manual_vehicle.x;
        double dy = static_vehicle_.y - manual_vehicle.y;
        
        double dot_product = rel_vx * dx + rel_vy * dy;
        return dot_product > 0;
    }
    
    // Helper: Calculate closest distance between bounding boxes
    double calculateClosestDistance(const VehicleState& v1, const VehicleState& v2) const {
        if (!v1.bbox_received || !v2.bbox_received) {
            return calculateDistance2D(v1.x, v1.y, v2.x, v2.y);
        }
        
        double center_distance = calculateDistance2D(v1.x, v1.y, v2.x, v2.y);
        double bbox_sum = (v1.bbox_length + v2.bbox_length) / 2.0;
        
        return std::max(0.0, center_distance - bbox_sum);
    }
    
    // Calculate TTC
    double calculateTTC(const VehicleState& manual_vehicle) const {
        if (!static_vehicle_.pose_received || !static_vehicle_.velocity_received ||
            !manual_vehicle.pose_received || !manual_vehicle.velocity_received) {
            return -1.0;
        }
        
        double rel_vx = manual_vehicle.vx - static_vehicle_.vx;
        double rel_vy = manual_vehicle.vy - static_vehicle_.vy;
        double relative_speed = std::sqrt(rel_vx * rel_vx + rel_vy * rel_vy);
        
        if (!isApproaching(manual_vehicle)) {
            return -1.0;
        }
        
        if (relative_speed < 0.1) {
            return -1.0;
        }
        
        double dx = manual_vehicle.x - static_vehicle_.x;
        double dy = manual_vehicle.y - static_vehicle_.y;
        double pos_magnitude = std::sqrt(dx * dx + dy * dy);
        
        if (pos_magnitude < 0.001) {
            return 0.0;
        }
        
        double pos_norm_x = dx / pos_magnitude;
        double pos_norm_y = dy / pos_magnitude;
        double closing_speed = -(rel_vx * pos_norm_x + rel_vy * pos_norm_y);
        
        if (closing_speed <= 0) {
            return -1.0;
        }
        
        double closest_distance = calculateClosestDistance(static_vehicle_, manual_vehicle);
        
        if (closest_distance <= 0) {
            return 0.0;
        }
        
        return closest_distance / closing_speed;
    }
    
    // Determine warning level based on TTC
    std::string determineWarningLevel(double ttc) const {
        if (ttc < 0) {
            return "SAFE";
        } else if (ttc < 0.05) {  // TTC is essentially 0 (collision or imminent collision)
            return "COLLISION";
        } else if (ttc <= ttc_critical_threshold_) {
            return "CRITICAL";
        } else if (ttc <= ttc_warning_threshold_) {
            return "WARNING";
        } else {
            return "SAFE";
        }
    }
    
    // Process and publish warning with communication delay
    void processWarningWithDelay(const std::string& current_level, const ros::Time& current_time) {
        // Check if we need to trigger a new warning
        if (current_level != "SAFE" && current_level != current_warning_.level) {
            current_warning_.level = current_level;
            current_warning_.trigger_time = current_time;
            current_warning_.active = true;
        } else if (current_level == "SAFE") {
            current_warning_.level = "SAFE";
            current_warning_.active = false;
        }
        
        // Publish warning if delay has passed
        std_msgs::String warning_msg;
        if (current_warning_.active) {
            ros::Duration elapsed = current_time - current_warning_.trigger_time;
            if (elapsed.toSec() >= comm_delay_) {
                warning_msg.data = current_warning_.level;
            } else {
                // Still in delay period, publish previous state
                warning_msg.data = "SAFE";
            }
        } else {
            warning_msg.data = current_warning_.level;
        }
        
        warning_level_pub_.publish(warning_msg);
    }
    
    // Publish TF transforms for vehicles
    void publishTF(const ros::Time& timestamp) {
        // Only publish if we have received pose data
        if (!static_vehicle_.pose_received || !manual_vehicle_true_.pose_received) {
            return;
        }
        
        // Static vehicle TF
        geometry_msgs::TransformStamped static_tf;
        static_tf.header.stamp = timestamp;
        static_tf.header.frame_id = "map";
        static_tf.child_frame_id = "static_vehicle";
        
        static_tf.transform.translation.x = static_vehicle_.x;
        static_tf.transform.translation.y = static_vehicle_.y;
        static_tf.transform.translation.z = static_vehicle_.z;
        
        // Assuming yaw-only rotation for simplicity (CARLA typically uses yaw)
        tf2::Quaternion q_static;
        q_static.setRPY(0, 0, 0);  // Can be enhanced with actual orientation
        static_tf.transform.rotation.x = q_static.x();
        static_tf.transform.rotation.y = q_static.y();
        static_tf.transform.rotation.z = q_static.z();
        static_tf.transform.rotation.w = q_static.w();
        
        tf_broadcaster_.sendTransform(static_tf);
        
        // Manual vehicle TF (true position)
        geometry_msgs::TransformStamped manual_tf;
        manual_tf.header.stamp = timestamp;
        manual_tf.header.frame_id = "map";
        manual_tf.child_frame_id = "manual_vehicle";
        
        manual_tf.transform.translation.x = manual_vehicle_true_.x;
        manual_tf.transform.translation.y = manual_vehicle_true_.y;
        manual_tf.transform.translation.z = manual_vehicle_true_.z;
        
        tf2::Quaternion q_manual;
        q_manual.setRPY(0, 0, 0);  // Can be enhanced with actual orientation
        manual_tf.transform.rotation.x = q_manual.x();
        manual_tf.transform.rotation.y = q_manual.y();
        manual_tf.transform.rotation.z = q_manual.z();
        manual_tf.transform.rotation.w = q_manual.w();
        
        tf_broadcaster_.sendTransform(manual_tf);
    }
    
    void timerCallback(const ros::TimerEvent&) {
        ros::Time current_time = ros::Time::now();
        
        // Calculate TTC values
        double ttc_true = calculateTTC(manual_vehicle_true_);
        double ttc_estimated = calculateTTC(manual_vehicle_noisy_);
        
        // Calculate distances
        double distance_true = calculateClosestDistance(static_vehicle_, manual_vehicle_true_);
        double distance_estimated = calculateClosestDistance(static_vehicle_, manual_vehicle_noisy_);
        
        // Calculate position error
        double position_error = calculateDistance2D(manual_vehicle_true_.x, manual_vehicle_true_.y,
                                                     manual_vehicle_noisy_.x, manual_vehicle_noisy_.y);
        
        // Publish TTC values
        std_msgs::Float64 ttc_true_msg, ttc_estimated_msg;
        ttc_true_msg.data = ttc_true;
        ttc_estimated_msg.data = ttc_estimated;
        ttc_true_pub_.publish(ttc_true_msg);
        ttc_estimated_pub_.publish(ttc_estimated_msg);
        
        // Publish distances
        std_msgs::Float64 dist_true_msg, dist_est_msg;
        dist_true_msg.data = distance_true;
        dist_est_msg.data = distance_estimated;
        distance_true_pub_.publish(dist_true_msg);
        distance_estimated_pub_.publish(dist_est_msg);
        
        // Publish position error
        std_msgs::Float64 pos_error_msg;
        pos_error_msg.data = position_error;
        position_error_pub_.publish(pos_error_msg);
        
        // Determine and publish warning level with delay
        std::string warning_level = determineWarningLevel(ttc_estimated);
        processWarningWithDelay(warning_level, current_time);
        
        // Broadcast TF frames for vehicles
        publishTF(current_time);
        
        // Console output
        if (ttc_true >= 0 || ttc_estimated >= 0) {
            if (ttc_true >= 0 && ttc_estimated >= 0) {
                double ttc_error = std::abs(ttc_true - ttc_estimated);
                
                // Use ROS_WARN for COLLISION events
                if (warning_level == "COLLISION") {
                    ROS_WARN("COLLISION DETECTED! TTC True: %.2f s | TTC Est: %.2f s | Distance: %.2f m", 
                             ttc_true, ttc_estimated, distance_estimated);
                } else {
                    ROS_INFO_THROTTLE(1.0, "TTC True: %.2f s | TTC Est: %.2f s | Error: %.2f s | Pos Error: %.2f m | Warning: %s", 
                                     ttc_true, ttc_estimated, ttc_error, position_error, warning_level.c_str());
                }
            } else {
                ROS_INFO_THROTTLE(2.0, "TTC: N/A (Not Approaching)");
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ttc_calculator");
    ros::NodeHandle nh("~");
    
    TTCCalculator ttc_calculator(nh);
    
    ros::spin();
    
    return 0;
}