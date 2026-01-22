#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

class TTCVisualizer {
public:
    TTCVisualizer(ros::NodeHandle& nh) : nh_(nh) {
        // Load parameters
        loadParameters();
        
        // Initialize subscribers
        initializeSubscribers();
        
        // Initialize publishers
        initializePublishers();
        
        ROS_INFO("TTC Visualizer Node Started");
        ROS_INFO("  Warning threshold: %.1f seconds", ttc_warning_threshold_);
        ROS_INFO("  Critical threshold: %.1f seconds", ttc_critical_threshold_);
    }
    
    ~TTCVisualizer() {}

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;
    
    // Cached data
    double ttc_true_;
    double ttc_estimated_;
    double distance_true_;
    double distance_estimated_;
    double position_error_;
    std::string warning_level_;
    
    // Position data
    geometry_msgs::PoseStamped static_pose_;
    geometry_msgs::PoseStamped manual_true_pose_;
    geometry_msgs::PoseStamped manual_noisy_pose_;
    
    bool ttc_true_received_;
    bool ttc_estimated_received_;
    bool distance_true_received_;
    bool distance_estimated_received_;
    bool warning_received_;
    bool static_pose_received_;
    bool manual_true_pose_received_;
    bool manual_noisy_pose_received_;
    
    // ROS Subscribers
    ros::Subscriber ttc_true_sub_;
    ros::Subscriber ttc_estimated_sub_;
    ros::Subscriber distance_true_sub_;
    ros::Subscriber distance_estimated_sub_;
    ros::Subscriber position_error_sub_;
    ros::Subscriber warning_level_sub_;
    ros::Subscriber static_pose_sub_;
    ros::Subscriber manual_true_pose_sub_;
    ros::Subscriber manual_noisy_pose_sub_;
    
    // ROS Publishers
    ros::Publisher ttc_marker_pub_;
    ros::Publisher ttc_text_pub_;
    
    // TF Broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Timer for visualization updates
    ros::Timer timer_;
    
    // Parameters
    double ttc_warning_threshold_;
    double ttc_critical_threshold_;
    double update_rate_;
    
    // Load parameters from ROS parameter server
    void loadParameters() {
        nh_.param<double>("ttc_warning_threshold", ttc_warning_threshold_, 5.0);
        nh_.param<double>("ttc_critical_threshold", ttc_critical_threshold_, 2.0);
        nh_.param<double>("update_rate", update_rate_, 50.0);
        
        // Initialize state
        ttc_true_ = -1.0;
        ttc_estimated_ = -1.0;
        distance_true_ = 0.0;
        distance_estimated_ = 0.0;
        position_error_ = 0.0;
        warning_level_ = "SAFE";
        
        ttc_true_received_ = false;
        ttc_estimated_received_ = false;
        distance_true_received_ = false;
        distance_estimated_received_ = false;
        warning_received_ = false;
        static_pose_received_ = false;
        manual_true_pose_received_ = false;
        manual_noisy_pose_received_ = false;
    }
    
    // Initialize all subscribers
    void initializeSubscribers() {
        ttc_true_sub_ = nh_.subscribe("/ttc/true", 10, 
                                      &TTCVisualizer::ttcTrueCallback, this);
        ttc_estimated_sub_ = nh_.subscribe("/ttc/estimated", 10, 
                                           &TTCVisualizer::ttcEstimatedCallback, this);
        distance_true_sub_ = nh_.subscribe("/ttc/distance_true", 10, 
                                           &TTCVisualizer::distanceTrueCallback, this);
        distance_estimated_sub_ = nh_.subscribe("/ttc/distance_estimated", 10, 
                                                &TTCVisualizer::distanceEstimatedCallback, this);
        position_error_sub_ = nh_.subscribe("/ttc/position_error", 10, 
                                            &TTCVisualizer::positionErrorCallback, this);
        warning_level_sub_ = nh_.subscribe("/ttc/warning_level", 10, 
                                           &TTCVisualizer::warningLevelCallback, this);
        static_pose_sub_ = nh_.subscribe("/vehicle/static/pose", 10, 
                                         &TTCVisualizer::staticPoseCallback, this);
        manual_true_pose_sub_ = nh_.subscribe("/vehicle/manual/pose_true", 10, 
                                              &TTCVisualizer::manualTruePoseCallback, this);
        manual_noisy_pose_sub_ = nh_.subscribe("/vehicle/manual/pose_estimated", 10, 
                                               &TTCVisualizer::manualNoisyPoseCallback, this);
    }
    
    // Initialize all publishers
    void initializePublishers() {
        ttc_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ttc/visualization", 10);
        ttc_text_pub_ = nh_.advertise<visualization_msgs::Marker>("/ttc/text", 10);
        
        // Create timer
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), 
                                &TTCVisualizer::timerCallback, this);
    }
    
    // Callbacks
    void ttcTrueCallback(const std_msgs::Float64::ConstPtr& msg) {
        ttc_true_ = msg->data;
        ttc_true_received_ = true;
    }
    
    void ttcEstimatedCallback(const std_msgs::Float64::ConstPtr& msg) {
        ttc_estimated_ = msg->data;
        ttc_estimated_received_ = true;
    }
    
    void distanceTrueCallback(const std_msgs::Float64::ConstPtr& msg) {
        distance_true_ = msg->data;
        distance_true_received_ = true;
    }
    
    void distanceEstimatedCallback(const std_msgs::Float64::ConstPtr& msg) {
        distance_estimated_ = msg->data;
        distance_estimated_received_ = true;
    }
    
    void positionErrorCallback(const std_msgs::Float64::ConstPtr& msg) {
        position_error_ = msg->data;
    }
    
    void warningLevelCallback(const std_msgs::String::ConstPtr& msg) {
        warning_level_ = msg->data;
        warning_received_ = true;
    }
    
    void staticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        static_pose_ = *msg;
        static_pose_received_ = true;
    }
    
    void manualTruePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        manual_true_pose_ = *msg;
        manual_true_pose_received_ = true;
    }
    
    void manualNoisyPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        manual_noisy_pose_ = *msg;
        manual_noisy_pose_received_ = true;
    }
    
    // Get color based on warning level
    void getColorFromWarningLevel(const std::string& level, 
                                   float& r, float& g, float& b, float& a) const {
        if (level == "COLLISION") {
            r = 1.0; g = 0.0; b = 1.0; a = 1.0;  // Magenta (Collision!)
        } else if (level == "CRITICAL") {
            r = 1.0; g = 0.0; b = 0.0; a = 1.0;  // Red
        } else if (level == "WARNING") {
            r = 1.0; g = 1.0; b = 0.0; a = 0.8;  // Yellow
        } else if (level == "SAFE") {
            r = 0.0; g = 1.0; b = 0.0; a = 0.6;  // Green
        } else {
            r = 0.5; g = 0.5; b = 0.5; a = 0.5;  // Gray (N/A)
        }
    }
    
    void publishVisualization() {
        if (!static_pose_received_ || !manual_true_pose_received_ || !manual_noisy_pose_received_) {
            return;
        }
        
        // Create marker array
        visualization_msgs::MarkerArray marker_array;
        
        // ===== Line marker (estimated - based on noisy position) =====
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "ttc_line_estimated";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point p1, p2;
        p1.x = manual_noisy_pose_.pose.position.x;
        p1.y = manual_noisy_pose_.pose.position.y;
        p1.z = manual_noisy_pose_.pose.position.z + 1.0;
        p2.x = static_pose_.pose.position.x;
        p2.y = static_pose_.pose.position.y;
        p2.z = static_pose_.pose.position.z + 1.0;
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.scale.x = 0.1;
        
        // Color based on warning level (with communication delay applied)
        float r, g, b, a;
        getColorFromWarningLevel(warning_level_, r, g, b, a);
        line_marker.color.r = r;
        line_marker.color.g = g;
        line_marker.color.b = b;
        line_marker.color.a = a;
        
        line_marker.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(line_marker);
        
        // ===== Line marker (true - based on true position) =====
        visualization_msgs::Marker line_marker_true;
        line_marker_true.header.frame_id = "map";
        line_marker_true.header.stamp = ros::Time::now();
        line_marker_true.ns = "ttc_line_true";
        line_marker_true.id = 1;
        line_marker_true.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker_true.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point p1_true, p2_true;
        p1_true.x = manual_true_pose_.pose.position.x;
        p1_true.y = manual_true_pose_.pose.position.y;
        p1_true.z = manual_true_pose_.pose.position.z + 1.2;
        p2_true.x = static_pose_.pose.position.x;
        p2_true.y = static_pose_.pose.position.y;
        p2_true.z = static_pose_.pose.position.z + 1.2;
        
        line_marker_true.points.push_back(p1_true);
        line_marker_true.points.push_back(p2_true);
        line_marker_true.scale.x = 0.08;
        line_marker_true.color.r = 0.0;
        line_marker_true.color.g = 0.0;
        line_marker_true.color.b = 1.0;
        line_marker_true.color.a = 0.7;
        line_marker_true.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(line_marker_true);
        
        ttc_marker_pub_.publish(marker_array);
        
        // ===== Text marker showing TTC and distance information =====
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "ttc_text";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = (manual_noisy_pose_.pose.position.x + static_pose_.pose.position.x) / 2.0;
        text_marker.pose.position.y = (manual_noisy_pose_.pose.position.y + static_pose_.pose.position.y) / 2.0;
        text_marker.pose.position.z = std::max(manual_noisy_pose_.pose.position.z, static_pose_.pose.position.z) + 3.0;
        
        std::stringstream ss;
        if (ttc_true_ >= 0 && ttc_estimated_ >= 0) {
            ss << "TTC (True): " << std::fixed << std::setprecision(2) << ttc_true_ << " s\n";
            ss << "TTC (Est):  " << std::fixed << std::setprecision(2) << ttc_estimated_ << " s\n";
            ss << "Error: " << std::fixed << std::setprecision(2) << std::abs(ttc_true_ - ttc_estimated_) << " s\n";
            ss << "Dist (True): " << std::fixed << std::setprecision(1) << distance_true_ << " m\n";
            ss << "Dist (Est):  " << std::fixed << std::setprecision(1) << distance_estimated_ << " m\n";
            ss << "Warning: " << warning_level_;
        } else {
            ss << "TTC: N/A (Not Approaching)";
        }
        
        text_marker.text = ss.str();
        text_marker.scale.z = 0.8;
        text_marker.color.r = r;
        text_marker.color.g = g;
        text_marker.color.b = b;
        text_marker.color.a = 1.0;  // Full opacity for text
        text_marker.lifetime = ros::Duration(0.2);
        
        ttc_text_pub_.publish(text_marker);
        
        // Publish TF frame for TTC visualization center point
        publishTTCCenterTF();
    }
    
    // Publish TF transform for TTC center point (midpoint between vehicles)
    void publishTTCCenterTF() {
        if (!static_pose_received_ || !manual_noisy_pose_received_) {
            return;
        }
        
        geometry_msgs::TransformStamped ttc_center_tf;
        ttc_center_tf.header.stamp = ros::Time::now();
        ttc_center_tf.header.frame_id = "map";
        ttc_center_tf.child_frame_id = "ttc_center";
        
        // Position at midpoint between vehicles
        ttc_center_tf.transform.translation.x = (manual_noisy_pose_.pose.position.x + static_pose_.pose.position.x) / 2.0;
        ttc_center_tf.transform.translation.y = (manual_noisy_pose_.pose.position.y + static_pose_.pose.position.y) / 2.0;
        ttc_center_tf.transform.translation.z = std::max(manual_noisy_pose_.pose.position.z, static_pose_.pose.position.z) + 2.0;
        
        // Identity rotation
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        ttc_center_tf.transform.rotation.x = q.x();
        ttc_center_tf.transform.rotation.y = q.y();
        ttc_center_tf.transform.rotation.z = q.z();
        ttc_center_tf.transform.rotation.w = q.w();
        
        tf_broadcaster_.sendTransform(ttc_center_tf);
    }
    
    void timerCallback(const ros::TimerEvent&) {
        publishVisualization();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ttc_visualizer");
    ros::NodeHandle nh("~");
    
    TTCVisualizer ttc_visualizer(nh);
    
    ros::spin();
    
    return 0;
}