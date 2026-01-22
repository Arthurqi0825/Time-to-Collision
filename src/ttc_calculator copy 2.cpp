#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include <random>
#include <limits>
#include <algorithm>

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
        nh_.param<double>("update_rate", update_rate, 50.0);
        timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate), 
                                 &TTCCalculator::timerCallback, this);
        
        ROS_INFO("TTC Calculator Node Started (with Gaussian Noise)");
        ROS_INFO("  Noise std deviation: %.2f meters", noise_std_);
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
    ros::Publisher ttc_marker_pub_;
    ros::Publisher ttc_text_pub_;
    ros::Publisher manual_true_pose_pub_;
    ros::Publisher manual_noisy_pose_pub_;
    
    // Timer
    ros::Timer timer_;
    
    // Parameters
    double noise_std_;
    double ttc_warning_threshold_;
    double ttc_critical_threshold_;
    
    // Topic names
    std::string static_pose_topic_;
    std::string static_velocity_topic_;
    std::string static_bbox_topic_;
    std::string manual_pose_topic_;
    std::string manual_velocity_topic_;
    std::string manual_bbox_topic_;
    std::string ttc_true_topic_;
    std::string ttc_estimated_topic_;
    std::string ttc_visualization_topic_;
    std::string ttc_text_topic_;
    std::string manual_true_pose_topic_;
    std::string manual_noisy_pose_topic_;
    
    // Random number generator
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
    
    // Load parameters from ROS parameter server
    void loadParameters() {
        nh_.param<double>("noise_std", noise_std_, 0.5);
        nh_.param<double>("ttc_warning_threshold", ttc_warning_threshold_, 5.0);
        nh_.param<double>("ttc_critical_threshold", ttc_critical_threshold_, 2.0);
        
        // Topic names
        nh_.param<std::string>("static_pose_topic", static_pose_topic_, "/vehicle/static/pose");
        nh_.param<std::string>("static_velocity_topic", static_velocity_topic_, "/vehicle/static/velocity");
        nh_.param<std::string>("static_bbox_topic", static_bbox_topic_, "/vehicle/static/bounding_box");
        nh_.param<std::string>("manual_pose_topic", manual_pose_topic_, "/vehicle/manual/pose");
        nh_.param<std::string>("manual_velocity_topic", manual_velocity_topic_, "/vehicle/manual/velocity");
        nh_.param<std::string>("manual_bbox_topic", manual_bbox_topic_, "/vehicle/manual/bounding_box");
        nh_.param<std::string>("ttc_true_topic", ttc_true_topic_, "/ttc/true");
        nh_.param<std::string>("ttc_estimated_topic", ttc_estimated_topic_, "/ttc/estimated");
        nh_.param<std::string>("ttc_visualization_topic", ttc_visualization_topic_, "/ttc/visualization");
        nh_.param<std::string>("ttc_text_topic", ttc_text_topic_, "/ttc/text");
        nh_.param<std::string>("manual_true_pose_topic", manual_true_pose_topic_, "/vehicle/manual/pose_true");
        nh_.param<std::string>("manual_noisy_pose_topic", manual_noisy_pose_topic_, "/vehicle/manual/pose_estimated");
        
        // Update noise distribution
        distribution_ = std::normal_distribution<double>(0.0, noise_std_);
    }
    
    // Initialize all subscribers
    void initializeSubscribers() {
        static_pose_sub_ = nh_.subscribe(static_pose_topic_, 10, 
                                         &TTCCalculator::staticPoseCallback, this);
        static_vel_sub_ = nh_.subscribe(static_velocity_topic_, 10, 
                                        &TTCCalculator::staticVelocityCallback, this);
        static_bbox_sub_ = nh_.subscribe(static_bbox_topic_, 10, 
                                         &TTCCalculator::staticBBoxCallback, this);
        manual_pose_sub_ = nh_.subscribe(manual_pose_topic_, 10, 
                                         &TTCCalculator::manualPoseCallback, this);
        manual_vel_sub_ = nh_.subscribe(manual_velocity_topic_, 10, 
                                        &TTCCalculator::manualVelocityCallback, this);
        manual_bbox_sub_ = nh_.subscribe(manual_bbox_topic_, 10, 
                                         &TTCCalculator::manualBBoxCallback, this);
    }
    
    // Initialize all publishers
    void initializePublishers() {
        ttc_true_pub_ = nh_.advertise<std_msgs::Float64>(ttc_true_topic_, 10);
        ttc_estimated_pub_ = nh_.advertise<std_msgs::Float64>(ttc_estimated_topic_, 10);
        ttc_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ttc_visualization_topic_, 10);
        ttc_text_pub_ = nh_.advertise<visualization_msgs::Marker>(ttc_text_topic_, 10);
        manual_true_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(manual_true_pose_topic_, 10);
        manual_noisy_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(manual_noisy_pose_topic_, 10);
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
        
        // Add Gaussian noise for estimated position
        double noise_x = distribution_(generator_);
        double noise_y = distribution_(generator_);
        
        manual_vehicle_noisy_.x = msg->pose.position.x + noise_x;
        manual_vehicle_noisy_.y = msg->pose.position.y + noise_y;
        manual_vehicle_noisy_.z = msg->pose.position.z;
        manual_vehicle_noisy_.timestamp = msg->header.stamp;
        manual_vehicle_noisy_.pose_received = true;
        
        // Publish true position
        geometry_msgs::PoseStamped true_pose;
        true_pose.header.stamp = ros::Time::now();
        true_pose.header.frame_id = "map";
        true_pose.pose = msg->pose;
        manual_true_pose_pub_.publish(true_pose);
        
        // Publish noisy position
        geometry_msgs::PoseStamped noisy_pose;
        noisy_pose.header.stamp = ros::Time::now();
        noisy_pose.header.frame_id = "map";
        noisy_pose.pose.position.x = manual_vehicle_noisy_.x;
        noisy_pose.pose.position.y = manual_vehicle_noisy_.y;
        noisy_pose.pose.position.z = manual_vehicle_noisy_.z;
        noisy_pose.pose.orientation = msg->pose.orientation;
        manual_noisy_pose_pub_.publish(noisy_pose);
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
    
    // Helper functions
    double calculateDistance2D(double x1, double y1, double x2, double y2) const {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Get four corner points of a bounding box in 2D (simplified, assuming axis-aligned)
    std::vector<Point2D> getBBoxCorners(const VehicleState& vehicle) const {
        std::vector<Point2D> corners;
        double half_length = vehicle.bbox_length / 2.0;
        double half_width = vehicle.bbox_width / 2.0;
        
        // Four corners of the bounding box
        corners.push_back(Point2D(vehicle.x + half_length, vehicle.y + half_width));
        corners.push_back(Point2D(vehicle.x + half_length, vehicle.y - half_width));
        corners.push_back(Point2D(vehicle.x - half_length, vehicle.y + half_width));
        corners.push_back(Point2D(vehicle.x - half_length, vehicle.y - half_width));
        
        return corners;
    }
    
    // Calculate distance from a point to a line segment
    double pointToSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b) const {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        
        if (dx == 0 && dy == 0) {
            // Segment is a point
            return calculateDistance2D(p.x, p.y, a.x, a.y);
        }
        
        // Calculate parameter t for the projection of point p onto line ab
        double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);
        
        // Clamp t to [0, 1] to handle points outside the segment
        t = std::max(0.0, std::min(1.0, t));
        
        // Calculate the closest point on the segment
        double closest_x = a.x + t * dx;
        double closest_y = a.y + t * dy;
        
        return calculateDistance2D(p.x, p.y, closest_x, closest_y);
    }
    
    // Calculate minimum distance between two bounding boxes using closest points
    double calculateClosestDistance(const VehicleState& vehicle1, const VehicleState& vehicle2) const {
        std::vector<Point2D> corners1 = getBBoxCorners(vehicle1);
        std::vector<Point2D> corners2 = getBBoxCorners(vehicle2);
        
        double min_distance = std::numeric_limits<double>::max();
        
        // Check distance from each corner of vehicle1 to each edge of vehicle2
        for (const auto& corner : corners1) {
            for (size_t i = 0; i < corners2.size(); ++i) {
                size_t j = (i + 1) % corners2.size();
                double dist = pointToSegmentDistance(corner, corners2[i], corners2[j]);
                min_distance = std::min(min_distance, dist);
            }
        }
        
        // Check distance from each corner of vehicle2 to each edge of vehicle1
        for (const auto& corner : corners2) {
            for (size_t i = 0; i < corners1.size(); ++i) {
                size_t j = (i + 1) % corners1.size();
                double dist = pointToSegmentDistance(corner, corners1[i], corners1[j]);
                min_distance = std::min(min_distance, dist);
            }
        }
        
        // Also check center-to-center distance as a baseline
        double center_dist = calculateDistance2D(vehicle1.x, vehicle1.y, vehicle2.x, vehicle2.y);
        
        // If bounding boxes overlap, return 0
        if (min_distance < 1e-6) {
            return 0.0;
        }
        
        return min_distance;
    }
    
    bool isApproaching(const VehicleState& manual_vehicle) const {
        double dx = manual_vehicle.x - static_vehicle_.x;
        double dy = manual_vehicle.y - static_vehicle_.y;
        double dvx = manual_vehicle.vx - static_vehicle_.vx;
        double dvy = manual_vehicle.vy - static_vehicle_.vy;
        double dot_product = dx * dvx + dy * dvy;
        return dot_product < 0;
    }
    
    double calculateTTC(const VehicleState& manual_vehicle) {
        // Check if all data is received
        if (!static_vehicle_.pose_received || !static_vehicle_.velocity_received || 
            !static_vehicle_.bbox_received || !manual_vehicle.pose_received || 
            !manual_vehicle.velocity_received || !manual_vehicle.bbox_received) {
            return -1.0;
        }
        
        // Calculate relative velocity
        double rel_vx = manual_vehicle.vx - static_vehicle_.vx;
        double rel_vy = manual_vehicle.vy - static_vehicle_.vy;
        double relative_speed = std::sqrt(rel_vx * rel_vx + rel_vy * rel_vy);
        
        // Check if approaching
        if (!isApproaching(manual_vehicle)) {
            return -1.0;
        }
        
        if (relative_speed < 0.1) {
            return -1.0;
        }
        
        // Calculate closing speed along the line connecting the two vehicles
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
        
        // Calculate closest distance between two bounding boxes
        double closest_distance = calculateClosestDistance(static_vehicle_, manual_vehicle);
        
        if (closest_distance <= 0) {
            return 0.0;
        }
        
        return closest_distance / closing_speed;
    }
    
    void publishVisualization(double ttc_true, double ttc_estimated) {
        // Publish numeric TTC values
        std_msgs::Float64 ttc_true_msg, ttc_estimated_msg;
        ttc_true_msg.data = ttc_true;
        ttc_estimated_msg.data = ttc_estimated;
        ttc_true_pub_.publish(ttc_true_msg);
        ttc_estimated_pub_.publish(ttc_estimated_msg);
        
        // Create marker array for visualization (using noisy/estimated position)
        visualization_msgs::MarkerArray marker_array;
        
        // Line marker (estimated)
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "ttc_line_estimated";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point p1, p2;
        p1.x = manual_vehicle_noisy_.x;
        p1.y = manual_vehicle_noisy_.y;
        p1.z = manual_vehicle_noisy_.z + 1.0;
        p2.x = static_vehicle_.x;
        p2.y = static_vehicle_.y;
        p2.z = static_vehicle_.z + 1.0;
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.scale.x = 0.1;
        
        // Color based on estimated TTC
        if (ttc_estimated < 0) {
            line_marker.color.r = 0.5;
            line_marker.color.g = 0.5;
            line_marker.color.b = 0.5;
            line_marker.color.a = 0.5;
        } else if (ttc_estimated <= ttc_critical_threshold_) {
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
        } else if (ttc_estimated <= ttc_warning_threshold_) {
            line_marker.color.r = 1.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.8;
        } else {
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 0.6;
        }
        
        line_marker.lifetime = ros::Duration(0.2);
        marker_array.markers.push_back(line_marker);
        
        // Line marker (true) - in blue
        visualization_msgs::Marker line_marker_true;
        line_marker_true.header.frame_id = "map";
        line_marker_true.header.stamp = ros::Time::now();
        line_marker_true.ns = "ttc_line_true";
        line_marker_true.id = 1;
        line_marker_true.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker_true.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point p1_true, p2_true;
        p1_true.x = manual_vehicle_true_.x;
        p1_true.y = manual_vehicle_true_.y;
        p1_true.z = manual_vehicle_true_.z + 1.2;
        p2_true.x = static_vehicle_.x;
        p2_true.y = static_vehicle_.y;
        p2_true.z = static_vehicle_.z + 1.2;
        
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
        
        // Text marker showing both TTCs
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "ttc_text";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        text_marker.pose.position.x = (manual_vehicle_noisy_.x + static_vehicle_.x) / 2.0;
        text_marker.pose.position.y = (manual_vehicle_noisy_.y + static_vehicle_.y) / 2.0;
        text_marker.pose.position.z = std::max(manual_vehicle_noisy_.z, static_vehicle_.z) + 3.0;
        
        std::stringstream ss;
        double distance_true = calculateClosestDistance(static_vehicle_, manual_vehicle_true_);
        double distance_est = calculateClosestDistance(static_vehicle_, manual_vehicle_noisy_);
        
        if (ttc_true >= 0 && ttc_estimated >= 0) {
            ss << "TTC (True): " << std::fixed << std::setprecision(2) << ttc_true << " s\n";
            ss << "TTC (Est):  " << std::fixed << std::setprecision(2) << ttc_estimated << " s\n";
            ss << "Error: " << std::fixed << std::setprecision(2) << std::abs(ttc_true - ttc_estimated) << " s\n";
            ss << "Dist (True): " << std::fixed << std::setprecision(1) << distance_true << " m\n";
            ss << "Dist (Est):  " << std::fixed << std::setprecision(1) << distance_est << " m";
        } else {
            ss << "TTC: N/A (Not Approaching)";
        }
        
        text_marker.text = ss.str();
        text_marker.scale.z = 0.8;
        text_marker.color = line_marker.color;
        text_marker.lifetime = ros::Duration(0.2);
        
        ttc_text_pub_.publish(text_marker);
    }
    
    void timerCallback(const ros::TimerEvent&) {
        double ttc_true = calculateTTC(manual_vehicle_true_);
        double ttc_estimated = calculateTTC(manual_vehicle_noisy_);
        
        publishVisualization(ttc_true, ttc_estimated);
        
        // Console output
        if (ttc_true >= 0 || ttc_estimated >= 0) {
            double distance_true = calculateClosestDistance(static_vehicle_, manual_vehicle_true_);
            double distance_est = calculateClosestDistance(static_vehicle_, manual_vehicle_noisy_);
            
            double pos_error = calculateDistance2D(manual_vehicle_true_.x, manual_vehicle_true_.y,
                                                   manual_vehicle_noisy_.x, manual_vehicle_noisy_.y);
            
            if (ttc_true >= 0 && ttc_estimated >= 0) {
                double ttc_error = std::abs(ttc_true - ttc_estimated);
                ROS_INFO_THROTTLE(1.0, "TTC True: %.2f s | TTC Est: %.2f s | Error: %.2f s | Pos Error: %.2f m", 
                                 ttc_true, ttc_estimated, ttc_error, pos_error);
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