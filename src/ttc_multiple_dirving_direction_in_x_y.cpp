#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>

class TTCCalculator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber ego_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher ttc_pub_;
    
    // Parameters
    Eigen::Vector3d front_offset_, back_offset_;
    double front_vehicle_bbox_length_, back_vehicle_bbox_length_;
    double safety_threshold_;
    
    // Eigen::Vector3d back_prev_position_, front_prev_position_;
    // Eigen::Vector3d back_curr_position_, front_curr_position_;
    // Eigen::Vector3d back_velocity_, front_velocity_;
    struct State
    {
        double timestamp;
        Eigen::Vector3d center_position;
    };
    
    Eigen::Vector3d front_direction_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d back_direction_ = -Eigen::Vector3d::UnitX();
    // State variables
    State ego_state_prev_;
    State ego_state_curr_;
    State target_state_prev_;
    State target_state_curr_;

    geometry_msgs::PoseStamped ego_pose_, ego_pose_prev_;
    geometry_msgs::PoseStamped target_pose_, target_pose_prev_;

    bool ego_initialized_;
    bool target_initialized_;
    
    ros::Time fronconditiont_last_update_time_;
    ros::Time back_last_update_time_;

public:
    TTCCalculator() : ego_initialized_(false), target_initialized_(false) {
        // Load parameters
        loadParameters();
        
        // Setup subscribers
        ego_sub_ = nh_.subscribe("/ego_vehicle/pose", 10, 
                                 &TTCCalculator::egoSensorCallback, this);
        target_sub_ = nh_.subscribe("/target_object/pose", 10, 
                                    &TTCCalculator::targetSensorCallback, this);
        
        // Setup publisher
        ttc_pub_ = nh_.advertise<std_msgs::Float64>("/ttc_estimate", 10);
        
        ROS_INFO("TTC Calculator initialized");
    }
    
    void loadParameters() {
        // Extrinsic matrix (sensor to vehicle origin transform)
        std::vector<double> front_ext_vec;
        if (nh_.getParam("/ttc_calculator/front_translation", front_ext_vec) && front_ext_vec.size() == 3) {
            front_offset_ = Eigen::Map<Eigen::Vector3d>(front_ext_vec.data());
        } else {
            // Default: zero offset
            front_offset_ = Eigen::Vector3d::Zero();
            ROS_WARN("Using zero front translation");
        }
        
        std::vector<double> back_ext_vec;
        if (nh_.getParam("/ttc_calculator/back_translation", back_ext_vec) && back_ext_vec.size() == 3) {
            back_offset_ = Eigen::Map<Eigen::Vector3d>(back_ext_vec.data());
        } else {
            // Default: zero offset
            back_offset_ = Eigen::Vector3d::Zero();
            ROS_WARN("Using zero back translation");
        }
        
        // Vehicle bounding box length (1D for longitudinal direction)
        nh_.param("/ttc_calculator/front_vehicle_bbox_length", front_vehicle_bbox_length_, 4.5);
        nh_.param("/ttc_calculator/back_vehicle_bbox_length", back_vehicle_bbox_length_, 4.5);

        // Safety threshold for TTC warning
        nh_.param("/ttc_calculator/safety_threshold", safety_threshold_, 3.0);
        
        ROS_INFO("Vehicle bbox length: %.2f m", front_vehicle_bbox_length_);
    }
    
    void egoSensorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!ego_initialized_) {
            ego_state_prev_.timestamp = msg.header.stamp.toSec();
            ego_state_prev_.center_position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

            ego_initialized_ = true;
            return;
        }

        // Store ego current pose
        ego_state_curr_.timestamp = msg.header.stamp.toSec();
        ego_state_curr_.center_position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

        computeTTC();
    }

    void targetSensorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (!target_initialized_) {
            target_state_prev_.timestamp = msg.header.stamp.toSec();
            target_state_prev_.center_position = Eigen::Vector2d(msg.pose.position.x, msg.pose.position.y);
            target_initialized_ = true;
            return;
        }

        // Store target current pose
        target_state_curr_.timestamp = msg.header.stamp.toSec();
        target_state_curr_.center_position = Eigen::Vector2d(msg.pose.position.x, msg.pose.position.y);
        computeTTC();
    }
    
    Eigen::Vector3d CalculatingVelocity(State previous_state, State current_state) {
        double dt = (current_state.timestamp - previous_state.timestamp);
        Eigen::Vector3d velocity(0.0, 0.0, 0.0);
        if (dt > 0.001) {
            velocity(0) = (current_state.position.x - previous_state.position.x) / dt;
            velocity(1) = (current_state.position.y - previous_state.position.y) / dt;
            velocity(2) = (current_state.position.z - previous_state.position.z) / dt;
        }

        return velocity;
    }

    // Getting Driving Direction
    Eigen::Vector3d GetDrivingDirection(const Eigen::Vector3d& velocity) {
        Eigen::Vector3d direction = velocity.normalized();
        return direction;
    }

    void computeTTC() {
        // both have prev and curr state
        if (!ego_initialized_ || !target_initialized_) {
            return;
        }

        if (ego_state_prev_.timestamp - ego_state_curr_.timestamp > 2 || target_state_prev_.timestamp - target_state_curr_.timestamp > 2) {
            ROS_INFO("No recent data to compute TTC, Need to reinitialize");
            ego_initialized_ = false;
            target_initialized_ = false;
            return;
        }
        
        Eigen::Vector3d ego_pos_sensor(     ego_state_curr_.center_position(0),
                                            ego_state_curr_.center_position(1),
                                            ego_state_curr_.center_position(2)   );
        Eigen::Vector3d target_pos_sensor(  target_state_curr_.center_position(0),
                                            target_state_curr_.center_position(1),
                                            target_state_curr_.center_position(2));

        Eigen::Vector3d ego_center_pos_ = ego_pos_sensor + back_offset_;
        Eigen::Vector3d target_center_pos_ = target_pos_sensor + front_offset_;
        
        // center distance of vehicles
        Eigen::Vector3d relative_dist = target_center_pos_vehicle - ego_center_pos_vehicle;
        // Collision Distance 
        Eigen::Vector2d collision_distance_vec = Eigen::Vector2d(
            relative_dist.x() - (front_vehicle_bbox_length_ / 2.0 + back_vehicle_bbox_length_ / 2.0),
            relative_dist.y() - (front_vehicle_bbox_length_ / 2.0 + back_vehicle_bbox_length_ / 2.0)
        );

        // Account for vehicle bounding box
        // considering the driving direction, the relative distance is a vector in 3d. and now we need to project it to the driving direction
        // relative_dist -= vehicle_bbox_length_ / 2.0;
        Eigen::Vector3d ego_velocity = CalculatingVelocity(ego_state_prev_, ego_state_curr_);
        Eigen::Vector3d target_velocity = CalculatingVelocity(target_state_prev_, target_state_curr_);
        
        ROS_INFO("Ego Velocity: [%.2f, %.2f, %.2f] m/s", ego_velocity(0), ego_velocity(1), ego_velocity(2));
        ROS_INFO("Target Velocity: [%.2f, %.2f, %.2f] m/s", target_velocity(0), target_velocity(1), target_velocity(2));

        // Calculate velocities if we have previous poses
        Eigen::Vector3d ego_direction = GetDrivingDirection(ego_velocity);
        double relative_velocity = (target_velocity - ego_velocity).dot(ego_direction);
        double collision_dist = collision_distance_vec.dot(ego_direction);
        
        last_update_time_ = ego_pose_.header.stamp;
        
        // Calculate TTC
        double ttc = -1.0;  // Invalid TTC
        
        if (relative_velocity > 0.01) {  // Approaching
            ttc = collision_dist / relative_velocity;
            ROS_INFO("Relative Distance: %.2f m, Relative Velocity: %.2f m/s, Time to Collision: %.2f s", collision_dist, relative_velocity, ttc);
            if (ttc < 0) {
                ttc = -1.0;  // Already passed or collision occurred
            }
        } else if (relative_velocity < -0.01) {  // Moving away
            ttc = -2.0;  // Negative TTC indicates diverging
        } else {
            ttc = -3.0;  // Stationary relative motion
        }
        
        // Publish TTC
        std_msgs::Float64 ttc_msg;
        ttc_msg.data = ttc;
        ttc_pub_.publish(ttc_msg);
        
        // Log warning if TTC is critical
        if (ttc > 0 && ttc < safety_threshold_) {
            ROS_WARN("Critical TTC: %.2f seconds! Distance: %.2f m, Relative velocity: %.2f m/s",
                     ttc, relative_dist, relative_velocity);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ttc_calculator");
    TTCCalculator ttc_calculator;
    ros::spin();
    return 0;
}