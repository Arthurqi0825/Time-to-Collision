#!/usr/bin/env python3
"""
CARLA ROS Publisher
Publishes vehicle data from CARLA simulation to ROS topics:
- Car positions
- Car speeds
- Bounding boxes
"""

import carla
import rospy
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Float32
import math
import random

from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import Float32, Header   
from sensor_msgs.msg import Image, PointCloud2, PointField

class CarlaROSPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('carla_ros_publisher', anonymous=True)
        
        # Publishers for front car
        self.pub_front_position = rospy.Publisher('/front_car/position', Point, queue_size=10)
        self.pub_front_speed = rospy.Publisher('/front_car/speed', Float32, queue_size=10)
        self.pub_front_bbox = rospy.Publisher('/front_car/bounding_box', Vector3, queue_size=10)
        self.pub_front_pose = rospy.Publisher('/front_car/pose', Pose, queue_size=10)
        
        # Publishers for rear car
        self.pub_rear_position = rospy.Publisher('/rear_car/position', Point, queue_size=10)
        self.pub_rear_speed = rospy.Publisher('/rear_car/speed', Float32, queue_size=10)
        self.pub_rear_bbox = rospy.Publisher('/rear_car/bounding_box', Vector3, queue_size=10)
        self.pub_rear_pose = rospy.Publisher('/rear_car/pose', Pose, queue_size=10)

        self.pub_camera = rospy.Publisher('/carla/camera/image', Image, queue_size=10)
        self.pub_lidar = rospy.Publisher('/carla/lidar/points', PointCloud2, queue_size=10)

        # CARLA variables
        self.client = None
        self.world = None
        self.front_vehicle = None
        self.rear_vehicle = None
        self.front_speed = 25.0
        self.rear_speed = 22.0
        self.min_distance = 10.0
        
        rospy.loginfo("CARLA ROS Publisher initialized")
    
    def connect_to_carla(self):
        """Connect to CARLA simulator"""
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            rospy.loginfo("Connected to CARLA simulator")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to connect to CARLA: {e}")
            return False
    
    def setup_world(self):
        """Setup world settings"""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
    
    def spawn_vehicles(self):
        """Spawn two vehicles"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bps = blueprint_library.filter('vehicle.*')
        bp1 = random.choice(vehicle_bps)
        bp2 = random.choice(vehicle_bps)
        
        spawn_points = self.world.get_map().get_spawn_points()
        
        if len(spawn_points) < 2:
            rospy.logerr("Not enough spawn points!")
            return False
        
        # Spawn front vehicle
        spawn_point_front = spawn_points[0]
        self.front_vehicle = self.world.try_spawn_actor(bp1, spawn_point_front)
        
        # Spawn rear vehicle
        spawn_point_rear = carla.Transform(
            carla.Location(
                x=spawn_point_front.location.x - 15.0,
                y=spawn_point_front.location.y,
                z=spawn_point_front.location.z
            ),
            spawn_point_front.rotation
        )
        self.rear_vehicle = self.world.try_spawn_actor(bp2, spawn_point_rear)
        
        if self.front_vehicle is None or self.rear_vehicle is None:
            rospy.logerr("Failed to spawn vehicles")
            return False
        
        rospy.loginfo(f"Vehicles spawned successfully")
        return True
    

    def setup_sensors(self):
        """Setup Camera and LiDAR on the rear vehicle"""
        if self.rear_vehicle is None:
            return

        blueprint_library = self.world.get_blueprint_library()

        # 1. Calculate attachment position (Front bumper, slightly raised)
        bound_x = self.rear_vehicle.bounding_box.extent.x
        sensor_transform = carla.Transform(carla.Location(x=bound_x, z=1.5))

        # --- 2. RGB Camera ---
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.im_width))
        cam_bp.set_attribute('image_size_y', str(self.im_height))
        cam_bp.set_attribute('fov', '90')
        cam_bp.set_attribute('sensor_tick', '0.05') # Match simulation step roughly
        
        camera = self.world.spawn_actor(cam_bp, sensor_transform, attach_to=self.rear_vehicle)
        self.sensor_actors.append(camera)
        
        # Register callback
        camera.listen(self.process_camera_data)
        rospy.loginfo("Camera spawned and listening")

        # --- 3. LiDAR ---
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_bp.set_attribute('sensor_tick', '0.05')
        
        lidar = self.world.spawn_actor(lidar_bp, sensor_transform, attach_to=self.rear_vehicle)
        self.sensor_actors.append(lidar)
        
        # Register callback
        lidar.listen(self.process_lidar_data)
        rospy.loginfo("LiDAR spawned and listening")

    def process_camera_data(self, image):
        """Callback to publish camera data to ROS"""
        # Convert raw data to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4)) # BGRA
        array = array[:, :, :3] # Drop Alpha channel -> BGR
        
        # Create ROS Image message
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "rear_car_camera_link"
        img_msg.height = image.height
        img_msg.width = image.width
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.step = image.width * 3
        img_msg.data = array.tobytes()
        
        self.pub_camera.publish(img_msg)

    def process_lidar_data(self, lidar_data):
        """Callback to publish LiDAR data to ROS"""
        # CARLA gives float32 x,y,z and maybe intensity. 
        # We need to construct a PointCloud2 message.
        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "rear_car_lidar_link"
        
        # Define fields for PointCloud2
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # CARLA 0.9.10+ provides intensity in 4th float, usually
            # PointField('intensity', 12, PointField.FLOAT32, 1),
        ]
        
        # Convert raw buffer directly
        # CARLA Raw data is a buffer of float32s (x, y, z, intensity)
        # We can just pass the bytes if the endianness matches (usually does on x86)
        
        lidar_msg = PointCloud2()
        lidar_msg.header = header
        lidar_msg.height = 1
        lidar_msg.width = int(len(lidar_data.raw_data) / 16) # 4 floats * 4 bytes = 16 bytes per point
        lidar_msg.fields = fields
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 16 # 4 * 4 bytes
        lidar_msg.row_step = len(lidar_data.raw_data)
        lidar_msg.is_dense = True
        lidar_msg.data = lidar_data.raw_data # Direct copy
        
        self.pub_lidar.publish(lidar_msg)

    def get_vehicle_speed(self, vehicle):
        """Get vehicle speed in km/h"""
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return speed
    
    def get_vehicle_length(self, vehicle):
        """Get vehicle length"""
        bb = vehicle.bounding_box
        return bb.extent.x * 2
    
    def get_distance_between_vehicles(self):
        """Calculate distance between vehicles"""
        front_loc = self.front_vehicle.get_location()
        rear_loc = self.rear_vehicle.get_location()
        
        distance = math.sqrt(
            (front_loc.x - rear_loc.x)**2 + 
            (front_loc.y - rear_loc.y)**2
        )
        
        front_length = self.get_vehicle_length(self.front_vehicle)
        distance -= front_length / 2
        
        return distance
    
    def adjust_front_speed(self):
        """Adjust front vehicle speed dynamically"""
        self.front_speed = random.uniform(20.0, 30.0)
    
    def control_vehicles(self):
        """Control vehicle speeds with collision avoidance"""
        distance = self.get_distance_between_vehicles()
        
        if distance < self.min_distance:
            actual_rear_speed = self.rear_speed * 0.8
        else:
            actual_rear_speed = self.rear_speed
        
        front_velocity = self.front_speed / 3.6
        rear_velocity = actual_rear_speed / 3.6
        
        front_transform = self.front_vehicle.get_transform()
        rear_transform = self.rear_vehicle.get_transform()
        
        front_forward = front_transform.get_forward_vector()
        rear_forward = rear_transform.get_forward_vector()
        
        self.front_vehicle.set_target_velocity(carla.Vector3D(
            front_forward.x * front_velocity,
            front_forward.y * front_velocity,
            front_forward.z * front_velocity
        ))
        
        self.rear_vehicle.set_target_velocity(carla.Vector3D(
            rear_forward.x * rear_velocity,
            rear_forward.y * rear_velocity,
            rear_forward.z * rear_velocity
        ))
    
    def publish_vehicle_data(self, vehicle, position_pub, speed_pub, bbox_pub, pose_pub):
        """Publish vehicle data to ROS topics"""
        # Get vehicle data
        location = vehicle.get_location()
        transform = vehicle.get_transform()
        rotation = transform.rotation
        bbox = vehicle.bounding_box
        speed = self.get_vehicle_speed(vehicle)
        
        # Publish position
        position_msg = Point()
        position_msg.x = location.x
        position_msg.y = location.y
        position_msg.z = location.z
        position_pub.publish(position_msg)
        
        # Publish speed
        speed_msg = Float32()
        speed_msg.data = speed
        speed_pub.publish(speed_msg)
        
        # Publish bounding box (extent is half dimensions)
        bbox_msg = Vector3()
        bbox_msg.x = bbox.extent.x * 2  # length
        bbox_msg.y = bbox.extent.y * 2  # width
        bbox_msg.z = bbox.extent.z * 2  # height
        bbox_pub.publish(bbox_msg)
        
        # Publish full pose
        pose_msg = Pose()
        pose_msg.position.x = location.x
        pose_msg.position.y = location.y
        pose_msg.position.z = location.z
        
        # Convert rotation to quaternion (simplified)
        yaw = math.radians(rotation.yaw)
        pitch = math.radians(rotation.pitch)
        roll = math.radians(rotation.roll)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose_msg.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        pose_pub.publish(pose_msg)
    
    def print_vehicle_info(self):
        """Print vehicle information to terminal"""
        front_length = self.get_vehicle_length(self.front_vehicle)
        rear_length = self.get_vehicle_length(self.rear_vehicle)
        front_speed = self.get_vehicle_speed(self.front_vehicle)
        rear_speed = self.get_vehicle_speed(self.rear_vehicle)
        
        print("\n" + "="*60)
        print(f"FRONT CAR - Length: {front_length:.2f}m, Speed: {front_speed:.2f} km/h")
        print(f"REAR CAR  - Length: {rear_length:.2f}m, Speed: {rear_speed:.2f} km/h")
        print("="*60)
    
    def run(self):
        """Main loop"""
        if not self.connect_to_carla():
            return
        
        self.setup_world()
        
        if not self.spawn_vehicles():
            return
        
        rate = rospy.Rate(20)  # 20 Hz
        frame_count = 0
        
        try:
            while not rospy.is_shutdown():
                # Adjust front car speed periodically
                if frame_count % 100 == 0:
                    self.adjust_front_speed()
                
                # Control vehicles
                self.control_vehicles()
                
                # Tick CARLA world
                self.world.tick()
                
                # Publish data for both vehicles
                self.publish_vehicle_data(
                    self.front_vehicle,
                    self.pub_front_position,
                    self.pub_front_speed,
                    self.pub_front_bbox,
                    self.pub_front_pose
                )
                
                self.publish_vehicle_data(
                    self.rear_vehicle,
                    self.pub_rear_position,
                    self.pub_rear_speed,
                    self.pub_rear_bbox,
                    self.pub_rear_pose
                )
                
                # Print info every 10 frames
                if frame_count % 10 == 0:
                    self.print_vehicle_info()
                
                frame_count += 1
                rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS node interrupted")
        except KeyboardInterrupt:
            rospy.loginfo("Simulation stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up actors"""
        if self.front_vehicle:
            self.front_vehicle.destroy()
        if self.rear_vehicle:
            self.rear_vehicle.destroy()
        rospy.loginfo("Cleanup completed")

if __name__ == '__main__':
    try:
        publisher = CarlaROSPublisher()
        publisher.run()
    except Exception as e:
        rospy.logerr(f"Error: {e}")