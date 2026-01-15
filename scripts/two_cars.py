#!/usr/bin/env python3
"""
CARLA Simulator - Two Car Scenario with Sensors
Creates a straight road with two cars moving in the same direction.
Front car has dynamic speed, rear car has constant speed.
Rear car is equipped with Front Camera and LiDAR.
"""

import carla
import random
import time
import math

class CarlaSimulation:
    def __init__(self):
        self.client = None
        self.world = None
        self.front_vehicle = None
        self.rear_vehicle = None
        self.sensor_actors = []  # List to keep track of sensors for cleanup
        self.front_speed = 25.0  # km/h
        self.rear_speed = 22.0   # km/h (constant)
        self.min_distance = 10.0  # minimum safe distance in meters
        
    def connect(self):
        """Connect to CARLA simulator"""
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            print("Connected to CARLA simulator")
            return True
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            return False
    
    def setup_world(self):
        """Setup the world settings"""
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        
    def spawn_vehicles(self):
        """Spawn two vehicles on a straight road"""
        blueprint_library = self.world.get_blueprint_library()
        
        # Get vehicle blueprints
        vehicle_bps = blueprint_library.filter('vehicle.*')
        bp1 = random.choice(vehicle_bps)
        bp2 = random.choice(vehicle_bps)
        
        # Get spawn points
        spawn_points = self.world.get_map().get_spawn_points()
        
        if len(spawn_points) < 2:
            print("Not enough spawn points!")
            return False
        
        # Spawn front vehicle
        spawn_point_front = spawn_points[0]
        self.front_vehicle = self.world.try_spawn_actor(bp1, spawn_point_front)
        
        # Spawn rear vehicle behind the front one
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
            print("Failed to spawn vehicles")
            return False
        
        print(f"Front vehicle spawned: {bp1.id}")
        print(f"Rear vehicle spawned: {bp2.id}")
        return True

    def setup_sensors(self):
        """Add Camera and LiDAR to the front of the rear vehicle"""
        if self.rear_vehicle is None:
            return

        blueprint_library = self.world.get_blueprint_library()

        # --- 1. Calculate attachment position (Front of vehicle) ---
        # We use the bounding box extent to find the front bumper
        # extent.x is half the length of the car
        bound_x = self.rear_vehicle.bounding_box.extent.x
        bound_z = self.rear_vehicle.bounding_box.extent.z

        # Position: Front bumper (x), slightly raised (z)
        sensor_transform = carla.Transform(
            carla.Location(x=bound_x, z=1.5) # 1.5m high, at the front
        )

        # --- 2. Setup RGB Camera ---
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_bp.set_attribute('fov', '90')
        
        camera = self.world.spawn_actor(
            cam_bp,
            sensor_transform,
            attach_to=self.rear_vehicle
        )
        self.sensor_actors.append(camera)
        
        # Add a listener (currently does nothing, but proves it works)
        # In a real app, you would save image.save_to_disk() here
        camera.listen(lambda image: None) 
        print(f"Camera mounted at x={bound_x:.2f} relative to rear vehicle center.")

        # --- 3. Setup LiDAR ---
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        
        lidar = self.world.spawn_actor(
            lidar_bp,
            sensor_transform,
            attach_to=self.rear_vehicle
        )
        self.sensor_actors.append(lidar)
        
        # Add listener
        lidar.listen(lambda data: None)
        print(f"LiDAR mounted at x={bound_x:.2f} relative to rear vehicle center.")

    def get_vehicle_length(self, vehicle):
        """Get vehicle bounding box length"""
        bb = vehicle.bounding_box
        return bb.extent.x * 2  # extent is half-length
    
    def get_distance_between_vehicles(self):
        """Calculate distance between front and rear vehicles"""
        front_loc = self.front_vehicle.get_location()
        rear_loc = self.rear_vehicle.get_location()
        
        distance = math.sqrt(
            (front_loc.x - rear_loc.x)**2 + 
            (front_loc.y - rear_loc.y)**2
        )
        
        # Subtract vehicle lengths to get actual gap
        front_length = self.get_vehicle_length(self.front_vehicle)
        distance -= front_length / 2
        
        return distance
    
    def get_vehicle_speed(self, vehicle):
        """Get vehicle speed in km/h"""
        velocity = vehicle.get_velocity()
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return speed
    
    def adjust_front_speed(self):
        """Dynamically adjust front vehicle speed"""
        # Random speed variation between 20-30 km/h
        self.front_speed = random.uniform(20.0, 30.0)
    
    def control_vehicles(self):
        """Control vehicle speeds"""
        # Check distance for collision avoidance
        distance = self.get_distance_between_vehicles()
        
        # If too close, slow down rear vehicle
        if distance < self.min_distance:
            actual_rear_speed = self.rear_speed * 0.8
        else:
            actual_rear_speed = self.rear_speed
        
        # Apply velocities (convert km/h to m/s)
        front_velocity = self.front_speed / 3.6
        rear_velocity = actual_rear_speed / 3.6
        
        # Get vehicle forward vectors
        front_transform = self.front_vehicle.get_transform()
        rear_transform = self.rear_vehicle.get_transform()
        
        front_forward = front_transform.get_forward_vector()
        rear_forward = rear_transform.get_forward_vector()
        
        # Set velocities
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
    
    def get_vehicle_data(self):
        """Get all vehicle data for display"""
        front_length = self.get_vehicle_length(self.front_vehicle)
        rear_length = self.get_vehicle_length(self.rear_vehicle)
        front_speed = self.get_vehicle_speed(self.front_vehicle)
        rear_speed = self.get_vehicle_speed(self.rear_vehicle)
        distance = self.get_distance_between_vehicles()
        
        front_loc = self.front_vehicle.get_location()
        rear_loc = self.rear_vehicle.get_location()
        front_bb = self.front_vehicle.bounding_box
        rear_bb = self.rear_vehicle.bounding_box
        
        return {
            'front': {
                'length': front_length,
                'speed': front_speed,
                'position': (front_loc.x, front_loc.y, front_loc.z),
                'bbox': (front_bb.extent.x, front_bb.extent.y, front_bb.extent.z)
            },
            'rear': {
                'length': rear_length,
                'speed': rear_speed,
                'position': (rear_loc.x, rear_loc.y, rear_loc.z),
                'bbox': (rear_bb.extent.x, rear_bb.extent.y, rear_bb.extent.z)
            },
            'distance': distance
        }
    
    def print_vehicle_info(self, data):
        """Print vehicle information to terminal"""
        print("\n" + "="*60)
        print(f"FRONT CAR:")
        print(f"  Length: {data['front']['length']:.2f} m")
        print(f"  Speed: {data['front']['speed']:.2f} km/h")
        print(f"  Position: ({data['front']['position'][0]:.2f}, "
              f"{data['front']['position'][1]:.2f}, {data['front']['position'][2]:.2f})")
        print(f"  BBox (x,y,z): ({data['front']['bbox'][0]:.2f}, "
              f"{data['front']['bbox'][1]:.2f}, {data['front']['bbox'][2]:.2f})")
        
        print(f"\nREAR CAR (SENSORS ATTACHED):")
        print(f"  Length: {data['rear']['length']:.2f} m")
        print(f"  Speed: {data['rear']['speed']:.2f} km/h")
        print(f"  Position: ({data['rear']['position'][0]:.2f}, "
              f"{data['rear']['position'][1]:.2f}, {data['rear']['position'][2]:.2f})")
        
        print(f"\nDistance between cars: {data['distance']:.2f} m")
        print("="*60)
    
    def run(self):
        """Main simulation loop"""
        if not self.connect():
            return
        
        self.setup_world()
        
        if not self.spawn_vehicles():
            return
            
        # Initialize the sensors after vehicles are spawned
        self.setup_sensors()
        
        try:
            frame_count = 0
            while True:
                # Randomly change front car speed every 100 frames
                if frame_count % 100 == 0:
                    self.adjust_front_speed()
                
                self.control_vehicles()
                self.world.tick()
                
                # Get and print vehicle data every 10 frames
                if frame_count % 10 == 0:
                    data = self.get_vehicle_data()
                    self.print_vehicle_info(data)
                
                frame_count += 1
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nSimulation stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up actors"""
        # Clean up sensors first
        for sensor in self.sensor_actors:
            if sensor is not None and sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        print("Sensors cleaned up")
        
        if self.front_vehicle:
            self.front_vehicle.destroy()
        if self.rear_vehicle:
            self.rear_vehicle.destroy()
        print("Vehicles cleaned up")

if __name__ == '__main__':
    sim = CarlaSimulation()
    sim.run()