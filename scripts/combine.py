#!/usr/bin/env python3

"""
CARLA Dual Vehicle System with ROS Position Publishing

This script:
1. Spawns a static vehicle at a random spawn point
2. Spawns a manual control vehicle near the static vehicle (within configurable radius)
3. Publishes both vehicles' positions and velocities to ROS topics
4. Provides keyboard control for the manual vehicle

ROS Topics:
- /vehicle/static/pose - Static vehicle position (geometry_msgs/PoseStamped)
- /vehicle/static/velocity - Static vehicle velocity (geometry_msgs/TwistStamped)
- /vehicle/static/bounding_box - Static vehicle bounding box (visualization_msgs/Marker)
- /vehicle/manual/pose - Manual control vehicle position (geometry_msgs/PoseStamped)
- /vehicle/manual/velocity - Manual control vehicle velocity (geometry_msgs/TwistStamped)
- /vehicle/manual/bounding_box - Manual control vehicle bounding box (visualization_msgs/Marker)

Controls:
    W/S          : throttle/brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    ESC          : quit
"""

import carla
import argparse
import weakref
import random
import time
import math
import sys
from datetime import datetime

try:
    import pygame
    from pygame.locals import K_ESCAPE, K_SPACE, K_a, K_d, K_s, K_w, K_q, K_p
    from pygame.locals import KMOD_CTRL
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3
    from std_msgs.msg import Header, ColorRGBA
    from visualization_msgs.msg import Marker
    ROS_AVAILABLE = True
except ImportError:
    print("WARNING: ROS not available. Install ROS or rospy to enable ROS publishing.")
    print("Continuing without ROS support...\n")
    ROS_AVAILABLE = False


# ==============================================================================
# -- ROS Publisher -------------------------------------------------------------
# ==============================================================================

class ROSPositionPublisher:
    """Publishes vehicle position and velocity data to ROS topics"""
    
    def __init__(self, vehicle_name, output_file=None):
        self.vehicle_name = vehicle_name
        self.output_file = output_file
        self.message_count = 0
        
        if ROS_AVAILABLE:
            # Initialize ROS publishers
            self.pose_topic = f"/vehicle/{vehicle_name}/pose"
            self.velocity_topic = f"/vehicle/{vehicle_name}/velocity"
            self.bbox_topic = f"/vehicle/{vehicle_name}/bounding_box"
            
            self.pose_pub = rospy.Publisher(
                self.pose_topic, 
                PoseStamped, 
                queue_size=10
            )
            self.velocity_pub = rospy.Publisher(
                self.velocity_topic, 
                TwistStamped, 
                queue_size=10
            )
            self.bbox_pub = rospy.Publisher(
                self.bbox_topic,
                Marker,
                queue_size=10
            )
            
            print(f"ROS publishers initialized for {vehicle_name}:")
            print(f"  - {self.pose_topic}")
            print(f"  - {self.velocity_topic}")
            print(f"  - {self.bbox_topic}")
        else:
            print(f"Console-only publisher for {vehicle_name} (ROS not available)")
            self.pose_topic = f"/vehicle/{vehicle_name}/pose_velocity"
        
        # CSV logging
        if self.output_file:
            with open(self.output_file, 'w') as f:
                f.write(f"# Vehicle: {vehicle_name}\n")
                f.write("# Format: timestamp, x, y, z, qx, qy, qz, qw, vx, vy, vz, speed_kmh, bbox_x, bbox_y, bbox_z\n")
    
    def publish(self, vehicle):
        """Publish vehicle position and velocity to ROS topics"""
        if vehicle is None:
            return
        
        transform = vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation
        velocity = vehicle.get_velocity()
        
        # Get bounding box
        bounding_box = vehicle.bounding_box
        bbox_extent = bounding_box.extent  # Half-size of the bounding box
        
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Convert CARLA rotation to quaternion
        qx, qy, qz, qw = self._euler_to_quaternion(
            math.radians(rotation.pitch),
            math.radians(rotation.yaw),
            math.radians(rotation.roll)
        )
        
        if ROS_AVAILABLE:
            # Create header with current timestamp
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            
            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header = header
            pose_msg.pose.position.x = location.x
            pose_msg.pose.position.y = location.y
            pose_msg.pose.position.z = location.z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            
            self.pose_pub.publish(pose_msg)
            
            # Publish TwistStamped
            velocity_msg = TwistStamped()
            velocity_msg.header = header
            velocity_msg.twist.linear.x = velocity.x
            velocity_msg.twist.linear.y = velocity.y
            velocity_msg.twist.linear.z = velocity.z
            velocity_msg.twist.angular.x = 0.0
            velocity_msg.twist.angular.y = 0.0
            velocity_msg.twist.angular.z = 0.0
            
            self.velocity_pub.publish(velocity_msg)
            
            # Publish Bounding Box as Marker
            bbox_msg = Marker()
            bbox_msg.header = header
            bbox_msg.ns = f"vehicle_{self.vehicle_name}"
            bbox_msg.id = 0
            bbox_msg.type = Marker.CUBE
            bbox_msg.action = Marker.ADD
            
            # Position (center of bounding box)
            bbox_msg.pose.position.x = location.x + bounding_box.location.x
            bbox_msg.pose.position.y = location.y + bounding_box.location.y
            bbox_msg.pose.position.z = location.z + bounding_box.location.z
            bbox_msg.pose.orientation.x = qx
            bbox_msg.pose.orientation.y = qy
            bbox_msg.pose.orientation.z = qz
            bbox_msg.pose.orientation.w = qw
            
            # Scale (full size of bounding box)
            bbox_msg.scale.x = bbox_extent.x * 2.0
            bbox_msg.scale.y = bbox_extent.y * 2.0
            bbox_msg.scale.z = bbox_extent.z * 2.0
            
            # Color (semi-transparent)
            if self.vehicle_name == "static":
                bbox_msg.color.r = 1.0
                bbox_msg.color.g = 0.0
                bbox_msg.color.b = 0.0
            else:
                bbox_msg.color.r = 0.0
                bbox_msg.color.g = 1.0
                bbox_msg.color.b = 0.0
            bbox_msg.color.a = 0.5
            
            bbox_msg.lifetime = rospy.Duration(0.2)  # Marker lifetime
            
            self.bbox_pub.publish(bbox_msg)
        
        # Console output (throttled)
        if self.message_count % 10 == 0:
            print(f"\n[{self.pose_topic}]")
            print(f"  Position: ({location.x:.2f}, {location.y:.2f}, {location.z:.2f})")
            print(f"  Quaternion: ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
            print(f"  Velocity: ({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f})")
            print(f"  Speed: {speed_kmh:.2f} km/h")
            print(f"  BBox Size: ({bbox_extent.x*2:.2f}, {bbox_extent.y*2:.2f}, {bbox_extent.z*2:.2f})")
        
        # CSV logging
        if self.output_file:
            with open(self.output_file, 'a') as f:
                timestamp = time.time()
                f.write(f"{timestamp:.3f}, {location.x:.3f}, {location.y:.3f}, {location.z:.3f}, "
                       f"{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}, "
                       f"{velocity.x:.3f}, {velocity.y:.3f}, {velocity.z:.3f}, {speed_kmh:.3f}, "
                       f"{bbox_extent.x*2:.3f}, {bbox_extent.y*2:.3f}, {bbox_extent.z*2:.3f}\n")
        
        self.message_count += 1
    
    def _euler_to_quaternion(self, pitch, yaw, roll):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw


# ==============================================================================
# -- Keyboard Control ----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """Keyboard control handler for manual vehicle"""
    
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        world.player.set_autopilot(self._autopilot_enabled)
        self._world = world
        
    def parse_events(self, clock):
        """Parse keyboard events and apply controls"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if event.key == K_ESCAPE:
                    return True
        
        keys = pygame.key.get_pressed()
        milliseconds = clock.get_time()
        
        if keys[K_p]:
            self._autopilot_enabled = not self._autopilot_enabled
            self._world.player.set_autopilot(self._autopilot_enabled)
            time.sleep(0.2)
        
        if not self._autopilot_enabled:
            self._parse_vehicle_keys(keys, milliseconds)
            self._world.player.apply_control(self._control)
        
        return False
    
    def _parse_vehicle_keys(self, keys, milliseconds):
        """Parse vehicle control keys"""
        if keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.1, 1.0)
        else:
            self._control.throttle = 0.0
        
        if keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1.0)
        else:
            self._control.brake = 0.0
        
        # Improved steering with better responsiveness
        steer_increment = 0.05  # Faster steering response
        if keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0  # Quick reset when changing direction
            self._steer_cache -= steer_increment
        elif keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0  # Quick reset when changing direction
            self._steer_cache += steer_increment
        else:
            # Gradually return to center when no key pressed
            if abs(self._steer_cache) > steer_increment:
                self._steer_cache -= steer_increment * (1 if self._steer_cache > 0 else -1)
            else:
                self._steer_cache = 0.0
        
        # Clamp steering to valid range
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = self._steer_cache
        
        if keys[K_q]:
            self._control.reverse = not self._control.reverse
            time.sleep(0.2)
        
        self._control.hand_brake = keys[K_SPACE]


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================

class HUD(object):
    """Simple HUD to display vehicle information"""
    
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        self._notifications = []
        self._info_text = []
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
    
    def tick(self, world, clock):
        """Update HUD information"""
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            ''
        ]
        
        if ROS_AVAILABLE:
            self._info_text.append('ROS: Connected')
        else:
            self._info_text.append('ROS: Not Available')
        
        self._info_text.append('')
        
        if world.player is not None:
            vehicle = world.player
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            self._info_text += [
                'Manual Control Vehicle:',
                '  Speed:   % 15.0f km/h' % speed,
                '  Position: (% 5.1f, % 5.1f, % 5.1f)' % (
                    transform.location.x, transform.location.y, transform.location.z),
                ''
            ]
        
        if world.static_vehicle is not None:
            transform = world.static_vehicle.get_transform()
            self._info_text += [
                'Static Vehicle:',
                '  Position: (% 5.1f, % 5.1f, % 5.1f)' % (
                    transform.location.x, transform.location.y, transform.location.z),
            ]
            
            if world.player is not None:
                static_loc = world.static_vehicle.get_location()
                manual_loc = world.player.get_location()
                distance = static_loc.distance(manual_loc)
                self._info_text.append(f'  Distance: {distance:.1f} m')
    
    def render(self, display):
        """Render HUD on display"""
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        info_surface = pygame.Surface((250, self.dim[1]))
        info_surface.set_alpha(100)
        display.blit(info_surface, (0, 0))
        
        v_offset = 10
        for item in self._info_text:
            surface = font.render(item, True, (255, 255, 255))
            display.blit(surface, (10, v_offset))
            v_offset += 25
    
    def on_world_tick(self, timestamp):
        """Callback for world tick"""
        self.server_fps = timestamp.delta_seconds
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World(object):
    """Main world handler"""
    
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.static_vehicle = None
        self.camera_manager = None
        self._actor_filter = args.filter
        
        self.static_publisher = ROSPositionPublisher('static', args.static_log)
        self.manual_publisher = ROSPositionPublisher('manual', args.manual_log)
        
        self.spawn_static_vehicle()
        self.spawn_manual_vehicle(args.spawn_radius)
        
        if self.player is not None:
            self.setup_camera()
        
        self.world.on_tick(hud.on_world_tick)
    
    def spawn_static_vehicle(self):
        """Spawn a static vehicle at a random spawn point"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        
        spawn_points = self.world.get_map().get_spawn_points()
        
        if not spawn_points:
            print('Error: No spawn points found!')
            sys.exit(1)
        
        spawn_transform = random.choice(spawn_points)
        
        self.static_vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
        self.static_vehicle.set_simulate_physics(False)
        
        location = spawn_transform.location
        print(f"\nStatic vehicle spawned:")
        print(f"  Type: {vehicle_bp.id}")
        print(f"  Position: ({location.x:.2f}, {location.y:.2f}, {location.z:.2f})")
        print(f"  Physics: Disabled (static)")
    
    def spawn_manual_vehicle(self, radius):
        """Spawn a vehicle for manual control near the static vehicle"""
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter(self._actor_filter))
        vehicle_bp.set_attribute('role_name', 'hero')
        
        if self.static_vehicle is None:
            print("Error: Static vehicle must be spawned first!")
            sys.exit(1)
        
        static_location = self.static_vehicle.get_location()
        
        spawn_points = self.world.get_map().get_spawn_points()
        
        nearby_spawns = [
            sp for sp in spawn_points 
            if static_location.distance(sp.location) <= radius
        ]
        
        if not nearby_spawns:
            print(f"Warning: No spawn points within {radius}m of static vehicle")
            print(f"Using nearest spawn point instead...")
            nearby_spawns = sorted(spawn_points, 
                                  key=lambda sp: static_location.distance(sp.location))
            spawn_transform = nearby_spawns[0]
        else:
            spawn_transform = random.choice(nearby_spawns)
        
        self.player = self.world.spawn_actor(vehicle_bp, spawn_transform)
        
        distance = static_location.distance(spawn_transform.location)
        
        print(f"\nManual control vehicle spawned:")
        print(f"  Type: {vehicle_bp.id}")
        print(f"  Position: ({spawn_transform.location.x:.2f}, "
              f"{spawn_transform.location.y:.2f}, {spawn_transform.location.z:.2f})")
        print(f"  Distance to static vehicle: {distance:.2f} m")
    
    def setup_camera(self):
        """Setup camera attached to manual vehicle"""
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        
        camera_transform = carla.Transform(
            carla.Location(x=-5.5, z=2.8),
            carla.Rotation(pitch=-15)
        )
        
        self.camera = self.world.spawn_actor(
            camera_bp, camera_transform, attach_to=self.player)
        
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: World._parse_image(weak_self, image))
        
        self.camera_surface = None
    
    @staticmethod
    def _parse_image(weak_self, image):
        """Parse camera image"""
        self = weak_self()
        if not self:
            return
        
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.camera_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    
    def tick(self, clock):
        """Update world state"""
        self.hud.tick(self, clock)
        
        self.static_publisher.publish(self.static_vehicle)
        self.manual_publisher.publish(self.player)
    
    def render(self, display):
        """Render world"""
        if self.camera_surface is not None:
            display.blit(self.camera_surface, (0, 0))
        self.hud.render(display)
    
    def destroy(self):
        """Clean up all actors"""
        actors = [self.camera, self.player, self.static_vehicle]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- Game Loop -----------------------------------------------------------------
# ==============================================================================

def game_loop(args):
    """Main game loop"""
    pygame.init()
    pygame.font.init()
    world = None
    
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        
        sim_world = client.get_world()
        
        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)
        
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        
        pygame.display.set_caption('CARLA - Dual Vehicle System with ROS')
        
        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = KeyboardControl(world, args.autopilot)
        
        clock = pygame.time.Clock()
        
        print("\n" + "="*70)
        print("SYSTEM READY")
        print("="*70)
        if ROS_AVAILABLE:
            print("\nROS Topics Published:")
            print(f"  - {world.static_publisher.pose_topic}")
            print(f"  - {world.static_publisher.velocity_topic}")
            print(f"  - {world.static_publisher.bbox_topic}")
            print(f"  - {world.manual_publisher.pose_topic}")
            print(f"  - {world.manual_publisher.velocity_topic}")
            print(f"  - {world.manual_publisher.bbox_topic}")
        else:
            print("\nROS NOT AVAILABLE - Running in console-only mode")
        print("\nControls: W/A/S/D for movement, Space for brake, P for autopilot")
        print("Press ESC to quit\n")
        
        while True:
            if args.sync:
                sim_world.tick()
            
            clock.tick(60)
            
            if controller.parse_events(clock):
                break
            
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
    
    finally:
        if world is not None:
            world.destroy()
        pygame.quit()
        print("\nShutdown complete.")


# ==============================================================================
# -- Main ----------------------------------------------------------------------
# ==============================================================================

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Dual Vehicle System with ROS Position Publishing')
    
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='800x600',
        help='window resolution (default: 800x600)')
    
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter for manual vehicle (default: "vehicle.*")')
    
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='Start manual vehicle in autopilot mode')
    
    argparser.add_argument(
        '--spawn-radius',
        type=float,
        default=50.0,
        help='Maximum distance for manual vehicle spawn from static vehicle (default: 50.0 meters)')
    
    argparser.add_argument(
        '--static-log',
        metavar='FILE',
        help='Log static vehicle data to CSV file')
    
    argparser.add_argument(
        '--manual-log',
        metavar='FILE',
        help='Log manual vehicle data to CSV file')
    
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]
    
    # Initialize ROS node if available
    if ROS_AVAILABLE:
        rospy.init_node('carla_dual_vehicle_publisher', anonymous=True)
        print("ROS node initialized: carla_dual_vehicle_publisher")
    
    print(__doc__)
    
    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except Exception as e:
        print(f'\nError: {e}')
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()