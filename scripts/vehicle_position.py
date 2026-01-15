#!/usr/bin/env python3
"""
CARLA Vehicle Position Tracker
This script connects to a CARLA simulator and displays the positions of all vehicles.
"""

import carla
import time
import sys


def get_all_vehicle_positions(world):
    """
    Get positions of all vehicles in the CARLA world.
    
    Args:
        world: CARLA world object
        
    Returns:
        List of dictionaries containing vehicle information
    """
    vehicles = []
    
    # Get all actors in the world
    actors = world.get_actors()
    
    # Filter for vehicles
    vehicle_actors = actors.filter('vehicle.*')
    
    for vehicle in vehicle_actors:
        # Get vehicle transform (location and rotation)
        transform = vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation
        
        # Get velocity
        velocity = vehicle.get_velocity()
        speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5  # km/h
        
        vehicle_info = {
            'id': vehicle.id,
            'type': vehicle.type_id,
            'location': {
                'x': location.x,
                'y': location.y,
                'z': location.z
            },
            'rotation': {
                'pitch': rotation.pitch,
                'yaw': rotation.yaw,
                'roll': rotation.roll
            },
            'speed': speed
        }
        
        vehicles.append(vehicle_info)
    
    return vehicles


def display_vehicle_positions(vehicles):
    """
    Display vehicle positions in a formatted way.
    
    Args:
        vehicles: List of vehicle information dictionaries
    """
    print("\n" + "="*80)
    print(f"Total Vehicles: {len(vehicles)}")
    print("="*80)
    
    for i, vehicle in enumerate(vehicles, 1):
        print(f"\nVehicle #{i}")
        print(f"  ID: {vehicle['id']}")
        print(f"  Type: {vehicle['type']}")
        print(f"  Position (x, y, z): ({vehicle['location']['x']:.2f}, "
              f"{vehicle['location']['y']:.2f}, {vehicle['location']['z']:.2f})")
        print(f"  Rotation (pitch, yaw, roll): ({vehicle['rotation']['pitch']:.2f}°, "
              f"{vehicle['rotation']['yaw']:.2f}°, {vehicle['rotation']['roll']:.2f}°)")
        print(f"  Speed: {vehicle['speed']:.2f} km/h")
    
    print("\n" + "="*80)


def main():
    """
    Main function to connect to CARLA and display vehicle positions.
    """
    try:
        # Connect to CARLA server
        print("Connecting to CARLA server...")
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        
        # Get the world
        world = client.get_world()
        print(f"Connected to CARLA world: {world.get_map().name}")
        
        # Get and display vehicle positions
        vehicles = get_all_vehicle_positions(world)
        
        if not vehicles:
            print("\nNo vehicles found in the simulation.")
            print("You may need to spawn vehicles first.")
        else:
            display_vehicle_positions(vehicles)
        
        # Optional: Continuous monitoring mode
        print("\nPress Ctrl+C to exit continuous monitoring mode...")
        print("Updating vehicle positions every 2 seconds...\n")
        
        try:
            while True:
                time.sleep(2)
                vehicles = get_all_vehicle_positions(world)
                
                # Clear screen (works on Unix-like systems)
                print("\033[2J\033[H", end="")
                
                display_vehicle_positions(vehicles)
                
        except KeyboardInterrupt:
            print("\n\nStopped monitoring.")
    
    except Exception as e:
        print(f"Error: {e}")
        print("\nMake sure CARLA simulator is running on localhost:2000")
        sys.exit(1)


if __name__ == "__main__":
    main()