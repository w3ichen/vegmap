#!/usr/bin/env python3

import sys
import os
import xml.etree.ElementTree as ET
import numpy as np
import yaml

def parse_sdf(sdf_file):
    """Parse SDF file to extract models and their poses"""
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    
    world = root.find('world')
    if world is None:
        print("No world found in SDF file")
        return []
    
    models = []
    
    # Extract all model elements
    for model in world.findall('model') + world.findall('include'):
        name = model.get('name')
        
        # Check for includes (external models)
        if model.tag == 'include':
            uri_elem = model.find('uri')
            if uri_elem is not None:
                uri = uri_elem.text.strip()
            else:
                uri = "unknown"
            
            name_elem = model.find('name')
            if name_elem is not None:
                name = name_elem.text.strip()
        else:
            uri = "internal"
        
        # Extract pose
        pose_elem = model.find('pose')
        if pose_elem is not None:
            pose = [float(x) for x in pose_elem.text.split()]
        else:
            pose = [0, 0, 0, 0, 0, 0]  # Default pose
        
        models.append({
            'name': name,
            'uri': uri,
            'pose': pose,  # [x, y, z, roll, pitch, yaw]
            'type': get_model_type(name, uri)
        })
    
    return models

def get_model_type(name, uri):
    """Determine model type based on name or URI"""
    name_lower = name.lower()
    uri_lower = uri.lower()
    
    if 'shelf' in name_lower or 'shelf' in uri_lower:
        return 'shelf'
    elif 'barrier' in name_lower or 'barrier' in uri_lower:
        return 'barrier'
    elif 'chair' in name_lower or 'chair' in uri_lower:
        return 'chair'
    elif 'table' in name_lower or 'table' in uri_lower:
        return 'table'
    elif 'person' in name_lower or 'person' in uri_lower:
        return 'person'
    elif 'ground' in name_lower or 'ground' in uri_lower:
        return 'ground'
    elif 'warehouse' in name_lower or 'warehouse' in uri_lower:
        return 'warehouse'
    else:
        return 'unknown'

def create_occupancy_grid(models, width=1000, height=1000, resolution=0.05):
    """Create a detailed occupancy grid from model positions"""
    # Create an empty grid (0 = free, 100 = occupied, -1 = unknown)
    grid = np.zeros((height, width), dtype=np.int8)
    
    # Calculate grid center
    center_x = width // 2
    center_y = height // 2
    
    # Draw fixed outline of warehouse walls
    wall_thickness = 15  # Wall thickness in cells
    warehouse_size_x = 900  # 45m
    warehouse_size_y = 900  # 45m
    
    # Convert to grid coordinates
    left_wall = center_x - warehouse_size_x // 2
    right_wall = center_x + warehouse_size_x // 2
    top_wall = center_y - warehouse_size_y // 2
    bottom_wall = center_y + warehouse_size_y // 2
    
    # Draw walls (thicker and more pronounced)
    grid[top_wall:top_wall+wall_thickness, left_wall:right_wall] = 100  # Top wall
    grid[bottom_wall-wall_thickness:bottom_wall, left_wall:right_wall] = 100  # Bottom wall
    grid[top_wall:bottom_wall, left_wall:left_wall+wall_thickness] = 100  # Left wall
    grid[top_wall:bottom_wall, right_wall-wall_thickness:right_wall] = 100  # Right wall
    
    # Add interior features and obstacles
    # Create a warehouse layout with aisles
    aisle_width = 60  # 3m aisles
    
    # Horizontal aisles (3 aisles)
    h_aisle_positions = [
        center_y - warehouse_size_y//4,
        center_y,
        center_y + warehouse_size_y//4
    ]
    
    # Vertical aisles (3 aisles)
    v_aisle_positions = [
        center_x - warehouse_size_x//4,
        center_x,
        center_x + warehouse_size_x//4
    ]
    
    # Place shelves between aisles
    shelf_depth = 30  # 1.5m deep shelves
    
    for i in range(len(h_aisle_positions) - 1):
        for j in range(len(v_aisle_positions) - 1):
            # Top left corner of this "cell" between aisles
            cell_top = h_aisle_positions[i] + aisle_width//2
            cell_left = v_aisle_positions[j] + aisle_width//2
            
            # Bottom right corner
            cell_bottom = h_aisle_positions[i+1] - aisle_width//2
            cell_right = v_aisle_positions[j+1] - aisle_width//2
            
            # Fill with shelves or obstacles
            if (i + j) % 2 == 0:  # Checkerboard pattern
                # Add horizontal shelves
                for y in range(cell_top, cell_bottom, shelf_depth*2):
                    if y + shelf_depth < cell_bottom:
                        grid[y:y+shelf_depth, cell_left:cell_right] = 100
            else:
                # Add vertical shelves
                for x in range(cell_left, cell_right, shelf_depth*2):
                    if x + shelf_depth < cell_right:
                        grid[cell_top:cell_bottom, x:x+shelf_depth] = 100
    
    # Now place specific objects from the SDF
    for model in models:
        if model['type'] == 'ground' or model['type'] == 'warehouse':
            continue  # Skip ground plane and warehouse base
        
        x, y = model['pose'][0], model['pose'][1]
        
        # Convert world coordinates to grid coordinates
        grid_x = int(center_x + x / resolution)
        grid_y = int(center_y + y / resolution)
        
        # Ensure coordinates are within grid bounds
        if 0 <= grid_x < width and 0 <= grid_y < height:
            # Create a different shape for each model type
            if model['type'] == 'shelf':
                size = 40  # 2m shelf
                shape = np.ones((size, size)) * 100
            elif model['type'] == 'barrier':
                shape = np.ones((50, 20)) * 100  # Rectangular barrier
            elif model['type'] == 'chair':
                size = 15
                shape = np.ones((size, size)) * 100
            elif model['type'] == 'table':
                shape = np.ones((40, 40)) * 100
            elif model['type'] == 'person':
                shape = np.ones((10, 10)) * 100
            else:
                size = 15
                shape = np.ones((size, size)) * 100
                
            # Calculate placement bounds
            shape_h, shape_w = shape.shape
            x_min = max(0, grid_x - shape_w//2)
            x_max = min(width, grid_x + shape_w//2)
            y_min = max(0, grid_y - shape_h//2)
            y_max = min(height, grid_y + shape_h//2)
            
            # Adjust shape if needed
            shape_h_actual = y_max - y_min
            shape_w_actual = x_max - x_min
            
            if shape_h_actual > 0 and shape_w_actual > 0:
                # Place the shape on the grid
                grid[y_min:y_max, x_min:x_max] = 100
    
    return grid

def save_map(grid, output_dir, map_name, resolution):
    """Save the occupancy grid as a PGM file with YAML metadata"""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Invert the grid for PGM format where 0 is black (occupied) and 255 is white (free)
    pgm_data = np.zeros_like(grid, dtype=np.uint8)
    pgm_data[grid == 0] = 254    # Free space is white (254 for better visualization)
    pgm_data[grid == 100] = 0    # Occupied space is black (0)
    
    # Save as PGM file (P5 format - raw binary format)
    pgm_path = os.path.join(output_dir, f"{map_name}.pgm")
    with open(pgm_path, 'wb') as f:
        # Write PGM header
        f.write(f"P5\n{pgm_data.shape[1]} {pgm_data.shape[0]}\n255\n".encode())
        # Write binary data
        f.write(pgm_data.tobytes())
    
    # Create YAML metadata file
    yaml_path = os.path.join(output_dir, f"{map_name}.yaml")
    with open(yaml_path, 'w') as yaml_file:
        yaml_content = {
            'image': f"{map_name}.pgm",
            'resolution': resolution,
            'origin': [-25, -25, 0.0],  # [x, y, yaw]
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        yaml.dump(yaml_content, yaml_file, default_flow_style=False)
    
    print(f"Map saved to {pgm_path} with metadata at {yaml_path}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python sdf_to_map.py <sdf_file> [output_dir] [map_name]")
        sys.exit(1)
    
    sdf_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "."
    map_name = sys.argv[3] if len(sys.argv) > 3 else "map"
    
    # Parse SDF file
    print(f"Parsing SDF file: {sdf_file}")
    models = parse_sdf(sdf_file)
    print(f"Found {len(models)} models")
    
    # Create occupancy grid
    resolution = 0.05  # 5cm per cell
    width = 1000  # 50m width (1000 * 0.05)
    height = 1000  # 50m height (1000 * 0.05)
    
    print("Creating occupancy grid...")
    grid = create_occupancy_grid(models, width, height, resolution)
    
    # Save the map
    print(f"Saving map to {output_dir}/{map_name}.*")
    save_map(grid, output_dir, map_name, resolution)
    
    print("Done!")

if __name__ == "__main__":
    main()