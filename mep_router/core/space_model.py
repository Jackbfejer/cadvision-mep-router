"""Space modeling module for MEP routing.

This module handles the conversion of architectural geometries into a space model
suitable for routing, using both raster and vector-based approaches.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

@dataclass
class SpaceModelConfig:
    """Configuration for space modeling."""
    raster_resolution: float = 100.0  # pixels per unit
    clearance_distance: float = 0.5   # minimum clearance from obstacles
    min_room_size: float = 2.0        # minimum room size to consider
    threshold_value: int = 127        # binary threshold value (0-255)

class SpaceModeler:
    """Handles space modeling for MEP routing."""
    
    def __init__(self, config: Optional[SpaceModelConfig] = None):
        """Initialize the space modeler with optional configuration.
        
        Args:
            config: Configuration for space modeling. If None, uses defaults.
        """
        self.config = config or SpaceModelConfig()
        
    def _get_bounds(self, geometries: List[LineString | Polygon]) -> Tuple[float, float, float, float]:
        """Calculate the bounding box of all geometries.
        
        Args:
            geometries: List of Shapely geometries
            
        Returns:
            Tuple of (minx, miny, maxx, maxy)
        """
        bounds = [geom.bounds for geom in geometries]
        minx = min(b[0] for b in bounds)
        miny = min(b[1] for b in bounds)
        maxx = max(b[2] for b in bounds)
        maxy = max(b[3] for b in bounds)
        return minx, miny, maxx, maxy
        
    def rasterize_geometries(self, 
                           wall_geometry: Polygon,
                           door_geometries: List[LineString | Polygon],
                           equipment_geometries: List[LineString | Polygon]) -> np.ndarray:
        """Convert geometries to a binary occupancy grid.
        
        Args:
            wall_geometry: Unioned wall polygon
            door_geometries: List of door geometries
            equipment_geometries: List of equipment geometries
            
        Returns:
            Binary numpy array where 0 represents free space and 1 represents obstacles
        """
        # Get bounds and calculate image dimensions
        minx, miny, maxx, maxy = self._get_bounds([wall_geometry] + door_geometries + equipment_geometries)
        width = int((maxx - minx) * self.config.raster_resolution)
        height = int((maxy - miny) * self.config.raster_resolution)
        
        # Create blank image
        image = np.zeros((height, width), dtype=np.uint8)
        
        # Helper function to convert world coordinates to image coordinates
        def world_to_image(x: float, y: float) -> Tuple[int, int]:
            img_x = int((x - minx) * self.config.raster_resolution)
            img_y = int((maxy - y) * self.config.raster_resolution)  # Flip Y axis
            return img_x, img_y
        
        # Draw walls
        if isinstance(wall_geometry, Polygon):
            exterior = wall_geometry.exterior
            points = [world_to_image(x, y) for x, y in exterior.coords]
            cv2.fillPoly(image, [np.array(points)], 255)
            
            # Fill holes (interiors)
            for interior in wall_geometry.interiors:
                points = [world_to_image(x, y) for x, y in interior.coords]
                cv2.fillPoly(image, [np.array(points)], 0)
        
        # Draw equipment
        for geom in equipment_geometries:
            if isinstance(geom, Polygon):
                points = [world_to_image(x, y) for x, y in geom.exterior.coords]
                cv2.fillPoly(image, [np.array(points)], 255)
            elif isinstance(geom, LineString):
                points = [world_to_image(x, y) for x, y in geom.coords]
                cv2.polylines(image, [np.array(points)], False, 255, 2)
        
        # Apply threshold to get binary image
        _, binary = cv2.threshold(image, self.config.threshold_value, 1, cv2.THRESH_BINARY)
        
        # Apply clearance distance using morphological operations
        kernel_size = int(self.config.clearance_distance * self.config.raster_resolution)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        binary = cv2.dilate(binary, kernel, iterations=1)
        
        return binary
    
    def vectorize_space(self,
                       wall_geometry: Polygon,
                       door_geometries: List[LineString | Polygon],
                       equipment_geometries: List[LineString | Polygon]) -> List[Polygon]:
        """Create a vector-based space model using Shapely operations.
        
        Args:
            wall_geometry: Unioned wall polygon
            door_geometries: List of door geometries
            equipment_geometries: List of equipment geometries
            
        Returns:
            List of free space polygons
        """
        # Create buffer around walls and equipment for clearance
        obstacles = [wall_geometry]
        obstacles.extend(geom.buffer(self.config.clearance_distance) 
                        for geom in equipment_geometries)
        
        # Union all obstacles
        obstacle_union = unary_union(obstacles)
        
        # Get the bounding box of the entire space
        bounds = obstacle_union.bounds
        bbox = Polygon([
            (bounds[0], bounds[1]),
            (bounds[2], bounds[1]),
            (bounds[2], bounds[3]),
            (bounds[0], bounds[3])
        ])
        
        # Subtract obstacles from bounding box to get free space
        free_space = bbox.difference(obstacle_union)
        
        # Split into separate polygons if multipolygon
        if free_space.geom_type == 'MultiPolygon':
            spaces = list(free_space.geoms)
        else:
            spaces = [free_space]
            
        # Filter out spaces that are too small
        spaces = [space for space in spaces 
                 if space.area >= self.config.min_room_size ** 2]
        
        return spaces
    
    def find_connection_points(self,
                             space_polygons: List[Polygon],
                             door_geometries: List[LineString | Polygon]) -> List[Point]:
        """Find potential connection points between spaces.
        
        Args:
            space_polygons: List of free space polygons
            door_geometries: List of door geometries
            
        Returns:
            List of connection points (typically near doors)
        """
        connection_points = []
        
        for door in door_geometries:
            if isinstance(door, LineString):
                # Use midpoint of door as connection point
                midpoint = door.interpolate(0.5, normalized=True)
                connection_points.append(midpoint)
            elif isinstance(door, Polygon):
                # Use centroid of door polygon
                connection_points.append(door.centroid)
                
        return connection_points 