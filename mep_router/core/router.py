"""Router module implementing modified A* algorithm for MEP routing.

This module implements a modified A* pathfinding algorithm tailored for MEP systems,
incorporating bend penalties and clearance requirements.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import networkx as nx
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

@dataclass
class RouterConfig:
    """Configuration for MEP routing."""
    max_bends: int = 5              # maximum number of bends allowed
    min_bend_radius: float = 0.5    # minimum radius for bends
    max_segment_length: float = 10.0 # maximum length of straight segments
    bend_penalty: float = 1.5       # penalty multiplier for bends
    clearance_penalty: float = 2.0   # penalty multiplier for tight spaces
    smoothing_factor: float = 0.1   # factor for path smoothing (0-1)

class MEPRouter:
    """Implements modified A* routing for MEP systems."""
    
    def __init__(self, config: Optional[RouterConfig] = None):
        """Initialize the router with optional configuration.
        
        Args:
            config: Configuration for routing. If None, uses defaults.
        """
        self.config = config or RouterConfig()
        
    def _heuristic(self, node1: Point, node2: Point, 
                  obstacles: List[Polygon]) -> float:
        """Calculate heuristic cost between two nodes.
        
        Args:
            node1: Start node
            node2: End node
            obstacles: List of obstacle polygons
            
        Returns:
            Estimated cost between nodes
        """
        # Base cost is Euclidean distance
        base_cost = node1.distance(node2)
        
        # Add clearance penalty
        line = LineString([node1, node2])
        min_clearance = min(line.distance(obs) for obs in obstacles)
        if min_clearance < self.config.min_bend_radius:
            base_cost *= self.config.clearance_penalty
            
        return base_cost
    
    def _count_bends(self, path: List[Point]) -> int:
        """Count the number of bends in a path.
        
        Args:
            path: List of points forming the path
            
        Returns:
            Number of bends
        """
        if len(path) < 3:
            return 0
            
        bends = 0
        for i in range(1, len(path) - 1):
            v1 = np.array([path[i].x - path[i-1].x, path[i].y - path[i-1].y])
            v2 = np.array([path[i+1].x - path[i].x, path[i+1].y - path[i].y])
            angle = np.arccos(np.dot(v1, v2) / 
                            (np.linalg.norm(v1) * np.linalg.norm(v2)))
            if angle < np.pi * 0.75:  # Less than 135 degrees
                bends += 1
                
        return bends
    
    def _smooth_path(self, path: List[Point]) -> List[Point]:
        """Smooth a path using moving average.
        
        Args:
            path: List of points forming the path
            
        Returns:
            Smoothed path
        """
        if len(path) < 3:
            return path
            
        smoothed = [path[0]]  # Keep start point
        for i in range(1, len(path) - 1):
            prev = path[i-1]
            curr = path[i]
            next_pt = path[i+1]
            
            # Calculate smoothed point
            smooth_x = (prev.x + curr.x + next_pt.x) / 3
            smooth_y = (prev.y + curr.y + next_pt.y) / 3
            
            # Interpolate between original and smoothed point
            new_x = curr.x + (smooth_x - curr.x) * self.config.smoothing_factor
            new_y = curr.y + (smooth_y - curr.y) * self.config.smoothing_factor
            
            smoothed.append(Point(new_x, new_y))
            
        smoothed.append(path[-1])  # Keep end point
        return smoothed
    
    def _enforce_constraints(self, path: List[Point], 
                           obstacles: List[Polygon]) -> List[Point]:
        """Enforce MEP routing constraints on a path.
        
        Args:
            path: List of points forming the path
            obstacles: List of obstacle polygons
            
        Returns:
            Modified path satisfying constraints
        """
        if len(path) < 3:
            return path
            
        # Split long segments
        new_path = [path[0]]
        for i in range(len(path) - 1):
            segment = LineString([path[i], path[i+1]])
            if segment.length > self.config.max_segment_length:
                # Add intermediate points
                num_points = int(segment.length / self.config.max_segment_length) + 1
                for j in range(1, num_points):
                    t = j / num_points
                    new_point = Point(
                        path[i].x + t * (path[i+1].x - path[i].x),
                        path[i].y + t * (path[i+1].y - path[i].y)
                    )
                    new_path.append(new_point)
            new_path.append(path[i+1])
            
        # Smooth path while maintaining clearance
        smoothed = self._smooth_path(new_path)
        
        # Verify clearance after smoothing
        for i in range(len(smoothed) - 1):
            segment = LineString([smoothed[i], smoothed[i+1]])
            if any(segment.distance(obs) < self.config.min_bend_radius 
                  for obs in obstacles):
                # Revert to original point if clearance is violated
                smoothed[i] = new_path[i]
                
        return smoothed
    
    def find_path(self, G: nx.Graph, start: Point, end: Point,
                 obstacles: List[Polygon]) -> Optional[List[Point]]:
        """Find an optimal path between two points using modified A*.
        
        Args:
            G: NetworkX graph for routing
            start: Start point
            end: End point
            obstacles: List of obstacle polygons
            
        Returns:
            List of points forming the path, or None if no path exists
        """
        # Find nearest graph nodes to start and end points
        start_node = min(G.nodes(), key=lambda n: n.distance(start))
        end_node = min(G.nodes(), key=lambda n: n.distance(end))
        
        # Define heuristic function for A*
        def heuristic(n1: Point, n2: Point) -> float:
            return self._heuristic(n1, n2, obstacles)
        
        try:
            # Find path using A*
            path = nx.astar_path(G, start_node, end_node, 
                                heuristic=heuristic, weight='weight')
            
            # Add actual start and end points
            full_path = [start] + path + [end]
            
            # Count bends and check constraints
            if self._count_bends(full_path) > self.config.max_bends:
                return None
                
            # Enforce MEP constraints
            constrained_path = self._enforce_constraints(full_path, obstacles)
            
            return constrained_path
            
        except nx.NetworkXNoPath:
            return None
    
    def find_multiple_paths(self, G: nx.Graph, 
                           endpoints: List[Tuple[Point, Point]],
                           obstacles: List[Polygon]) -> List[List[Point]]:
        """Find multiple paths between endpoint pairs.
        
        Args:
            G: NetworkX graph for routing
            endpoints: List of (start, end) point pairs
            obstacles: List of obstacle polygons
            
        Returns:
            List of paths, where each path is a list of points
        """
        paths = []
        for start, end in endpoints:
            path = self.find_path(G, start, end, obstacles)
            if path is not None:
                paths.append(path)
        return paths
    
    def optimize_paths(self, paths: List[List[Point]], 
                      obstacles: List[Polygon]) -> List[List[Point]]:
        """Optimize a set of paths to minimize conflicts.
        
        Args:
            paths: List of paths to optimize
            obstacles: List of obstacle polygons
            
        Returns:
            List of optimized paths
        """
        if not paths:
            return paths
            
        # Create buffer around existing paths
        path_buffers = []
        for path in paths:
            path_line = LineString(path)
            path_buffers.append(path_line.buffer(self.config.min_bend_radius))
            
        # Add path buffers to obstacles for subsequent paths
        current_obstacles = obstacles.copy()
        optimized_paths = []
        
        for i, path in enumerate(paths):
            # Add buffers of previous paths to obstacles
            if i > 0:
                current_obstacles.extend(path_buffers[:i])
                
            # Re-route path with updated obstacles
            if len(path) >= 2:
                G = nx.Graph()  # Create new graph for this path
                # Add nodes and edges (simplified for example)
                for point in path:
                    G.add_node(point, pos=(point.x, point.y))
                for j in range(len(path) - 1):
                    G.add_edge(path[j], path[j+1])
                    
                new_path = self.find_path(G, path[0], path[-1], current_obstacles)
                if new_path is not None:
                    optimized_paths.append(new_path)
                    # Update path buffer
                    path_buffers[i] = LineString(new_path).buffer(
                        self.config.min_bend_radius)
                else:
                    optimized_paths.append(path)
            else:
                optimized_paths.append(path)
                
        return optimized_paths 