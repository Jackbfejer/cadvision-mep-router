"""Graph construction module for MEP routing.

This module handles the creation of routing networks from space models,
supporting both raster and vector-based approaches.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import networkx as nx
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

@dataclass
class GraphConfig:
    """Configuration for graph construction."""
    node_spacing: float = 1.0          # minimum distance between nodes
    max_edge_length: float = 5.0       # maximum edge length
    bend_penalty: float = 1.5          # penalty multiplier for bends
    clearance_weight: float = 2.0      # weight multiplier for edges near obstacles
    min_clearance: float = 0.5         # minimum clearance from obstacles

class GraphBuilder:
    """Builds routing networks from space models."""
    
    def __init__(self, config: Optional[GraphConfig] = None):
        """Initialize the graph builder with optional configuration.
        
        Args:
            config: Configuration for graph construction. If None, uses defaults.
        """
        self.config = config or GraphConfig()
        
    def _create_grid_nodes(self, binary_grid: np.ndarray, 
                          transform: Tuple[float, float, float, float]) -> List[Point]:
        """Create nodes from a binary occupancy grid.
        
        Args:
            binary_grid: Binary numpy array (0=free, 1=obstacle)
            transform: Tuple of (minx, miny, maxx, maxy) for coordinate conversion
            
        Returns:
            List of node points in world coordinates
        """
        minx, miny, maxx, maxy = transform
        height, width = binary_grid.shape
        nodes = []
        
        # Convert image coordinates to world coordinates
        def image_to_world(img_x: int, img_y: int) -> Tuple[float, float]:
            world_x = minx + (img_x / width) * (maxx - minx)
            world_y = maxy - (img_y / height) * (maxy - miny)  # Flip Y axis
            return world_x, world_y
        
        # Create nodes at free spaces
        for y in range(0, height, int(self.config.node_spacing * width / (maxx - minx))):
            for x in range(0, width, int(self.config.node_spacing * width / (maxx - minx))):
                if binary_grid[y, x] == 0:  # Free space
                    world_x, world_y = image_to_world(x, y)
                    nodes.append(Point(world_x, world_y))
                    
        return nodes
    
    def _create_vector_nodes(self, space_polygons: List[Polygon],
                           connection_points: List[Point]) -> List[Point]:
        """Create nodes from vector space model.
        
        Args:
            space_polygons: List of free space polygons
            connection_points: List of connection points (e.g., near doors)
            
        Returns:
            List of node points
        """
        nodes = []
        
        # Add connection points
        nodes.extend(connection_points)
        
        # Add grid points within each space polygon
        for space in space_polygons:
            minx, miny, maxx, maxy = space.bounds
            x_coords = np.arange(minx, maxx, self.config.node_spacing)
            y_coords = np.arange(miny, maxy, self.config.node_spacing)
            
            for x in x_coords:
                for y in y_coords:
                    point = Point(x, y)
                    if space.contains(point):
                        nodes.append(point)
                        
        return nodes
    
    def _create_edges(self, nodes: List[Point], 
                     obstacles: List[Polygon]) -> List[Tuple[Point, Point, float]]:
        """Create edges between nodes that don't intersect obstacles.
        
        Args:
            nodes: List of node points
            obstacles: List of obstacle polygons
            
        Returns:
            List of (node1, node2, weight) tuples
        """
        edges = []
        strtree = STRtree(obstacles)
        
        # Create edges between nodes that are close enough
        for i, node1 in enumerate(nodes):
            for node2 in nodes[i+1:]:
                # Skip if nodes are too far apart
                if node1.distance(node2) > self.config.max_edge_length:
                    continue
                    
                # Create line between nodes
                line = LineString([node1, node2])
                
                # Check for intersections with obstacles
                if not any(line.intersects(obs) for obs in strtree.query(line)):
                    # Calculate edge weight based on length and clearance
                    weight = node1.distance(node2)
                    
                    # Add clearance penalty
                    min_clearance = min(line.distance(obs) for obs in strtree.query(line))
                    if min_clearance < self.config.min_clearance:
                        weight *= self.config.clearance_weight
                        
                    edges.append((node1, node2, weight))
                    
        return edges
    
    def build_raster_graph(self, 
                          binary_grid: np.ndarray,
                          transform: Tuple[float, float, float, float],
                          obstacles: List[Polygon]) -> nx.Graph:
        """Build a routing graph from a raster space model.
        
        Args:
            binary_grid: Binary numpy array (0=free, 1=obstacle)
            transform: Tuple of (minx, miny, maxx, maxy) for coordinate conversion
            obstacles: List of obstacle polygons for clearance checking
            
        Returns:
            NetworkX graph for routing
        """
        # Create nodes
        nodes = self._create_grid_nodes(binary_grid, transform)
        
        # Create edges
        edges = self._create_edges(nodes, obstacles)
        
        # Build graph
        G = nx.Graph()
        for node in nodes:
            G.add_node(node, pos=(node.x, node.y))
        for node1, node2, weight in edges:
            G.add_edge(node1, node2, weight=weight)
            
        return G
    
    def build_vector_graph(self,
                          space_polygons: List[Polygon],
                          connection_points: List[Point],
                          obstacles: List[Polygon]) -> nx.Graph:
        """Build a routing graph from a vector space model.
        
        Args:
            space_polygons: List of free space polygons
            connection_points: List of connection points (e.g., near doors)
            obstacles: List of obstacle polygons for clearance checking
            
        Returns:
            NetworkX graph for routing
        """
        # Create nodes
        nodes = self._create_vector_nodes(space_polygons, connection_points)
        
        # Create edges
        edges = self._create_edges(nodes, obstacles)
        
        # Build graph
        G = nx.Graph()
        for node in nodes:
            G.add_node(node, pos=(node.x, node.y))
        for node1, node2, weight in edges:
            G.add_edge(node1, node2, weight=weight)
            
        return G
    
    def add_bend_penalties(self, G: nx.Graph) -> None:
        """Add penalties for edges that create sharp bends.
        
        Args:
            G: NetworkX graph to modify
        """
        for node in G.nodes():
            neighbors = list(G.neighbors(node))
            if len(neighbors) >= 2:
                for i, n1 in enumerate(neighbors):
                    for n2 in neighbors[i+1:]:
                        # Calculate angle between edges
                        v1 = np.array([n1.x - node.x, n1.y - node.y])
                        v2 = np.array([n2.x - node.x, n2.y - node.y])
                        angle = np.arccos(np.dot(v1, v2) / 
                                        (np.linalg.norm(v1) * np.linalg.norm(v2)))
                        
                        # Apply penalty for sharp angles
                        if angle < np.pi / 4:  # Less than 45 degrees
                            G[node][n1]['weight'] *= self.config.bend_penalty
                            G[node][n2]['weight'] *= self.config.bend_penalty 