"""DXF parsing and geometry conversion module.

This module handles the ingestion of DXF files and conversion of DXF entities
to Shapely geometries for spatial analysis.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import ezdxf
from ezdxf.document import Drawing
from ezdxf.entities import DXFEntity, Line, LWPolyline
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

@dataclass
class LayerConfig:
    """Configuration for DXF layer processing."""
    wall_layers: Set[str] = frozenset({"WALLS", "WALL"})
    door_layers: Set[str] = frozenset({"DOORS", "DOOR"})
    equipment_layers: Set[str] = frozenset({"EQUIPMENT", "EQ"})
    ignore_layers: Set[str] = frozenset({"DEFPOINTS", "0"})

class DXFGeometryConverter:
    """Converts DXF entities to Shapely geometries."""
    
    def __init__(self, layer_config: Optional[LayerConfig] = None):
        """Initialize the converter with optional layer configuration.
        
        Args:
            layer_config: Configuration for layer processing. If None, uses defaults.
        """
        self.layer_config = layer_config or LayerConfig()
        
    def _convert_line(self, entity: Line) -> LineString:
        """Convert an ezdxf Line to a Shapely LineString."""
        return LineString([(entity.dxf.start.x, entity.dxf.start.y),
                          (entity.dxf.end.x, entity.dxf.end.y)])
    
    def _convert_lwpolyline(self, entity: LWPolyline) -> LineString:
        """Convert an ezdxf LWPolyline to a Shapely LineString or Polygon."""
        points = [(point[0], point[1]) for point in entity.get_points()]
        if entity.closed:
            return Polygon(points)
        return LineString(points)
    
    def _convert_entity(self, entity: DXFEntity) -> Optional[LineString | Polygon]:
        """Convert a DXF entity to a Shapely geometry based on its type."""
        if isinstance(entity, Line):
            return self._convert_line(entity)
        elif isinstance(entity, LWPolyline):
            return self._convert_lwpolyline(entity)
        return None

class DXFReader:
    """Handles DXF file reading and initial processing."""
    
    def __init__(self, layer_config: Optional[LayerConfig] = None):
        """Initialize the reader with optional layer configuration.
        
        Args:
            layer_config: Configuration for layer processing. If None, uses defaults.
        """
        self.layer_config = layer_config or LayerConfig()
        self.converter = DXFGeometryConverter(layer_config)
        
    def read_file(self, file_path: Path) -> Tuple[Drawing, Dict[str, List[LineString | Polygon]]]:
        """Read a DXF file and convert its entities to Shapely geometries.
        
        Args:
            file_path: Path to the DXF file.
            
        Returns:
            Tuple containing:
            - The ezdxf Drawing object
            - Dictionary mapping layer names to lists of Shapely geometries
            
        Raises:
            ezdxf.DXFError: If the DXF file is invalid or cannot be read.
        """
        try:
            doc = ezdxf.readfile(str(file_path))
        except ezdxf.DXFError as e:
            raise ezdxf.DXFError(f"Failed to read DXF file: {e}")
            
        geometries: Dict[str, List[LineString | Polygon]] = {}
        
        for entity in doc.modelspace():
            if entity.dxf.layer in self.layer_config.ignore_layers:
                continue
                
            geometry = self.converter._convert_entity(entity)
            if geometry is not None:
                layer = entity.dxf.layer
                if layer not in geometries:
                    geometries[layer] = []
                geometries[layer].append(geometry)
                
        return doc, geometries
    
    def get_wall_geometry(self, geometries: Dict[str, List[LineString | Polygon]]) -> Polygon:
        """Extract and union all wall geometries.
        
        Args:
            geometries: Dictionary of layer geometries from read_file()
            
        Returns:
            Unioned Polygon of all wall geometries
        """
        wall_geoms = []
        for layer in self.layer_config.wall_layers:
            if layer in geometries:
                wall_geoms.extend(geometries[layer])
                
        if not wall_geoms:
            raise ValueError("No wall geometries found in the specified layers")
            
        return unary_union(wall_geoms)
    
    def get_door_geometry(self, geometries: Dict[str, List[LineString | Polygon]]) -> List[LineString | Polygon]:
        """Extract all door geometries.
        
        Args:
            geometries: Dictionary of layer geometries from read_file()
            
        Returns:
            List of door geometries
        """
        door_geoms = []
        for layer in self.layer_config.door_layers:
            if layer in geometries:
                door_geoms.extend(geometries[layer])
        return door_geoms
    
    def get_equipment_geometry(self, geometries: Dict[str, List[LineString | Polygon]]) -> List[LineString | Polygon]:
        """Extract all equipment geometries.
        
        Args:
            geometries: Dictionary of layer geometries from read_file()
            
        Returns:
            List of equipment geometries
        """
        equipment_geoms = []
        for layer in self.layer_config.equipment_layers:
            if layer in geometries:
                equipment_geoms.extend(geometries[layer])
        return equipment_geoms 