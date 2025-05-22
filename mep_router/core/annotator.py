"""Annotator module for exporting MEP routes to DXF files.

This module handles the conversion of routing paths back to DXF entities
and the creation of annotated DXF files with proper layering and styling.
"""

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import ezdxf
from ezdxf.document import Drawing
from ezdxf.entities import LWPolyline
from shapely.geometry import LineString, Point

class MEPType(Enum):
    """Types of MEP systems."""
    MECHANICAL = "MEP_Mechanical"
    ELECTRICAL = "MEP_Electrical"
    PLUMBING = "MEP_Plumbing"

@dataclass
class RouteStyle:
    """Styling configuration for MEP routes."""
    color: int = 1  # AutoCAD color index
    lineweight: float = 0.5  # mm
    linetype: str = "CONTINUOUS"
    layer: str = "MEP_Routes"

@dataclass
class AnnotationConfig:
    """Configuration for DXF annotation."""
    route_styles: Dict[MEPType, RouteStyle] = None
    text_height: float = 0.1
    text_style: str = "Standard"
    dimension_style: str = "Standard"
    
    def __post_init__(self):
        """Initialize default route styles if not provided."""
        if self.route_styles is None:
            self.route_styles = {
                MEPType.MECHANICAL: RouteStyle(color=1, layer="MEP_Mechanical"),
                MEPType.ELECTRICAL: RouteStyle(color=3, layer="MEP_Electrical"),
                MEPType.PLUMBING: RouteStyle(color=5, layer="MEP_Plumbing")
            }

class DXFAnnotator:
    """Handles the creation of annotated DXF files with MEP routes."""
    
    def __init__(self, config: Optional[AnnotationConfig] = None):
        """Initialize the annotator with optional configuration.
        
        Args:
            config: Configuration for annotation. If None, uses defaults.
        """
        self.config = config or AnnotationConfig()
        
    def _create_layers(self, doc: Drawing) -> None:
        """Create necessary layers in the DXF document.
        
        Args:
            doc: ezdxf Drawing object
        """
        for mep_type in MEPType:
            style = self.config.route_styles[mep_type]
            if style.layer not in doc.layers:
                doc.layers.new(name=style.layer, 
                             dxfattribs={
                                 'color': style.color,
                                 'lineweight': int(style.lineweight * 100),  # Convert to 1/100mm
                                 'linetype': style.linetype
                             })
    
    def _add_route(self, doc: Drawing, path: List[Point], 
                  mep_type: MEPType) -> None:
        """Add a route to the DXF document.
        
        Args:
            doc: ezdxf Drawing object
            path: List of points forming the route
            mep_type: Type of MEP system
        """
        if len(path) < 2:
            return
            
        style = self.config.route_styles[mep_type]
        msp = doc.modelspace()
        
        # Convert path to points for LWPolyline
        points = [(point.x, point.y, 0) for point in path]
        
        # Create LWPolyline entity
        polyline = msp.add_lwpolyline(points)
        polyline.dxf.layer = style.layer
        
        # Add route information as extended data
        polyline.add_xdata("MEP_TYPE", [(1000, mep_type.value)])
        polyline.add_xdata("ROUTE_INFO", [
            (1000, f"Length: {LineString(path).length:.2f}"),
            (1000, f"Points: {len(path)}")
        ])
    
    def _add_dimensions(self, doc: Drawing, path: List[Point],
                       mep_type: MEPType) -> None:
        """Add dimensions to the route.
        
        Args:
            doc: ezdxf Drawing object
            path: List of points forming the route
            mep_type: Type of MEP system
        """
        if len(path) < 2:
            return
            
        msp = doc.modelspace()
        style = self.config.route_styles[mep_type]
        
        # Add length dimension for each segment
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i+1]
            
            # Calculate dimension position
            mid_x = (start.x + end.x) / 2
            mid_y = (start.y + end.y) / 2
            offset = 0.5  # Offset from route
            
            # Add aligned dimension
            dim = msp.add_aligned_dim(
                base=(start.x, start.y, 0),
                p1=(end.x, end.y, 0),
                p2=(mid_x + offset, mid_y + offset, 0),
                dimstyle=self.config.dimension_style
            )
            dim.dxf.layer = style.layer
    
    def _add_labels(self, doc: Drawing, path: List[Point],
                   mep_type: MEPType) -> None:
        """Add labels to the route.
        
        Args:
            doc: ezdxf Drawing object
            path: List of points forming the route
            mep_type: Type of MEP system
        """
        if len(path) < 2:
            return
            
        msp = doc.modelspace()
        style = self.config.route_styles[mep_type]
        
        # Add label at start and end points
        for i, point in enumerate([path[0], path[-1]]):
            label = f"{mep_type.value}_{i+1}"
            text = msp.add_text(
                label,
                dxfattribs={
                    'height': self.config.text_height,
                    'style': self.config.text_style,
                    'layer': style.layer
                }
            )
            text.set_pos((point.x, point.y, 0), align='CENTER')
    
    def create_annotated_dxf(self, 
                            template_path: Optional[Path],
                            routes: Dict[MEPType, List[List[Point]]],
                            output_path: Path) -> None:
        """Create an annotated DXF file with MEP routes.
        
        Args:
            template_path: Path to template DXF file (optional)
            routes: Dictionary mapping MEP types to lists of routes
            output_path: Path to save the annotated DXF file
            
        Raises:
            ezdxf.DXFError: If there are issues with DXF file handling
        """
        try:
            # Create or load DXF document
            if template_path and template_path.exists():
                doc = ezdxf.readfile(str(template_path))
            else:
                doc = ezdxf.new('R2010')  # Use AutoCAD 2010 format
                
            # Create layers
            self._create_layers(doc)
            
            # Add routes and annotations
            for mep_type, path_list in routes.items():
                for path in path_list:
                    self._add_route(doc, path, mep_type)
                    self._add_dimensions(doc, path, mep_type)
                    self._add_labels(doc, path, mep_type)
                    
            # Save the annotated file
            doc.saveas(str(output_path))
            
        except ezdxf.DXFError as e:
            raise ezdxf.DXFError(f"Failed to create annotated DXF: {e}")
    
    def create_route_summary(self, routes: Dict[MEPType, List[List[Point]]]) -> str:
        """Create a text summary of the routes.
        
        Args:
            routes: Dictionary mapping MEP types to lists of routes
            
        Returns:
            Formatted summary string
        """
        summary = ["MEP Route Summary", "=" * 20, ""]
        
        for mep_type, path_list in routes.items():
            summary.append(f"{mep_type.value}:")
            total_length = 0
            for i, path in enumerate(path_list, 1):
                length = LineString(path).length
                total_length += length
                summary.append(f"  Route {i}:")
                summary.append(f"    Length: {length:.2f}")
                summary.append(f"    Points: {len(path)}")
                summary.append(f"    Start: ({path[0].x:.2f}, {path[0].y:.2f})")
                summary.append(f"    End: ({path[-1].x:.2f}, {path[-1].y:.2f})")
            summary.append(f"  Total Length: {total_length:.2f}")
            summary.append("")
            
        return "\n".join(summary) 