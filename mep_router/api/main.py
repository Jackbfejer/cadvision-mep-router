"""FastAPI application for MEP routing system.

This module implements the REST API endpoints for the MEP routing system,
handling file uploads, routing requests, and result downloads.
"""

import json
import shutil
import uuid
from pathlib import Path
from typing import Dict, List, Optional

from fastapi import FastAPI, File, HTTPException, UploadFile, BackgroundTasks
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, Field

from mep_router.core.parser import DXFReader, LayerConfig
from mep_router.core.space_model import SpaceModeler, SpaceModelConfig
from mep_router.core.graph import GraphBuilder, GraphConfig
from mep_router.core.router import MEPRouter, RouterConfig
from mep_router.core.annotator import DXFAnnotator, MEPType, AnnotationConfig

# Create FastAPI app
app = FastAPI(
    title="MEP Router API",
    description="API for automated MEP routing in architectural drawings",
    version="0.1.0"
)

# Configure upload and output directories
UPLOAD_DIR = Path("uploads")
OUTPUT_DIR = Path("outputs")
UPLOAD_DIR.mkdir(exist_ok=True)
OUTPUT_DIR.mkdir(exist_ok=True)

# Store job status
job_status: Dict[str, Dict] = {}

class RoutingRequest(BaseModel):
    """Schema for routing request configuration."""
    mep_type: MEPType = Field(..., description="Type of MEP system")
    start_points: List[List[float]] = Field(..., description="List of [x, y] start points")
    end_points: List[List[float]] = Field(..., description="List of [x, y] end points")
    layer_config: Optional[LayerConfig] = Field(None, description="Layer configuration")
    space_config: Optional[SpaceModelConfig] = Field(None, description="Space modeling configuration")
    graph_config: Optional[GraphConfig] = Field(None, description="Graph construction configuration")
    router_config: Optional[RouterConfig] = Field(None, description="Routing configuration")
    annotation_config: Optional[AnnotationConfig] = Field(None, description="Annotation configuration")

def process_routing_job(job_id: str, dxf_path: Path, request: RoutingRequest) -> None:
    """Process a routing job in the background.
    
    Args:
        job_id: Unique job identifier
        dxf_path: Path to uploaded DXF file
        request: Routing request configuration
    """
    try:
        # Update job status
        job_status[job_id]["status"] = "processing"
        
        # Initialize components
        reader = DXFReader(request.layer_config)
        modeler = SpaceModeler(request.space_config)
        graph_builder = GraphBuilder(request.graph_config)
        router = MEPRouter(request.router_config)
        annotator = DXFAnnotator(request.annotation_config)
        
        # Read and parse DXF
        doc, geometries = reader.read_file(dxf_path)
        wall_geometry = reader.get_wall_geometry(geometries)
        door_geometries = reader.get_door_geometry(geometries)
        equipment_geometries = reader.get_equipment_geometry(geometries)
        
        # Create space model
        space_polygons = modeler.vectorize_space(
            wall_geometry, door_geometries, equipment_geometries)
        connection_points = modeler.find_connection_points(
            space_polygons, door_geometries)
        
        # Build routing graph
        G = graph_builder.build_vector_graph(
            space_polygons, connection_points, [wall_geometry] + equipment_geometries)
        graph_builder.add_bend_penalties(G)
        
        # Prepare endpoints
        endpoints = []
        for start, end in zip(request.start_points, request.end_points):
            start_point = Point(start[0], start[1])
            end_point = Point(end[0], end[1])
            endpoints.append((start_point, end_point))
        
        # Find routes
        paths = router.find_multiple_paths(G, endpoints, [wall_geometry] + equipment_geometries)
        optimized_paths = router.optimize_paths(paths, [wall_geometry] + equipment_geometries)
        
        # Create annotated DXF
        routes = {request.mep_type: optimized_paths}
        output_path = OUTPUT_DIR / f"{job_id}_routes.dxf"
        annotator.create_annotated_dxf(dxf_path, routes, output_path)
        
        # Create summary
        summary = annotator.create_route_summary(routes)
        summary_path = OUTPUT_DIR / f"{job_id}_summary.txt"
        summary_path.write_text(summary)
        
        # Update job status
        job_status[job_id].update({
            "status": "completed",
            "output_files": {
                "dxf": str(output_path),
                "summary": str(summary_path)
            }
        })
        
    except Exception as e:
        # Update job status with error
        job_status[job_id].update({
            "status": "failed",
            "error": str(e)
        })

@app.post("/upload")
async def upload_dxf(
    background_tasks: BackgroundTasks,
    file: UploadFile = File(...),
    config: UploadFile = File(...)
) -> JSONResponse:
    """Upload a DXF file and routing configuration.
    
    Args:
        background_tasks: FastAPI background tasks
        file: DXF file to process
        config: JSON configuration file
        
    Returns:
        JSON response with job ID
    """
    try:
        # Generate job ID
        job_id = str(uuid.uuid4())
        
        # Save uploaded files
        dxf_path = UPLOAD_DIR / f"{job_id}.dxf"
        config_path = UPLOAD_DIR / f"{job_id}_config.json"
        
        with dxf_path.open("wb") as f:
            shutil.copyfileobj(file.file, f)
        with config_path.open("wb") as f:
            shutil.copyfileobj(config.file, f)
            
        # Parse configuration
        config_data = json.loads(config_path.read_text())
        request = RoutingRequest(**config_data)
        
        # Initialize job status
        job_status[job_id] = {
            "status": "queued",
            "input_files": {
                "dxf": str(dxf_path),
                "config": str(config_path)
            }
        }
        
        # Start background processing
        background_tasks.add_task(process_routing_job, job_id, dxf_path, request)
        
        return JSONResponse({
            "job_id": job_id,
            "status": "queued",
            "message": "Routing job started"
        })
        
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@app.get("/status/{job_id}")
async def get_job_status(job_id: str) -> JSONResponse:
    """Get the status of a routing job.
    
    Args:
        job_id: Job identifier
        
    Returns:
        JSON response with job status
    """
    if job_id not in job_status:
        raise HTTPException(status_code=404, detail="Job not found")
        
    return JSONResponse(job_status[job_id])

@app.get("/download/{job_id}")
async def download_results(job_id: str) -> FileResponse:
    """Download the results of a completed routing job.
    
    Args:
        job_id: Job identifier
        
    Returns:
        ZIP file containing results
    """
    if job_id not in job_status:
        raise HTTPException(status_code=404, detail="Job not found")
        
    status = job_status[job_id]
    if status["status"] != "completed":
        raise HTTPException(
            status_code=400,
            detail=f"Job not completed (status: {status['status']})"
        )
        
    # Create ZIP file
    zip_path = OUTPUT_DIR / f"{job_id}_results.zip"
    with shutil.ZipFile(zip_path, "w") as zip_file:
        # Add DXF file
        dxf_path = Path(status["output_files"]["dxf"])
        zip_file.write(dxf_path, dxf_path.name)
        
        # Add summary file
        summary_path = Path(status["output_files"]["summary"])
        zip_file.write(summary_path, summary_path.name)
        
    return FileResponse(
        zip_path,
        media_type="application/zip",
        filename=f"mep_routes_{job_id}.zip"
    )

@app.delete("/jobs/{job_id}")
async def delete_job(job_id: str) -> JSONResponse:
    """Delete a routing job and its files.
    
    Args:
        job_id: Job identifier
        
    Returns:
        JSON response confirming deletion
    """
    if job_id not in job_status:
        raise HTTPException(status_code=404, detail="Job not found")
        
    try:
        # Delete input files
        for file_path in job_status[job_id]["input_files"].values():
            Path(file_path).unlink(missing_ok=True)
            
        # Delete output files
        if "output_files" in job_status[job_id]:
            for file_path in job_status[job_id]["output_files"].values():
                Path(file_path).unlink(missing_ok=True)
                
        # Delete ZIP file
        zip_path = OUTPUT_DIR / f"{job_id}_results.zip"
        zip_path.unlink(missing_ok=True)
        
        # Remove job status
        del job_status[job_id]
        
        return JSONResponse({
            "status": "success",
            "message": f"Job {job_id} deleted"
        })
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) 