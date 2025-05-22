# CADVision MEP Router

An automated Mechanical, Electrical, and Plumbing (MEP) routing system that leverages modern Python libraries to generate optimal routing paths in architectural drawings.

## Features

- DXF file parsing and processing using ezdxf
- Space modeling with OpenCV and Shapely
- Graph-based routing using NetworkX
- Modified A* pathfinding with MEP-specific constraints
- FastAPI-based REST API
- Asynchronous processing with Celery
- Containerized deployment with Docker

## Prerequisites

- Python 3.9 or higher
- Poetry for dependency management
- Docker and Docker Compose (for containerized deployment)

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/cadvision-mep-router.git
cd cadvision-mep-router
```

2. Install dependencies using Poetry:
```bash
poetry install
```

3. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your configuration
```

## Development

1. Activate the virtual environment:
```bash
poetry shell
```

2. Run the development server:
```bash
uvicorn mep_router.api.main:app --reload
```

3. Run tests:
```bash
pytest
```

## Project Structure

```
mep_router/
├── api/                 # FastAPI application
│   ├── main.py         # API routes and configuration
│   └── schemas.py      # Pydantic models
├── core/               # Core business logic
│   ├── parser.py       # DXF parsing and geometry conversion
│   ├── space_model.py  # Space modeling and rasterization
│   ├── graph.py        # Graph construction and management
│   ├── router.py       # Routing algorithms
│   └── annotator.py    # DXF annotation and export
├── workers/            # Celery workers
│   └── tasks.py        # Background task definitions
└── tests/              # Test suite
    ├── conftest.py     # Test fixtures
    └── test_*.py       # Test modules
```

## API Usage

### Upload DXF and Request Routing

```bash
curl -X POST http://localhost:8000/upload \
  -F "file=@floor_plan.dxf" \
  -F "config=@routing_config.json"
```

### Check Job Status

```bash
curl http://localhost:8000/status/{job_id}
```

### Download Results

```bash
curl http://localhost:8000/download/{job_id} --output result.zip
```

## Docker Deployment

1. Build and start the containers:
```bash
docker-compose up --build
```

2. Access the API at `http://localhost:8000`
3. View API documentation at `http://localhost:8000/docs`

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Research by Choi et al. (2022) for the modified A* algorithm
- Survey by Blokland et al. (2023) for APR concept modeling
- Open-source libraries: ezdxf, Shapely, NetworkX, OpenCV 