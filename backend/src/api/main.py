from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api.routes import query, ingest
from src.config.settings import settings
from src.utils.helpers import setup_logging
import logging



# Set up logging
setup_logging()
logger = logging.getLogger(__name__)

# Create the FastAPI application
app = FastAPI(
    title=settings.app_name,
    description="API for querying book content using Retrieval Augmented Generation",
    version=settings.version,
    debug=settings.debug
)

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(query.router, prefix="/api/v1", tags=["Query"])
app.include_router(ingest.router, prefix="/api/v1", tags=["Books"])

import datetime

# Add a simple health check endpoint
@app.get("/health")
def health_check():
    """
    Simple health check endpoint
    """
    return {
        "status": "healthy",
        "version": settings.version,
        "timestamp": datetime.datetime.utcnow().isoformat()
    }


# Add root endpoint
@app.get("/")
def root():
    """
    Root endpoint with basic information
    """
    return {
        "app": settings.app_name,
        "version": settings.version,
        "description": "RAG Chatbot API for book content querying"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.api.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )