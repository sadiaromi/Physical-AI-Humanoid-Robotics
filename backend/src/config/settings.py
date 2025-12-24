from pydantic_settings import BaseSettings
from typing import Optional
from pydantic import Field
import os


class Settings(BaseSettings):
    # API Configuration
    api_key: str = Field(..., alias="API_KEY")

    # Cohere Configuration
    cohere_api_key: str = Field(..., alias="COHERE_API_KEY")

    # Database Configuration
    neon_database_url: str = Field(..., alias="NEON_DATABASE_URL")

    # Qdrant Configuration
    qdrant_host: str = Field(..., alias="QDRANT_HOST")
    qdrant_api_key: str = Field(..., alias="QDRANT_API_KEY")
    qdrant_port: int = 6333  # Default Qdrant port

    # Application Configuration
    app_name: str = "RAG Chatbot API"
    debug: bool = False
    version: str = "1.0.0"

    # Rate Limiting Configuration
    rate_limit_requests: int = 100  # requests per hour
    rate_limit_window: int = 3600   # seconds

    # Book Content Configuration
    chunk_size: int = 600  # tokens per chunk
    chunk_overlap: int = 100  # token overlap between chunks
    max_query_length: int = 1000  # max length of user query in characters

    # Vector Configuration
    vector_size: int = 1024  # Size of embedding vectors

    class Config:
        env_file = ".env"
        populate_by_name = True

    @property
    def API_KEY(self):
        """Property to access api_key as API_KEY"""
        return self.api_key

    def model_post_init(self, __context):
        """Post initialization hook to clean up API key and other values"""
        # Clean up API key to remove potential whitespace or newlines
        if hasattr(self, 'api_key'):
            self.api_key = self.api_key.strip()


# Create a single instance of settings
settings = Settings()