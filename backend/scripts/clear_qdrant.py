#!/usr/bin/env python3
"""
Script to clear the Qdrant collection before re-processing book data.
"""

import sys
import os
from pathlib import Path

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.vector_store import VectorStoreService
from src.config.settings import settings
from qdrant_client.http import models


def clear_qdrant_collection():
    """Clear the Qdrant collection."""
    print("Clearing Qdrant collection...")

    # Initialize the vector store service
    vector_store = VectorStoreService()

    # Get all vector IDs in the collection
    count = vector_store.get_vector_count()
    print(f"Found {count} vectors in collection")

    if count == 0:
        print("Collection is already empty.")
        return

    print(f"Deleting collection: {vector_store.collection_name}")

    # Delete and recreate the collection
    try:
        vector_store.client.delete_collection(vector_store.collection_name)
        print(f"Deleted collection: {vector_store.collection_name}")
    except Exception as e:
        print(f"Collection might not have existed: {e}")

    # Recreate collection with proper configuration
    vector_store.client.create_collection(
        collection_name=vector_store.collection_name,
        vectors_config=models.VectorParams(
            size=settings.vector_size,  # Dimension of the embeddings
            distance=models.Distance.COSINE  # Cosine distance for similarity search
        )
    )

    new_count = vector_store.get_vector_count()
    print(f"New collection created with {new_count} vectors")


if __name__ == "__main__":
    clear_qdrant_collection()