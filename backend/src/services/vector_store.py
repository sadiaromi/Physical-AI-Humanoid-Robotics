from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from ..config.settings import settings
import uuid


class VectorStoreService:
    def __init__(self):
        # Initialize Qdrant client with connection settings
        self.client = QdrantClient(
            url=settings.qdrant_host,
            api_key=settings.qdrant_api_key,
            port=settings.qdrant_port,
            grpc_port=6334,
            prefer_grpc=True
        )

        # Collection name for book content
        self.collection_name = "book_content_chunks"

        # Initialize the collection if it doesn't exist
        self._init_collection()

    def _init_collection(self):
        """
        Initialize the Qdrant collection if it doesn't exist
        """
        try:
            # Try to get collection info to see if it exists
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=settings.vector_size,  # Dimension of the embeddings
                    distance=models.Distance.COSINE  # Cosine distance for similarity search
                )
            )

    def add_vectors(self, vectors: List[List[float]], payloads: List[Dict[str, Any]], vector_ids: Optional[List[str]] = None):
        """
        Add vectors to the collection

        Args:
            vectors: List of embedding vectors to add
            payloads: List of metadata associated with each vector
            vector_ids: Optional list of IDs for the vectors (if not provided, UUIDs will be generated)
        """
        if vector_ids is None:
            # Generate UUIDs if not provided
            vector_ids = [str(uuid.uuid4()) for _ in range(len(vectors))]

        # Ensure the lengths match
        assert len(vectors) == len(payloads) == len(vector_ids), "Vectors, payloads, and IDs must have the same length"

        # Upload points (vectors with metadata) to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=vid,
                    vector=vec,
                    payload=payload
                )
                for vid, vec, payload in zip(vector_ids, vectors, payloads)
            ]
        )

    def search(self, query_vector: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar vectors to the query vector

        Args:
            query_vector: The embedding vector to search for similar ones
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing the payload data from matching vectors
        """
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit
        )

        # Extract payload data from search results
        return [hit.payload for hit in results]

    def delete_vectors(self, vector_ids: List[str]):
        """
        Delete vectors by their IDs

        Args:
            vector_ids: List of vector IDs to delete
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=vector_ids
            )
        )

    def get_vector_count(self) -> int:
        """
        Get the total number of vectors in the collection

        Returns:
            Number of vectors in the collection
        """
        return self.client.count(
            collection_name=self.collection_name
        ).count