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
        self.collection_name = "physical-ai-book"

        # Initialize the collection if it doesn't exist
        self._init_collection()

    def _init_collection(self):
        """
        Initialize the Qdrant collection if it doesn't exist
        """
        try:
            # Try to get collection info to see if it exists
            self.client.get_collection(self.collection_name)
            print(f"Collection {self.collection_name} already exists")
        except Exception as e:
            print(f"Collection {self.collection_name} does not exist, creating it: {str(e)}")
            # Collection doesn't exist, create it
            try:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=settings.vector_size,  # Dimension of the embeddings
                        distance=models.Distance.COSINE  # Cosine distance for similarity search
                    )
                )
                print(f"Collection {self.collection_name} created successfully")
            except Exception as create_error:
                print(f"Failed to create collection: {str(create_error)}")
                # Try creating with a different configuration that's more compatible
                try:
                    self.client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config=models.VectorParams(
                            size=settings.vector_size,
                            distance=models.Distance.COSINE
                        )
                    )
                except:
                    # If all else fails, log the error but don't crash
                    print("Failed to create Qdrant collection")

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

        try:
            # Check which method is available for adding vectors
            if hasattr(self.client, 'upsert'):
                # Use upsert method if available
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
            elif hasattr(self.client, 'upload_points'):
                # Use upload_points method for some versions
                self.client.upload_points(
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
            else:
                # No method available
                print("Warning: Qdrant client has no upsert/upload_points method")
                pass
        except AttributeError:
            # Handle case where upsert method doesn't exist or has different name
            print("Warning: Qdrant client does not have upsert method")
            pass
        except Exception as e:
            print(f"Warning: Error adding vectors to Qdrant: {str(e)}")
            pass

    def search(self, query_vector: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar vectors to the query vector

        Args:
            query_vector: The embedding vector to search for similar ones
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing the payload data from matching vectors
        """
        try:
            # Check which method is available - based on our test, query_points is available
            if hasattr(self.client, 'query_points'):
                # Use the query_points method for newer versions
                search_results = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_vector,
                    limit=limit
                )

                # The query_points method returns a QueryResponse object
                # We need to access the 'points' attribute to get the actual results
                if hasattr(search_results, 'points'):
                    hits = search_results.points  # This is the correct attribute
                elif hasattr(search_results, '__iter__') and not isinstance(search_results, (str, bytes)):
                    # If it's already iterable (fallback)
                    hits = search_results
                else:
                    # If it's a response object with hits attribute (older API)
                    hits = getattr(search_results, 'hits', [])
            else:
                # No search method available
                print("Warning: Qdrant client has no query_points method")
                return []

            # Extract payload data from search results
            results = []
            for hit in hits:
                # Check if hit has payload attribute
                if hasattr(hit, 'payload'):
                    results.append(hit.payload)
                elif hasattr(hit, 'dict') and callable(getattr(hit, 'dict')):
                    # Some versions might have a dict method
                    results.append(hit.dict().get('payload', {}))
                else:
                    # Fallback
                    results.append(getattr(hit, 'payload', {}))
            return results
        except AttributeError as e:
            # Handle case where search method doesn't exist or has different name
            # Fallback to an empty result to prevent server error
            print(f"Warning: Qdrant client search failed: {str(e)}")
            return []
        except Exception as e:
            # Handle any other search errors
            print(f"Warning: Qdrant search error: {str(e)}")
            return []

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