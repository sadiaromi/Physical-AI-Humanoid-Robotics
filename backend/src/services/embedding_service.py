import cohere
from typing import List, Optional
from ..config.settings import settings


class EmbeddingService:
    def __init__(self):
        self.client = cohere.Client(api_key=settings.cohere_api_key)
        self.model = "embed-english-v3.0"  # Using Cohere's recommended embedding model

    def create_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Create embeddings for a list of texts using Cohere API

        Args:
            texts: List of texts to embed
            input_type: Type of input (search_document, search_query, classification, etc.)

        Returns:
            List of embedding vectors
        """
        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type=input_type
        )

        return response.embeddings

    def create_embedding(self, text: str, input_type: str = "search_query") -> List[float]:
        """
        Create embedding for a single text

        Args:
            text: Text to embed
            input_type: Type of input (search_document, search_query, classification, etc.)

        Returns:
            Embedding vector
        """
        response = self.client.embed(
            texts=[text],
            model=self.model,
            input_type=input_type
        )

        return response.embeddings[0]  # Return the first (and only) embedding

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings produced by the model
        """
        # Using a sample text to determine embedding dimensions
        sample_embedding = self.create_embedding("sample text")
        return len(sample_embedding)

    def calculate_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings
        """
        import numpy as np

        # Convert to numpy arrays
        vec1 = np.array(embedding1)
        vec2 = np.array(embedding2)

        # Calculate cosine similarity
        dot_product = np.dot(vec1, vec2)
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        return float(dot_product / (norm1 * norm2))