import pytest
from unittest.mock import Mock, patch, AsyncMock
from src.services.embedding_service import EmbeddingService
from src.config.settings import settings


class TestEmbeddingService:
    """Unit tests for the EmbeddingService class"""
    
    @pytest.fixture
    def embedding_service(self):
        """Create an EmbeddingService instance for testing"""
        with patch('src.services.cohere_client.CohereClient') as mock_cohere:
            service = EmbeddingService()
            service.cohere_client = mock_cohere
            return service
    
    @pytest.mark.asyncio
    async def test_generate_embeddings(self, embedding_service):
        """Test the generate_embeddings method"""
        # Arrange
        text_list = ["This is the first sentence.", "This is the second sentence."]
        expected_embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        
        embedding_service.cohere_client.get_embeddings = AsyncMock(return_value=expected_embeddings)
        
        # Act
        result = await embedding_service.generate_embeddings(text_list)
        
        # Assert
        assert result == expected_embeddings
        embedding_service.cohere_client.get_embeddings.assert_called_once_with(text_list)
    
    @pytest.mark.asyncio
    async def test_generate_single_embedding(self, embedding_service):
        """Test the generate_single_embedding method"""
        # Arrange
        text = "This is a test sentence."
        expected_embedding = [0.1, 0.2, 0.3]
        
        embedding_service.cohere_client.get_embeddings = AsyncMock(return_value=[expected_embedding])
        
        # Act
        result = await embedding_service.generate_single_embedding(text)
        
        # Assert
        assert result == expected_embedding
        embedding_service.cohere_client.get_embeddings.assert_called_once_with([text])
    
    def test_calculate_similarity(self, embedding_service):
        """Test the calculate_similarity method"""
        # Arrange
        embedding1 = [1.0, 0.0, 0.0]
        embedding2 = [0.0, 1.0, 0.0]
        # Dot product of orthogonal vectors is 0
        
        # Act
        result = embedding_service.calculate_similarity(embedding1, embedding2)
        
        # Assert
        assert result == 0.0
    
    def test_calculate_similarity_identical(self, embedding_service):
        """Test the calculate_similarity method with identical embeddings"""
        # Arrange
        embedding1 = [1.0, 0.0, 0.0]
        embedding2 = [1.0, 0.0, 0.0]
        # Dot product of identical unit vectors is 1.0
        
        # Act
        result = embedding_service.calculate_similarity(embedding1, embedding2)
        
        # Assert
        assert result == 1.0