import pytest
from unittest.mock import Mock, patch, AsyncMock
from src.services.rag_engine import RAGEngine
from src.services.vector_store import VectorStoreService
from src.services.embedding_service import EmbeddingService
from src.services.cohere_client import CohereClient


class TestRAGEngine:
    """Unit tests for the RAGEngine class"""
    
    @pytest.fixture
    def rag_engine(self):
        """Create a RAGEngine instance for testing"""
        with patch.object(VectorStoreService, '__init__', return_value=None), \
             patch.object(EmbeddingService, '__init__', return_value=None), \
             patch.object(CohereClient, '__init__', return_value=None):
            
            engine = RAGEngine()
            engine.vector_store = Mock(spec=VectorStoreService)
            engine.embedding_service = Mock(spec=EmbeddingService)
            engine.cohere_client = Mock(spec=CohereClient)
            return engine
    
    @pytest.mark.asyncio
    async def test_query_with_full_book_context(self, rag_engine):
        """Test querying with full book context"""
        # Arrange
        question = "What is the main concept?"
        book_id = "test-book-id"
        relevant_chunks = [
            {"id": "chunk1", "content": "The main concept is about AI."},
            {"id": "chunk2", "content": "AI is a branch of computer science."}
        ]
        embeddings = [[0.1, 0.2, 0.3]]
        generated_response = "The main concept is about AI."
        
        rag_engine.embedding_service.generate_single_embedding = AsyncMock(return_value=embeddings[0])
        rag_engine.vector_store.search_similar_chunks = AsyncMock(return_value=relevant_chunks)
        rag_engine.cohere_client.generate_response = AsyncMock(return_value=generated_response)
        
        # Act
        result = await rag_engine.query(question, book_id)
        
        # Assert
        assert result["response"] == generated_response
        assert "context_used" in result
        rag_engine.embedding_service.generate_single_embedding.assert_called_once_with(question)
        rag_engine.vector_store.search_similar_chunks.assert_called_once()
        rag_engine.cohere_client.generate_response.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_query_with_selected_text(self, rag_engine):
        """Test querying with selected text (bypasses retrieval)"""
        # Arrange
        question = "Explain this concept"
        book_id = "test-book-id"
        selected_text = "The concept of machine learning is..."
        generated_response = "Machine learning is a subset of AI."
        
        rag_engine.cohere_client.generate_response = AsyncMock(return_value=generated_response)
        
        # Act
        result = await rag_engine.query(question, book_id, selected_text=selected_text)
        
        # Assert
        assert result["response"] == generated_response
        # When using selected text, retrieval should be bypassed
        rag_engine.vector_store.search_similar_chunks.assert_not_called()
        rag_engine.cohere_client.generate_response.assert_called_once()
    
    @pytest.mark.asyncio
    async def test_get_relevant_chunks(self, rag_engine):
        """Test retrieval of relevant chunks"""
        # Arrange
        query_embedding = [0.5, 0.6, 0.7]
        book_id = "test-book-id"
        expected_chunks = [
            {"id": "chunk1", "content": "Relevant content here", "score": 0.95},
            {"id": "chunk2", "content": "Also relevant content", "score": 0.87}
        ]
        
        rag_engine.embedding_service.generate_single_embedding = AsyncMock(return_value=query_embedding)
        rag_engine.vector_store.search_similar_chunks = AsyncMock(return_value=expected_chunks)
        
        # Act
        result = await rag_engine.get_relevant_chunks(question="Test question", book_id=book_id)
        
        # Assert
        assert result == expected_chunks
        rag_engine.embedding_service.generate_single_embedding.assert_called_once_with("Test question")
        rag_engine.vector_store.search_similar_chunks.assert_called_once_with(query_embedding, book_id, top_k=5)