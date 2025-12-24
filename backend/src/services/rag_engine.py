from typing import List, Dict, Any, Optional
from src.services.embedding_service import EmbeddingService
from src.services.vector_store import VectorStoreService
from src.services.cohere_client import CohereClient
from src.config.settings import settings


class RAGEngine:
    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.vector_store = VectorStoreService()
        self.cohere_client = CohereClient()
    
    def retrieve_relevant_chunks(self, query: str, book_id: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve the most relevant content chunks for a given query from a specific book
        
        Args:
            query: The user's query
            book_id: The ID of the book to search in
            top_k: Number of top results to return
            
        Returns:
            List of relevant content chunks with metadata
        """
        # Create embedding for the query
        query_embedding = self.embedding_service.create_embedding(query, input_type="search_query")
        
        # Search in the vector store for relevant chunks from the specified book
        results = self.vector_store.search(query_embedding, limit=top_k)
        
        # Filter results to only include chunks from the specified book
        relevant_chunks = []
        for result in results:
            if result.get("book_id") == book_id:
                relevant_chunks.append(result)
        
        # Limit to top_k results after filtering
        return relevant_chunks[:top_k]
    
    def generate_response(self, query: str, context_chunks: List[Dict[str, Any]], selected_text: Optional[str] = None) -> str:
        """
        Generate a response based on the query and context chunks or selected text
        
        Args:
            query: The user's query
            context_chunks: List of relevant content chunks
            selected_text: Optional selected text (if provided, context chunks are ignored)
            
        Returns:
            Generated response
        """
        if selected_text:
            # Use only the selected text as context (no retrieval)
            context = selected_text
        else:
            # Combine all relevant chunks as context
            context_parts = [chunk.get("content", "") for chunk in context_chunks]
            context = "\n\n".join(context_parts)
        
        # Create a prompt for the language model with the context
        if selected_text:
            prompt = f"""
            Based only on the following selected text, please answer the question.
            Do not use any other information beyond what is in the selected text:
            
            Selected text: {context}
            
            Question: {query}
            
            Answer:
            """
        else:
            prompt = f"""
            Based on the following context from the book, please answer the question:
            
            Context: {context}
            
            Question: {query}
            
            Answer:
            """
        
        # Generate and return the response
        return self.cohere_client.generate_response(prompt)
    
    def query_book(self, 
                   query: str, 
                   book_id: str, 
                   selected_text: Optional[str] = None, 
                   top_k: int = 5) -> Dict[str, Any]:
        """
        Main method to query a book, handling both full-book search and selected-text mode
        
        Args:
            query: The user's question
            book_id: The ID of the book to query
            selected_text: Optional selected text to focus on (if provided, ignores book content)
            top_k: Number of top results to return
            
        Returns:
            Dictionary containing the response and source information
        """
        # Validate query length
        if len(query) > settings.max_query_length:
            raise ValueError(f"Query exceeds maximum length of {settings.max_query_length} characters")
        
        # Generate response based on mode
        if selected_text:
            # Selected text mode: use only the provided text
            response = self.generate_response(query, [], selected_text)
            sources = [{"text_snippet": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text}]
        else:
            # Full-book mode: retrieve relevant chunks and generate response
            relevant_chunks = self.retrieve_relevant_chunks(query, book_id, top_k)
            response = self.generate_response(query, relevant_chunks)
            
            # Prepare sources information
            sources = []
            for chunk in relevant_chunks:
                sources.append({
                    "chunk_id": chunk.get("chunk_id"),
                    "section_title": chunk.get("section_title", ""),
                    "page_number": chunk.get("page_number", None)
                })
        
        return {
            "response": response,
            "sources": sources,
            "query": query,
            "book_id": book_id
        }