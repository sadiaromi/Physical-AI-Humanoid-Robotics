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

        # Check if we have sufficient context to answer the question
        if not context.strip() or len(context.strip()) == 0:
            # No relevant content found in the book for this query
            return "I cannot answer this question based on the book content. The information you're looking for is not available in the Physical AI & Humanoid Robotics book."

        # Create a prompt for the language model with the context
        if selected_text:
            prompt = f"""
            You are an AI assistant for the "Physical AI & Humanoid Robotics" book.
            Answer the user's question based only on the following selected text from the book.
            If the selected text does not contain information to answer the question,
            respond with: "I cannot answer this question based on the selected text.
            The information you're looking for is not available in the Physical AI & Humanoid Robotics book."

            Selected text: {context}

            Question: {query}

            Answer:
            """
        else:
            prompt = f"""
            You are an AI assistant for the "Physical AI & Humanoid Robotics" book.
            Answer the user's question based only on the following context from the book.
            If the context does not contain information to answer the question,
            respond with: "I cannot answer this question based on the book content.
            The information you're looking for is not available in the Physical AI & Humanoid Robotics book."

            Context: {context}

            Question: {query}

            Answer:
            """

        # Generate and return the response
        response = self.cohere_client.generate_response(prompt)

        # Check if the response indicates lack of information, is a mock response, or Cohere failed
        if ("cannot answer this question based on" in response.lower() or
            "mock response" in response.lower() or
            "no response generated" in response.lower() or
            response.strip() == ""):
            # If Cohere fails or returns no relevant info, return a summary of the context
            if context.strip() and len(context.strip()) > 0:
                # Return the most relevant context as the answer
                context_preview = context[:500] + "..." if len(context) > 500 else context
                return f"Based on the book content:\n\n{context_preview}"
            else:
                return "I cannot answer this question based on the book content. The information you're looking for is not available in the Physical AI & Humanoid Robotics book."

        return response
    
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