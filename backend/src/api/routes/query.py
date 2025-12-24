from fastapi import APIRouter, Depends, HTTPException, status
from typing import Dict, Any
from src.models.query import QueryRequest, QueryResponse
from src.services.rag_engine import RAGEngine
from src.middleware.auth import require_api_key
from src.middleware.rate_limit import check_rate_limit
from src.utils.helpers import generate_uuid
from src.config.database import get_db
from sqlalchemy.orm import Session
import logging


router = APIRouter()
logger = logging.getLogger(__name__)


@router.post("/query", 
             response_model=QueryResponse,
             summary="Query book content",
             description="Submit a question about book content and receive an AI-generated response")
async def query_book(
    request: QueryRequest,
    db: Session = Depends(get_db),
    auth_validated: bool = Depends(require_api_key),
    rate_limited: bool = Depends(check_rate_limit)
):
    """
    Query endpoint for asking questions about book content
    """
    # Validate parameters
    if not request.question.strip():
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Question cannot be empty"
        )
    
    # Initialize the RAG engine
    rag_engine = RAGEngine()
    
    try:
        # Perform the query
        result = rag_engine.query_book(
            query=request.question,
            book_id=request.book_id,
            selected_text=request.selected_text
        )
        
        # Generate a query ID for tracking
        query_id = generate_uuid()
        
        # Prepare the response
        response = QueryResponse(
            response=result["response"],
            sources=result["sources"],
            query_id=query_id
        )
        
        # Log the query for analytics (without storing the actual question per privacy requirements)
        logger.info(f"Query completed successfully with ID: {query_id}")
        
        return response
    
    except ValueError as e:
        # Handle validation errors (like query too long)
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    
    except Exception as e:
        # Log the error
        logger.error(f"Error processing query: {str(e)}")
        
        # Return a generic error response
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your query"
        )