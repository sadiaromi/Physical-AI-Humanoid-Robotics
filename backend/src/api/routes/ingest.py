from fastapi import APIRouter, Depends, HTTPException, status, UploadFile, File
from typing import Dict, Any, List
import uuid
from datetime import datetime
import tempfile
import os
from sqlalchemy.orm import Session
from src.middleware.auth import require_api_key
from src.middleware.rate_limit import check_rate_limit
from src.config.database import get_db
from src.models.book import Book
from src.services.embedding_service import EmbeddingService
from src.services.vector_store import VectorStoreService
from src.utils.helpers import get_token_count
from src.config.settings import settings


router = APIRouter()


@router.post("/books",
             summary="Ingest a new book",
             description="Upload and process a book for querying")
async def ingest_book(
    title: str,
    author: str,
    file: UploadFile = File(...),
    db: Session = Depends(get_db),
    auth_validated: bool = Depends(require_api_key),
    rate_limited: bool = Depends(check_rate_limit)
):
    """
    Endpoint to upload and process a book
    """
    try:
        # Validate file type
        if not file.content_type.startswith('text/') and file.content_type not in [
            'application/pdf', 'application/msword',
            'application/vnd.openxmlformats-officedocument.wordprocessingml.document'
        ]:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Unsupported file type. Please upload a text, PDF, or Word document."
            )

        # Create a book record in the database
        book = Book(
            title=title,
            author=author,
            status="processing"  # Set initial status to processing
        )
        db.add(book)
        db.commit()
        db.refresh(book)

        # Read the uploaded file
        content = await file.read()
        text_content = content.decode('utf-8') if file.content_type.startswith('text/') else extract_text_from_file(content, file.content_type)

        # Chunk the text content
        chunks = chunk_text(text_content, book.id)

        # Process chunks and create embeddings
        embedding_service = EmbeddingService()
        vector_store = VectorStoreService()

        # Prepare vectors and payloads for bulk upload
        vectors = []
        payloads = []
        vector_ids = []

        for i, chunk in enumerate(chunks):
            # Create embedding for the chunk
            embedding = embedding_service.create_embedding(chunk['content'], input_type="search_document")

            # Prepare payload with metadata
            payload = {
                "book_id": str(book.id),
                "chunk_id": str(uuid.uuid4()),
                "content": chunk['content'],
                "chunk_index": chunk['index'],
                "section_title": chunk.get('section_title', ''),
                "page_number": chunk.get('page_number')
            }

            vectors.append(embedding)
            payloads.append(payload)
            vector_ids.append(payload['chunk_id'])

        # Upload all vectors to the vector store
        vector_store.add_vectors(vectors, payloads, vector_ids)

        # Update book status to 'ready'
        book.status = "ready"
        db.commit()

        return {
            "book_id": str(book.id),
            "status": "ready",
            "message": f"Book '{title}' ingested successfully with {len(chunks)} chunks"
        }

    except Exception as e:
        # If there was an error, update book status to 'failed'
        book.status = "failed"
        db.commit()

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing book: {str(e)}"
        )


@router.get("/books/{book_id}",
            summary="Get book details",
            description="Retrieve metadata and status for a specific book")
async def get_book(
    book_id: str,
    db: Session = Depends(get_db),
    auth_validated: bool = Depends(require_api_key)
):
    """
    Get details for a specific book
    """
    try:
        # Validate UUID format
        uuid_obj = uuid.UUID(book_id)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid book ID format"
        )

    # Fetch the book from the database
    book = db.query(Book).filter(Book.id == uuid_obj).first()

    if not book:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Book not found"
        )

    return {
        "book_id": str(book.id),
        "title": book.title,
        "author": book.author,
        "isbn": book.isbn,
        "status": book.status,
        "language": book.language,
        "created_at": book.created_at,
        "updated_at": book.updated_at,
        "metadata": book.book_metadata
    }


def extract_text_from_file(file_content: bytes, content_type: str) -> str:
    """
    Extract text from various file types (PDF, DOC, DOCX, etc.)
    For this implementation, we'll focus on PDF as an example
    """
    if content_type == 'application/pdf':
        # Use PyPDF2 or similar to extract text from PDF
        # For now, we'll simulate the process
        import PyPDF2
        from io import BytesIO

        pdf_file = BytesIO(file_content)
        pdf_reader = PyPDF2.PdfReader(pdf_file)

        text = ""
        for page in pdf_reader.pages:
            text += page.extract_text() + "\n"

        return text
    else:
        # For other file types, return empty for now
        # In a complete implementation, you would handle DOC, DOCX, etc.
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Text extraction not implemented for {content_type}"
        )


def chunk_text(text: str, book_id: uuid.UUID) -> List[Dict[str, Any]]:
    """
    Split text into overlapping chunks
    """
    # Tokenize the text into sentences or paragraphs
    import re

    # Split text into paragraphs first
    paragraphs = text.split('\n\n')

    chunks = []
    chunk_index = 0

    for para in paragraphs:
        # If paragraph is too large, split into sentences
        para_token_count = get_token_count(para)

        if para_token_count > settings.chunk_size:
            # Split paragraph into sentences
            sentences = re.split(r'[.!?]+', para)
            current_chunk = ""

            for sentence in sentences:
                sentence = sentence.strip()
                if not sentence:
                    continue

                sentence_token_count = get_token_count(sentence)
                current_token_count = get_token_count(current_chunk)

                # If adding this sentence would exceed chunk size
                if current_token_count + sentence_token_count > settings.chunk_size and current_chunk:
                    # Save current chunk
                    chunks.append({
                        "content": current_chunk.strip(),
                        "index": chunk_index,
                        "book_id": book_id
                    })

                    # Start new chunk with overlap
                    overlap_start = max(0, len(current_chunk) - settings.chunk_overlap)
                    current_chunk = current_chunk[overlap_start:] + " " + sentence
                    chunk_index += 1
                else:
                    current_chunk += " " + sentence

            # Add the last chunk if it has content
            if current_chunk.strip():
                chunks.append({
                    "content": current_chunk.strip(),
                    "index": chunk_index,
                    "book_id": book_id
                })
                chunk_index += 1
        else:
            # Add the paragraph as a chunk if it's not too large
            if para.strip():
                chunks.append({
                    "content": para.strip(),
                    "index": chunk_index,
                    "book_id": book_id
                })
                chunk_index += 1

    return chunks