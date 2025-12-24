import argparse
import sys
import os
from sqlalchemy.orm import Session
from src.config.database import SessionLocal, init_db
from src.models.book import Book
from src.services.embedding_service import EmbeddingService
from src.services.vector_store import VectorStoreService
from src.api.routes.ingest import chunk_text
from src.config.settings import settings
from src.utils.helpers import get_token_count
import uuid


def get_db_session() -> Session:
    """
    Get a database session
    """
    db = SessionLocal()
    try:
        return db
    except Exception as e:
        print(f"Error creating database session: {e}")
        sys.exit(1)


def ingest_book_cli(file_path: str, title: str, author: str):
    """
    CLI function to ingest a book
    """
    print(f"Starting ingestion for '{title}' by {author}")
    print(f"File: {file_path}")
    
    # Check if file exists
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} does not exist")
        return
    
    # Read the file
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            text_content = file.read()
        print(f"File read successfully. Content length: {len(text_content)} characters")
    except Exception as e:
        print(f"Error reading file: {e}")
        return
    
    # Create a database session
    db = get_db_session()
    
    try:
        # Create a book record in the database
        book = Book(
            title=title,
            author=author,
            status="processing"  # Set initial status to processing
        )
        db.add(book)
        db.commit()
        db.refresh(book)
        
        print(f"Created book record with ID: {book.id}")
        
        # Chunk the text content
        print("Chunking text content...")
        chunks = chunk_text(text_content, book.id)
        print(f"Created {len(chunks)} chunks")
        
        # Process chunks and create embeddings
        print("Initializing embedding service...")
        embedding_service = EmbeddingService()
        
        print("Initializing vector store...")
        vector_store = VectorStoreService()
        
        # Prepare vectors and payloads for bulk upload
        vectors = []
        payloads = []
        vector_ids = []
        
        print("Creating embeddings for chunks...")
        for i, chunk in enumerate(chunks):
            # Show progress every 10 chunks
            if (i + 1) % 10 == 0:
                print(f"Processed {i + 1}/{len(chunks)} chunks...")
            
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
        print("Uploading vectors to vector store...")
        vector_store.add_vectors(vectors, payloads, vector_ids)
        
        # Update book status to 'ready'
        book.status = "ready"
        db.commit()
        
        print(f"Successfully ingested book '{title}' with {len(chunks)} chunks")
        print(f"Book ID: {book.id}")
        
    except Exception as e:
        print(f"Error during ingestion: {e}")
        # If there was an error, update book status to 'failed'
        if 'book' in locals():
            book.status = "failed"
            db.commit()
        raise e
    
    finally:
        db.close()


def init_db_cli():
    """
    CLI function to initialize the database
    """
    print("Initializing database...")
    init_db()
    print("Database initialized successfully!")


def main():
    parser = argparse.ArgumentParser(description="CLI tool for RAG Chatbot")
    subparsers = parser.add_subparsers(dest="command", help="Available commands")
    
    # Parser for 'ingest' command
    ingest_parser = subparsers.add_parser("ingest", help="Ingest a book for querying")
    ingest_parser.add_argument("--file", required=True, help="Path to the book file (text format)")
    ingest_parser.add_argument("--title", required=True, help="Title of the book")
    ingest_parser.add_argument("--author", required=True, help="Author of the book")
    
    # Parser for 'init_db' command
    init_db_parser = subparsers.add_parser("init_db", help="Initialize the database")
    
    args = parser.parse_args()
    
    if args.command == "ingest":
        ingest_book_cli(args.file, args.title, args.author)
    elif args.command == "init_db":
        init_db_cli()
    else:
        parser.print_help()


if __name__ == "__main__":
    main()