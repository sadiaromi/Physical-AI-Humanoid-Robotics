#!/usr/bin/env python3
"""
Script to process ALL humanoid robotics book content (both docs and modules), 
chunk it, generate embeddings, and store in Qdrant for RAG functionality.
This script should be run from the project root directory.
"""

import os
import sys
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from pathlib import Path
from typing import List, Dict, Any
import re
from uuid import uuid4
import time

# Load environment variables
from dotenv import load_dotenv
load_dotenv()


class Settings:
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_host = os.getenv("QDRANT_HOST")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_port = int(os.getenv("QDRANT_PORT", 6333))
    chunk_size = int(os.getenv("CHUNK_SIZE", 600))
    chunk_overlap = int(os.getenv("CHUNK_OVERLAP", 100))
    vector_size = int(os.getenv("VECTOR_SIZE", 1024))


settings = Settings()


class EmbeddingService:
    def __init__(self):
        self.client = cohere.Client(api_key=settings.cohere_api_key)
        self.model = "embed-english-v3.0"

    def create_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        response = self.client.embed(
            texts=texts,
            model=self.model,
            input_type=input_type
        )
        return response.embeddings


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
                    size=settings.vector_size,
                    distance=models.Distance.COSINE
                )
            )

    def add_vectors(self, vectors: List[List[float]], payloads: List[Dict[str, Any]], vector_ids: list = None):
        """
        Add vectors to the collection
        """
        if vector_ids is None:
            # Generate UUIDs if not provided
            import uuid
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

    def get_vector_count(self) -> int:
        """
        Get the total number of vectors in the collection
        """
        return self.client.count(
            collection_name=self.collection_name
        ).count


class BookDataProcessor:
    def __init__(self):
        """Initialize the book data processor with embedding and vector store services."""
        self.embedding_service = EmbeddingService()
        self.vector_store = VectorStoreService()
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap

    def read_book_content(self, base_path: str) -> List[Dict[str, Any]]:
        """
        Read all markdown files from the book documentation and modules.

        Args:
            base_path: Base directory containing the book markdown files

        Returns:
            List of dictionaries containing file path, title, and content
        """
        content_list = []

        # Define the pattern for book content files
        docs_path = Path(base_path) / "docs"

        if not docs_path.exists():
            print(f"Error: docs directory does not exist at {docs_path}")
            return []

        # Find all markdown files in the docs directory (including subdirectories for modules)
        md_files = list(docs_path.rglob("*.md"))

        for md_file in md_files:
            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                    # Extract title from the file (usually first heading)
                    title_match = re.search(r'^# (.+)$', content, re.MULTILINE)
                    title = title_match.group(1) if title_match else md_file.stem

                    content_list.append({
                        "file_path": str(md_file.relative_to(base_path)),
                        "title": title,
                        "content": content,
                        "module": self._extract_module_from_path(md_file)
                    })
            except Exception as e:
                print(f"Error reading {md_file}: {e}")

        return content_list

    def _extract_module_from_path(self, file_path: Path) -> str:
        """Extract the module name from the file path."""
        parts = file_path.parts
        for part in parts:
            if part.startswith("module"):
                return part
        return "unknown_module"

    def chunk_text(self, text: str, file_info: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Split text into overlapping chunks.

        Args:
            text: The text to chunk
            file_info: Information about the source file

        Returns:
            List of chunk dictionaries with content and metadata
        """
        # Split text into paragraphs first
        paragraphs = text.split('\n\n')

        chunks = []
        chunk_index = 0

        # Use regex to split into sentences for more natural chunks
        sentence_pattern = r'[.!?]+\s+'

        for para in paragraphs:
            # If paragraph is too large, split into sentences
            if len(para) > self.chunk_size:
                # Split paragraph into sentences
                sentences = re.split(sentence_pattern, para)
                current_chunk = ""

                for sentence in sentences:
                    sentence = sentence.strip()
                    if not sentence:
                        continue

                    sentence_length = len(sentence)
                    current_chunk_length = len(current_chunk)

                    # If adding this sentence would exceed chunk size
                    if current_chunk_length + sentence_length > self.chunk_size and current_chunk:
                        # Save current chunk
                        chunks.append({
                            "content": current_chunk.strip(),
                            "index": chunk_index,
                            "file_path": file_info["file_path"],
                            "title": file_info["title"],
                            "module": file_info["module"]
                        })

                        # Start new chunk with overlap
                        overlap_start = max(0, len(current_chunk) - self.chunk_overlap)
                        current_chunk = current_chunk[overlap_start:] + " " + sentence
                        chunk_index += 1
                    else:
                        current_chunk += " " + sentence if current_chunk else sentence

                # Add the last chunk if it has content
                if current_chunk.strip():
                    chunks.append({
                        "content": current_chunk.strip(),
                        "index": chunk_index,
                        "file_path": file_info["file_path"],
                        "title": file_info["title"],
                        "module": file_info["module"]
                    })
                    chunk_index += 1
            else:
                # Add the paragraph as a chunk if it's not too large
                if para.strip():
                    chunks.append({
                        "content": para.strip(),
                        "index": chunk_index,
                        "file_path": file_info["file_path"],
                        "title": file_info["title"],
                        "module": file_info["module"]
                    })
                    chunk_index += 1

        return chunks

    def process_book_data(self, book_path: str):
        """
        Process the entire book: read, chunk, embed, and store in Qdrant.

        Args:
            book_path: Path to the root of the book repository
        """
        print("Reading book content...")
        book_contents = self.read_book_content(book_path)
        print(f"Found {len(book_contents)} markdown files")

        all_chunks = []
        for content_info in book_contents:
            print(f"Processing {content_info['file_path']}...")

            # Skip the front matter in markdown files
            content = content_info['content']
            if content.startswith('---'):
                # Find the end of front matter
                end_front_matter = content.find('---', 3)
                if end_front_matter != -1:
                    content = content[end_front_matter + 3:].strip()

            # Update content in content_info
            content_info['content'] = content

            # Chunk the content
            chunks = self.chunk_text(content, content_info)
            all_chunks.extend(chunks)
            print(f"  Created {len(chunks)} chunks")

        print(f"\nTotal chunks created: {len(all_chunks)}")

        if len(all_chunks) == 0:
            print("No chunks to process. Exiting.")
            return

        # Generate embeddings and store in Qdrant
        print("\nGenerating embeddings and storing in Qdrant...")

        # Process in batches to avoid memory issues and network timeouts
        batch_size = 5  # Reduced batch size to help with potential timeouts
        success_count = 0
        for i in range(0, len(all_chunks), batch_size):
            batch_chunks = all_chunks[i:i+batch_size]
            print(f"Processing batch {i//batch_size + 1}/{(len(all_chunks)-1)//batch_size + 1}")

            # Extract content for embedding
            texts = [chunk["content"] for chunk in batch_chunks]

            try:
                # Generate embeddings
                embeddings = self.embedding_service.create_embeddings(texts, input_type="search_document")

                # Prepare payloads for Qdrant
                payloads = []
                vector_ids = []

                for j, chunk in enumerate(batch_chunks):
                    payload = {
                        "content": chunk["content"],
                        "file_path": chunk["file_path"],
                        "title": chunk["title"],
                        "module": chunk["module"],
                        "chunk_index": chunk["index"],
                        "book_part": "humanoid_robotics_book"
                    }

                    payloads.append(payload)
                    vector_ids.append(str(uuid4()))

                # Store in Qdrant
                self.vector_store.add_vectors(embeddings, payloads, vector_ids)
                success_count += len(batch_chunks)
                print(f"  Successfully stored {len(batch_chunks)} chunks in Qdrant")

            except Exception as e:
                print(f"Error processing batch: {e}")
                # Wait a bit before continuing
                time.sleep(2)

        print(f"\nSuccessfully stored {success_count} chunks in Qdrant")

        # Print summary
        print("\n--- SUMMARY ---")
        print(f"Files processed: {len(book_contents)}")
        print(f"Total chunks created: {len(all_chunks)}")
        print(f"Chunks stored in Qdrant: {success_count}")
        print(f"Qdrant collection: {self.vector_store.collection_name}")
        print(f"Collection vector count: {self.vector_store.get_vector_count()}")

        # List all processed files
        print("\nProcessed files:")
        for content_info in book_contents:
            print(f"  - {content_info['file_path']} (module: {content_info['module']})")


def main():
    # Current directory is the project root
    book_path = str(Path.cwd())

    # Verify the path exists
    if not os.path.exists(book_path):
        print(f"Error: Book path does not exist: {book_path}")
        return

    # Verify docs directory exists
    docs_path = Path(book_path) / "docs"
    if not docs_path.exists():
        print(f"Error: docs directory does not exist: {docs_path}")
        return

    # Verify required environment variables are set
    if not settings.cohere_api_key or not settings.qdrant_api_key:
        print("Error: Environment variables not set properly")
        print(f"Cohere API Key set: {bool(settings.cohere_api_key)}")
        print(f"Qdrant API Key set: {bool(settings.qdrant_api_key)}")
        return

    # Create processor and process the book
    processor = BookDataProcessor()
    processor.process_book_data(book_path)


if __name__ == "__main__":
    main()