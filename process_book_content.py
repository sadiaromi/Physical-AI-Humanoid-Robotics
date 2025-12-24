#!/usr/bin/env python3
"""
Script to process the humanoid robotics book content, chunk it,
generate embeddings, and store in Qdrant for RAG functionality.
This script should be run from the project root directory.
"""

import os
import sys
import glob
import re
from typing import List, Dict, Any
from pathlib import Path
from uuid import uuid4

# Add the backend/src directory to the Python path
backend_path = os.path.join(os.path.dirname(__file__), 'backend', 'src')
sys.path.insert(0, backend_path)

from services.embedding_service import EmbeddingService
from services.vector_store import VectorStoreService
from config.settings import settings


class BookDataProcessor:
    def __init__(self):
        """Initialize the book data processor with embedding and vector store services."""
        self.embedding_service = EmbeddingService()
        self.vector_store = VectorStoreService()
        self.chunk_size = settings.chunk_size
        self.chunk_overlap = settings.chunk_overlap

    def read_book_content(self, base_path: str) -> List[Dict[str, Any]]:
        """
        Read all markdown files from the book documentation.

        Args:
            base_path: Base directory containing the book markdown files

        Returns:
            List of dictionaries containing file path, title, and content
        """
        content_list = []

        # Define the pattern for book content files
        docs_path = Path(base_path) / "docs"

        # Find all markdown files in the docs directory (including subdirectories for modules)
        md_files = list(docs_path.rglob("*.md"))

        for md_file in md_files:
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

        # Generate embeddings and store in Qdrant
        print("\nGenerating embeddings and storing in Qdrant...")

        # Process in batches to avoid memory issues and network timeouts
        batch_size = 5  # Reduced batch size to help with potential timeouts
        for i in range(0, len(all_chunks), batch_size):
            batch_chunks = all_chunks[i:i+batch_size]
            print(f"Processing batch {i//batch_size + 1}/{(len(all_chunks)-1)//batch_size + 1}")

            # Extract content for embedding
            texts = [chunk["content"] for chunk in batch_chunks]

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
            try:
                self.vector_store.add_vectors(embeddings, payloads, vector_ids)
            except Exception as e:
                print(f"Error adding batch to Qdrant: {e}")
                # Wait a bit before retrying
                import time
                time.sleep(1)
                # Retry once
                try:
                    self.vector_store.add_vectors(embeddings, payloads, vector_ids)
                except Exception as e2:
                    print(f"Retry also failed: {e2}")
                    raise

        print(f"\nSuccessfully stored {len(all_chunks)} chunks in Qdrant")

        # Print summary
        print("\n--- SUMMARY ---")
        print(f"Files processed: {len(book_contents)}")
        print(f"Total chunks created: {len(all_chunks)}")
        print(f"Chunks stored in Qdrant: {len(all_chunks)}")
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