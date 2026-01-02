import uuid
from qdrant_client import QdrantClient
from qdrant_client.http import models
import requests
import os
from typing import List, Dict, Any

# Configuration
QDRANT_URL = "https://b60853c5-f084-4cb0-ba93-2879e05c483c.europe-west3-0.gcp.cloud.qdrant.io:6333"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.iWc12oasv1F0m2rO_kqarh9qBa7DiDvJS45lRuJ7u_0"
COHERE_API_KEY = "sqB7ygQv30JL2qrrb0OXmdh0vgyVtPQx3nQzPHcM"  # From your .env
COLLECTION_NAME = "physical-ai-book"

def create_embedding(text: str) -> List[float]:
    """
    Create an embedding using Cohere API
    """
    url = "https://api.cohere.com/v1/embed"

    headers = {
        "Authorization": f"Bearer {COHERE_API_KEY}",
        "Content-Type": "application/json"
    }

    data = {
        "texts": [text],
        "model": "embed-english-v3.0",  # Using Cohere's embedding model
        "input_type": "search_query"
    }

    response = requests.post(url, headers=headers, json=data)

    if response.status_code == 200:
        result = response.json()
        return result['embeddings'][0]  # Return the first embedding
    else:
        print(f"Error creating embedding: {response.status_code}, {response.text}")
        # Return a random vector as fallback
        import random
        return [random.random() for _ in range(1024)]

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[Dict[str, Any]]:
    """
    Split text into overlapping chunks
    """
    sentences = text.split('. ')
    chunks = []
    current_chunk = ""
    chunk_index = 0

    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue

        if len(current_chunk) + len(sentence) < chunk_size:
            current_chunk += " " + sentence + "."
        else:
            # Save current chunk
            if current_chunk.strip():
                chunks.append({
                    "content": current_chunk.strip(),
                    "chunk_index": chunk_index
                })
                chunk_index += 1

            # Start new chunk with overlap
            words = current_chunk.split()
            overlap_text = " ".join(words[-overlap:]) if len(words) > overlap else current_chunk
            current_chunk = overlap_text + " " + sentence + "."

    # Add the last chunk
    if current_chunk.strip():
        chunks.append({
            "content": current_chunk.strip(),
            "chunk_index": chunk_index
        })

    return chunks

def ingest_book_content():
    """
    Ingest book content into Qdrant
    """
    # Sample book content about ROS 2 and Physical AI
    book_content = """
# Chapter 1: Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## What is ROS 2?

ROS 2 is the second generation of the Robot Operating System. It addresses many of the limitations of ROS 1, particularly in areas of security, real-time performance, and support for commercial products. ROS 2 is built on DDS (Data Distribution Service), which provides a publish-subscribe communication model.

## Key Features of ROS 2

- **Real-time support**: ROS 2 provides better real-time performance compared to ROS 1.
- **Security**: Built-in security features including authentication, access control, and encryption.
- **Distributed architecture**: Nodes can run on different machines and communicate over networks.
- **Multi-language support**: Supports C++, Python, and other languages.

## Nodes in ROS 2

Nodes are the fundamental building blocks of a ROS 2 system. A node is an executable that uses ROS 2 to communicate with other nodes. Nodes can publish messages to topics, subscribe to topics, provide services, or call services.

## Topics and Messages

Topics are named buses over which nodes exchange messages. A node can publish messages to a topic and/or subscribe to messages from a topic. Messages are the data packets that are passed between nodes.

## Services

Services provide a request/reply communication pattern. A service client sends a request to a service server, which processes the request and returns a response.

# Chapter 2: Fundamentals of Humanoid Robotics

Humanoid robots are robots with human-like features. They are designed to interact with human environments and tools. Key components include actuators for movement, sensors for perception, and control systems for coordination.

# Chapter 3: Physics Simulation: Gravity, Collisions, Sensors

Physics simulation is crucial for humanoid robotics. It involves modeling gravitational forces affecting the robot, collision detection and response, and sensor simulation for virtual testing. Gravity simulation ensures robots can maintain balance. Collision detection prevents damage. Sensor simulation allows for testing in virtual environments.

# Chapter 4: AI for Humanoid Control

Artificial intelligence algorithms control humanoid robots through machine learning for movement optimization, neural networks for pattern recognition, and reinforcement learning for skill acquisition.
"""

    # Create Qdrant client
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    # Ensure collection exists
    try:
        client.get_collection(COLLECTION_NAME)
        print(f"Collection '{COLLECTION_NAME}' already exists")
    except:
        print(f"Creating collection '{COLLECTION_NAME}'...")
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embeddings are typically 1024 dimensions
                distance=models.Distance.COSINE
            )
        )

    # Chunk the content
    print("Chunking book content...")
    chunks = chunk_text(book_content, chunk_size=300, overlap=30)
    print(f"Created {len(chunks)} chunks")

    # Create embeddings and store in Qdrant
    points = []
    for i, chunk in enumerate(chunks):
        print(f"Processing chunk {i+1}/{len(chunks)}...")

        # Create embedding for the content
        vector = create_embedding(chunk["content"])

        # Create a point for Qdrant
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=vector,
            payload={
                "book_id": COLLECTION_NAME,
                "content": chunk["content"],
                "section_title": f"Book Section {chunk['chunk_index']}",
                "chunk_index": chunk["chunk_index"],
                "source": "Physical AI & Humanoid Robotics Book"
            }
        )
        points.append(point)

    # Upload points to Qdrant
    print(f"Uploading {len(points)} vectors to Qdrant...")
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )

    # Verify upload
    count = client.count(collection_name=COLLECTION_NAME)
    print(f"Successfully uploaded {count.count} points to collection '{COLLECTION_NAME}'")

    # Show a few examples
    records, _ = client.scroll(
        collection_name=COLLECTION_NAME,
        limit=3
    )

    print("\nSample stored content:")
    for i, record in enumerate(records):
        print(f"Sample {i+1}:")
        print(f"  Content preview: {record.payload['content'][:100]}...")
        print(f"  Chunk index: {record.payload['chunk_index']}")
        print()

if __name__ == "__main__":
    print("Starting book content ingestion...")
    ingest_book_content()
    print("Book content ingestion completed!")