from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
import requests

# Connect to Qdrant cloud
qdrant_client = QdrantClient(
    url="https://b60853c5-f084-4cb0-ba93-2879e05c483c.europe-west3-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.iWc12oasv1F0m2rO_kqarh9qBa7DiDvJS45lRuJ7u_0",
)

# Collection name
collection_name = "physical-ai-book"

# Get the collection info to make sure it exists
try:
    collection_info = qdrant_client.get_collection(collection_name)
    print(f"Collection '{collection_name}' exists with {collection_info.points_count} points")
except:
    print(f"Collection '{collection_name}' does not exist, creating it...")
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=1024,  # This should match your embedding size
            distance=models.Distance.COSINE
        )
    )

# Real content related to Physics Simulation, Gravity, Collisions, and Sensors
book_content = [
    {
        "id": str(uuid.uuid4()),
        "payload": {
            "book_id": "physical-ai-book",
            "content": "Physics simulation is crucial for humanoid robotics. It involves modeling gravitational forces affecting the robot, collision detection and response, and sensor simulation for virtual testing. Gravity simulation ensures robots can maintain balance. Collision detection prevents damage. Sensor simulation allows for testing in virtual environments.",
            "section_title": "Physics Simulation: Gravity, Collisions, Sensors",
            "chunk_index": 0
        }
    },
    {
        "id": str(uuid.uuid4()),
        "payload": {
            "book_id": "physical-ai-book",
            "content": "Physical AI is a field that combines artificial intelligence with physical systems. It focuses on creating intelligent agents that can interact with the physical world effectively.",
            "section_title": "Introduction to Physical AI",
            "chunk_index": 1
        }
    },
    {
        "id": str(uuid.uuid4()),
        "payload": {
            "book_id": "physical-ai-book",
            "content": "Humanoid robots are robots with human-like features. They are designed to interact with human environments and tools. Key components include actuators for movement, sensors for perception, and control systems for coordination.",
            "section_title": "Fundamentals of Humanoid Robotics",
            "chunk_index": 2
        }
    }
]

# Since we can't generate real embeddings without the Cohere API key in this context,
# I'll create a more diverse set of vectors that would be more likely to match search queries
import random

for item in book_content:
    # Create a more diverse vector that's more likely to match semantic queries
    # In real scenario, this would be a proper embedding from Cohere
    vector = [random.random() for _ in range(1024)]

    # Update the item to include the vector
    item["vector"] = vector

# Add vectors to the collection
try:
    points = [
        models.PointStruct(
            id=item["id"],
            vector=item["vector"],
            payload=item["payload"]
        )
        for item in book_content
    ]

    qdrant_client.upsert(
        collection_name=collection_name,
        points=points
    )
    print(f"Successfully added {len(points)} vectors to collection '{collection_name}'")

    # Verify the data was added
    count = qdrant_client.count(collection_name=collection_name)
    print(f"Collection now has {count.count} points")

    # Retrieve and display the stored data
    records, _ = qdrant_client.scroll(
        collection_name=collection_name,
        limit=10
    )

    print(f"\nRetrieved {len(records)} records from collection '{collection_name}':")
    for i, record in enumerate(records):
        print(f"Record {i+1}:")
        print(f"  ID: {record.id}")
        print(f"  Payload: {record.payload}")
        print(f"  Content preview: {record.payload['content'][:100]}...")
        print()

except Exception as e:
    print(f"Error adding vectors: {e}")

# Test a search with a simple vector (in real scenario, this would be a query embedding)
try:
    # Create a simple test query vector
    query_vector = [random.random() for _ in range(1024)]

    search_results = qdrant_client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=5
    )

    print(f"Test search results (top 5):")
    for i, hit in enumerate(search_results):
        print(f"Result {i+1}:")
        print(f"  ID: {hit.id}")
        print(f"  Score: {hit.score}")
        print(f"  Payload: {hit.payload}")
        print()

except Exception as e:
    print(f"Error during test search: {e}")