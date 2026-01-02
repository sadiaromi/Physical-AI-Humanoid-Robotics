from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid

# Connect to Qdrant cloud
qdrant_client = QdrantClient(
    url="https://b60853c5-f084-4cb0-ba93-2879e05c483c.europe-west3-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.iWc12oasv1F0m2rO_kqarh9qBa7DiDvJS45lRuJ7u_0",
)

print("Available collections:", qdrant_client.get_collections())

# Create or ensure the collection exists
collection_name = "physical-ai-book"
try:
    qdrant_client.get_collection(collection_name)
    print(f"Collection '{collection_name}' already exists")
except:
    print(f"Creating collection '{collection_name}'...")
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=1024,  # Adjust based on your embedding size
            distance=models.Distance.COSINE
        )
    )

# Sample book content to add
book_content = [
    {
        "id": str(uuid.uuid4()),
        "vector": [0.1] * 1024,  # Example vector - in real scenario this would be an embedding
        "payload": {
            "book_id": "physical-ai-book",
            "content": "Physical AI is a field that combines artificial intelligence with physical systems. It focuses on creating intelligent agents that can interact with the physical world effectively.",
            "section_title": "Introduction to Physical AI",
            "chunk_index": 0
        }
    },
    {
        "id": str(uuid.uuid4()),
        "vector": [0.2] * 1024,  # Example vector
        "payload": {
            "book_id": "physical-ai-book",
            "content": "Humanoid robots are robots with human-like features. They are designed to interact with human environments and tools. Key components include actuators for movement, sensors for perception, and control systems for coordination.",
            "section_title": "Fundamentals of Humanoid Robotics",
            "chunk_index": 1
        }
    },
    {
        "id": str(uuid.uuid4()),
        "vector": [0.3] * 1024,  # Example vector
        "payload": {
            "book_id": "physical-ai-book",
            "content": "Physics simulation is crucial for humanoid robotics. It involves modeling gravitational forces affecting the robot, collision detection and response, and sensor simulation for virtual testing.",
            "section_title": "Physics Simulation: Gravity, Collisions, Sensors",
            "chunk_index": 2
        }
    }
]

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
except Exception as e:
    print(f"Error adding vectors: {e}")

# Fetch and print the stored data
try:
    # Get all points from the collection
    records = qdrant_client.scroll(
        collection_name=collection_name,
        limit=10
    )

    print(f"\nRetrieved {len(records[0])} records from collection '{collection_name}':")
    for i, record in enumerate(records[0]):
        print(f"Record {i+1}:")
        print(f"  ID: {record.id}")
        print(f"  Payload: {record.payload}")
        print(f"  Vector length: {len(record.vector) if record.vector else 'N/A'}")
        print()

except Exception as e:
    print(f"Error fetching data: {e}")

# Test a search query
try:
    # Search with a sample query vector
    query_vector = [0.15] * 1024  # Similar to first entry
    search_results = qdrant_client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=5
    )

    print(f"Search results (top 5):")
    for i, hit in enumerate(search_results):
        print(f"Result {i+1}:")
        print(f"  ID: {hit.id}")
        print(f"  Score: {hit.score}")
        print(f"  Payload: {hit.payload}")
        print()

except Exception as e:
    print(f"Error searching: {e}")