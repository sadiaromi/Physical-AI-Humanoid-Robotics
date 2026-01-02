from qdrant_client import QdrantClient
from qdrant_client.http import models
import requests

# Configuration
QDRANT_URL = "https://b60853c5-f084-4cb0-ba93-2879e05c483c.europe-west3-0.gcp.cloud.qdrant.io:6333"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.iWc12oasv1F0m2rO_kqarh9qBa7DiDvJS45lRuJ7u_0"
COHERE_API_KEY = "sqB7ygQv30JL2qrrb0OXmdh0vgyVtPQx3nQzPHcM"
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
        "model": "embed-english-v3.0",
        "input_type": "search_query"
    }

    response = requests.post(url, headers=headers, json=data)

    if response.status_code == 200:
        result = response.json()
        return result['embeddings'][0]
    else:
        print(f"Error creating embedding: {response.status_code}, {response.text}")
        import random
        return [random.random() for _ in range(1024)]

# Create Qdrant client
client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

# Create embedding for a test query
query_text = "What is ROS 2?"
query_vector = create_embedding(query_text)

print(f"Query: {query_text}")
print(f"Query vector length: {len(query_vector)}")

# Search in Qdrant
search_results = client.query_points(
    collection_name=COLLECTION_NAME,
    query=query_vector,
    limit=5
)

print(f"\nSearch results type: {type(search_results)}")
print(f"Search results: {search_results}")

# Handle the search results properly
if isinstance(search_results, tuple):
    # If it's a tuple, the first element might contain the results
    if len(search_results) > 0:
        actual_results = search_results[0]
        if hasattr(actual_results, '__iter__') and not isinstance(actual_results, (str, bytes)):
            hits = actual_results
        else:
            hits = []
    else:
        hits = []
elif hasattr(search_results, '__iter__') and not isinstance(search_results, (str, bytes)):
    # If it's already iterable
    hits = search_results
else:
    # Otherwise, get hits attribute
    hits = getattr(search_results, 'hits', [])

print(f"\nHits type: {type(hits)}, length: {len(hits) if hasattr(hits, '__len__') else 'N/A'}")

print(f"\nSearch results (top 5):")
for i, hit in enumerate(hits):
    print(f"Result {i+1}:")
    if hasattr(hit, 'id'):
        print(f"  ID: {hit.id}")
    if hasattr(hit, 'score'):
        print(f"  Score: {hit.score}")
    if hasattr(hit, 'payload'):
        print(f"  Book ID: {hit.payload.get('book_id', 'N/A')}")
        print(f"  Content preview: {hit.payload.get('content', '')[:100]}...")
    print()

# Check total count
count = client.count(collection_name=COLLECTION_NAME)
print(f"\nTotal points in collection: {count.count}")

# Get all book IDs in the collection
records, _ = client.scroll(
    collection_name=COLLECTION_NAME,
    limit=100
)

book_ids = set()
for record in records:
    book_id = record.payload.get('book_id')
    if book_id:
        book_ids.add(book_id)

print(f"Book IDs in collection: {book_ids}")