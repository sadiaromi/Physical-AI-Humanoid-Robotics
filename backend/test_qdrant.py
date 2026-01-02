#!/usr/bin/env python3
"""Test script to check Qdrant client methods"""

from qdrant_client import QdrantClient
from src.config.settings import settings

# Initialize Qdrant client with connection settings
client = QdrantClient(
    url=settings.qdrant_host,
    api_key=settings.qdrant_api_key,
    port=settings.qdrant_port,
    grpc_port=6334,
    prefer_grpc=True
)

print("Testing Qdrant client methods...")

# Check if search method exists
print(f"Has search method: {hasattr(client, 'search')}")
print(f"Has query_points method: {hasattr(client, 'query_points')}")

# Check available methods
available_methods = [m for m in dir(client) if not m.startswith('_') and callable(getattr(client, m))]
print(f"Available methods: {available_methods}")

# Check if search is in the available methods
search_methods = [m for m in available_methods if 'search' in m.lower()]
print(f"Search-related methods: {search_methods}")

# Test getting collection
try:
    collection_info = client.get_collection("book_content_chunks")
    print(f"Collection exists: {collection_info}")
except Exception as e:
    print(f"Collection error: {e}")