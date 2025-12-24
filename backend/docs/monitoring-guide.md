# Monitoring Implementation Guide

This document describes how to monitor Qdrant and Neon usage in the RAG Chatbot system.

## Qdrant Monitoring

### Metrics to Track

1. **Vector Database Performance**
   - Query response times
   - Indexing performance
   - Memory usage
   - Disk space utilization

2. **API Usage**
   - Requests per minute/hour/day
   - API call success/failure rates
   - Embedding generation counts

3. **Collection Management**
   - Number of vectors stored
   - Collection sizes
   - Index statistics

### Monitoring Implementation

```python
# Example monitoring code for Qdrant usage
import time
from typing import Dict, Any
from qdrant_client import QdrantClient

class QdrantMonitor:
    def __init__(self, client: QdrantClient):
        self.client = client
        self.metrics = {
            "queries": 0,
            "query_time_total": 0.0,
            "last_query_time": 0.0
        }
    
    def monitored_search(self, *args, **kwargs):
        start_time = time.time()
        try:
            result = self.client.search(*args, **kwargs)
            self.metrics["queries"] += 1
            query_time = time.time() - start_time
            self.metrics["query_time_total"] += query_time
            self.metrics["last_query_time"] = query_time
            return result
        except Exception as e:
            # Log errors for monitoring
            self.log_error("search", str(e))
            raise
    
    def get_metrics(self) -> Dict[str, Any]:
        avg_query_time = (
            self.metrics["query_time_total"] / self.metrics["queries"]
            if self.metrics["queries"] > 0 else 0
        )
        return {
            "total_queries": self.metrics["queries"],
            "average_query_time": avg_query_time,
            "last_query_time": self.metrics["last_query_time"],
            "collection_info": self.client.openapi()
        }
```

## Neon Postgres Monitoring

### Metrics to Track

1. **Database Performance**
   - Query response times
   - Connection pool utilization
   - Active connections
   - Slow query log

2. **Resource Usage**
   - Database size
   - Storage utilization
   - CPU and memory usage

3. **Connection Metrics**
   - Connection attempts
   - Successful connections
   - Failed connections
   - Long-running transactions

### Monitoring Implementation

```python
# Example monitoring code for Neon usage
import asyncio
import asyncpg
from typing import Dict, Any

class NeonMonitor:
    def __init__(self, connection_pool):
        self.pool = connection_pool
        self.metrics = {
            "queries": 0,
            "query_time_total": 0.0,
            "active_connections": 0
        }
    
    async def monitored_execute(self, query: str, *args, **kwargs):
        start_time = time.time()
        try:
            result = await self.pool.execute(query, *args, **kwargs)
            self.metrics["queries"] += 1
            query_time = time.time() - start_time
            self.metrics["query_time_total"] += query_time
            return result
        except Exception as e:
            # Log errors for monitoring
            self.log_error("execute", str(e))
            raise
    
    def get_metrics(self) -> Dict[str, Any]:
        avg_query_time = (
            self.metrics["query_time_total"] / self.metrics["queries"]
            if self.metrics["queries"] > 0 else 0
        )
        return {
            "total_queries": self.metrics["queries"],
            "average_query_time": avg_query_time,
            "active_connections": self.pool._queue.qsize() if hasattr(self.pool, '_queue') else 0
        }
```

## System-wide Monitoring

### Dashboard Implementation

For a comprehensive view, implement a monitoring dashboard with:

1. Real-time metrics for Qdrant and Neon
2. API response times and success rates
3. Error rates and common failure types
4. System resource utilization
5. Cost tracking for API usage

### Alerting

Set up alerts for:

- Response times exceeding 5-second threshold
- High error rates (>5%)
- Database connection failures
- Vector database unavailability
- API quota approaching limits