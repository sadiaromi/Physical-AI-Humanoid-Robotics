# Monitoring Dashboard for Query Performance

This document describes the implementation of a monitoring dashboard for tracking query performance in the RAG Chatbot system.

## Dashboard Components

### 1. Real-Time Query Metrics

#### Response Time Metrics
- Current average response time
- 95th percentile response time
- Maximum response time in the last minute
- Distribution of response times

#### Throughput Metrics
- Queries per minute (QPM)
- Queries per second (QPS)
- Total queries processed in the last hour/day

#### Error Metrics
- Error rate percentage
- Types of errors encountered
- Error rate over time

### 2. System Resource Metrics

#### API Usage
- Cohere API token usage
- Qdrant vector database usage
- Neon Postgres database connections

#### Performance Indicators
- API endpoint response times
- Database query times
- Vector search performance

### 3. Business Metrics

#### Usage Analytics
- Number of active sessions
- Unique users (anonymized)
- Popular books being queried
- Query patterns over time

## Implementation Approach

### Backend Metrics Collection

```python
# Example metrics collection code
import time
from dataclasses import dataclass
from typing import Dict, List
import asyncio
from datetime import datetime


@dataclass
class QueryMetrics:
    timestamp: datetime
    response_time: float
    success: bool
    book_id: str
    query_complexity: str  # basic, detailed, inference


class MetricsCollector:
    def __init__(self):
        self.metrics: List[QueryMetrics] = []
        self.metrics_lock = asyncio.Lock()
    
    async def record_query(self, response_time: float, success: bool, book_id: str, query_complexity: str):
        async with self.metrics_lock:
            metric = QueryMetrics(
                timestamp=datetime.now(),
                response_time=response_time,
                success=success,
                book_id=book_id,
                query_complexity=query_complexity
            )
            self.metrics.append(metric)
    
    async def get_recent_metrics(self, minutes: int = 5) -> Dict:
        """Get metrics from the last specified minutes"""
        async with self.metrics_lock:
            cutoff_time = datetime.now().timestamp() - (minutes * 60)
            recent_metrics = [m for m in self.metrics if m.timestamp.timestamp() > cutoff_time]
            
            if not recent_metrics:
                return {}
            
            response_times = [m.response_time for m in recent_metrics]
            avg_response_time = sum(response_times) / len(response_times)
            
            successful_queries = [m for m in recent_metrics if m.success]
            success_rate = len(successful_queries) / len(recent_metrics) * 100
            
            return {
                "total_queries": len(recent_metrics),
                "avg_response_time": avg_response_time,
                "success_rate": success_rate,
                "recent_errors": len(recent_metrics) - len(successful_queries)
            }
```

### Dashboard Frontend

For the dashboard interface, you could implement:

1. A web-based dashboard using a framework like Dash, Streamlit, or Grafana
2. Real-time charts and graphs for key metrics
3. Historical trend analysis
4. Alerting for performance thresholds

### Metrics Storage

The metrics could be stored in:

1. A time-series database like InfluxDB
2. A simple database table for short-term metrics
3. A monitoring service like Prometheus with Grafana visualization

### Alerting System

Set up alerts for:

- Response times exceeding 5 seconds for more than 5% of queries
- Error rates exceeding 5%
- Sudden drops in query volume (potential service issues)
- High resource utilization

## Sample Dashboard Layout

```
┌─────────────────────────────────────────────────────────┐
│                    RAG Chatbot Dashboard                │
├─────────────────┬─────────────────┬─────────────────────┤
│   Response Time │   Throughput    │      Errors         │
│                 │                 │                     │
│   Avg: 1.2s     │   QPM: 42       │   Error Rate: 0.8%  │
│   P95: 2.5s     │   QPS: 0.7      │   Recent: 3         │
│   Max: 4.1s     │                 │                     │
├─────────────────┼─────────────────┼─────────────────────┤
│              System Resource Usage                      │
│                                                         │
│  Cohere Usage: ██ 35%    Qdrant Usage: ████ 42%        │
│  Neon Usage: ███ 28%     API Tokens: ████ 45%          │
├─────────────────────────────────────────────────────────┤
│                     Business Metrics                    │
│  Active Sessions: 24    Popular Books: [BookA, BookB]  │
│  Avg Queries/User: 3.2  Peak Hours: [10-11AM, 2-3PM]   │
└─────────────────────────────────────────────────────────┘
```

## Implementation Notes

1. Metrics should be collected without impacting query performance
2. Sensitive information should not be stored in metrics
3. Historical data should be aggregated to preserve storage space
4. The dashboard should be accessible only to authorized personnel
5. Regular cleanup of old metrics is necessary to manage storage