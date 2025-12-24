# Performance Testing Script for RAG Chatbot

This script tests the performance of the RAG chatbot API endpoints to ensure responses are delivered within the required 5-second limit.

## Test Scenarios

1. Query response time with different query complexities
2. Query response time with different book sizes
3. Concurrent query handling
4. Vector search performance
5. API rate limiting effectiveness

## Test Implementation

We need to implement automated performance tests that validate:

- 95% of queries respond in under 5 seconds
- Performance consistency across different book content sizes
- Proper handling of concurrent requests
- Rate limiting doesn't impact legitimate usage

## Metrics to Track

- Average response time
- 95th percentile response time
- Error rate
- Throughput (queries per second)
- Resource utilization (API tokens, vector database usage)

## Recommended Tools

- Apache Bench (ab) or wrk for load testing
- Custom Python scripts using the requests library for specific scenarios
- Monitoring tools to track API and database performance