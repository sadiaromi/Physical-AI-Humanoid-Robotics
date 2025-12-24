# Project Handoff: RAG Chatbot for Book Content Querying

## Project Overview

The RAG Chatbot for Book Content Querying is a backend service that enables users to ask questions about book content and receive accurate answers based on the text. The system uses Retrieval Augmented Generation (RAG) with Cohere for embeddings and text generation, backed by Qdrant for vector storage and Neon Postgres for metadata.

### Key Features
1. Query book content with natural language questions
2. Embed chatbot functionality into digital books
3. Process user-selected text queries
4. Maintain conversation context within sessions
5. Secure API with rate limiting

### Architecture
- **Backend**: Python 3.11 with FastAPI
- **Embeddings**: Cohere embed-english-v3.0
- **Generation**: Cohere command-r-plus
- **Vector DB**: Qdrant Cloud
- **Metadata DB**: Neon Serverless Postgres
- **Authentication**: API key-based
- **Rate Limiting**: Token bucket algorithm

## Deployment Instructions

### Prerequisites
- Python 3.11+
- Docker and Docker Compose
- Access to Cohere API
- Qdrant Cloud cluster
- Neon Postgres database

### Production Deployment
1. Create `.env.production` with required environment variables:
   - `COHERE_API_KEY`
   - `NEON_DATABASE_URL`
   - `QDRANT_API_KEY`
   - `QDRANT_HOST`
   - `API_KEY`
   - Any other required variables

2. Run the deployment script:
   ```bash
   ./deploy.sh deploy
   ```

3. Verify the deployment:
   ```bash
   ./deploy.sh status
   ```

### Environment Configuration
See `.env.example` for all required environment variables and their descriptions.

## System Components

### Models (src/models/)
- `book.py`: Book entity with metadata
- `book_content_chunk.py`: Text chunks with vector IDs
- `query_session.py`: Session management
- `query_history.py`: Query history (metadata only)
- `query.py`: Request/response models

### Services (src/services/)
- `rag_engine.py`: Core RAG functionality
- `embedding_service.py`: Handles text embedding
- `vector_store.py`: Manages vector database operations
- `cohere_client.py`: Cohere API wrapper

### API Routes (src/api/routes/)
- `query.py`: Query endpoint
- `ingest.py`: Book ingestion endpoint
- `main.py`: Application entry point

### Middlewares (src/middleware/)
- `auth.py`: Authentication handling
- `rate_limit.py`: API rate limiting

## Security Considerations

- All API endpoints require authentication via API key
- Rate limiting prevents abuse
- User query history is not permanently stored
- Input validation prevents injection attacks
- HTTPS is required for all communications

## Performance Requirements

- 95% of queries respond in under 5 seconds
- 95%+ accuracy on test query dataset
- Support for concurrent users
- Efficient vector database queries

## Monitoring and Maintenance

### Metrics Tracked
- API response times
- Query success/failure rates
- Database performance
- Vector store performance
- API token usage

### Logs
- Application logs in structured format
- Error logs with stack traces
- Performance metrics logs

### Maintenance Tasks
- Regular review of API usage costs
- Database maintenance and cleanup
- Performance monitoring and optimization
- Security updates for dependencies

## Troubleshooting

### Common Issues
- **Slow Response Times**: Check Cohere API and Qdrant Cloud availability
- **Connection Errors**: Verify API keys and connection strings
- **Rate Limits**: Check account limits with service providers
- **Validation Errors**: Ensure requests match API specifications

### Debugging
- Check application logs for detailed error information
- Verify environment variables are correctly set
- Use the API documentation at `/docs` for request format

## Future Enhancements

1. Caching layer for frequently asked questions
2. Support for additional document formats (PDF, EPUB)
3. Advanced analytics and user behavior insights
4. Multi-language support
5. User account system with personalized experiences

## Documentation References

- `docs/embedding-guide.md` - Guide for embedding the chatbot in digital books
- `docs/selected-text-feature.md` - Documentation for the selected text feature
- `docs/setup-guide.md` - 1-hour reproducible setup guide
- `docs/testing-strategy.md` - Testing approach and methodology
- `docs/accuracy-validation.md` - Approach to validate response accuracy
- `docs/monitoring-guide.md` - Implementation of monitoring for Qdrant and Neon
- `docs/security-testing.md` - Security testing procedures
- `docs/code-review-checklist.md` - Code review checklist against specifications
- `docs/monitoring-dashboard.md` - Monitoring dashboard for query performance

## Testing

### Unit Tests
Located in `tests/unit/`, covering individual service functions.

### Integration Tests
Located in `tests/integration/`, covering API endpoints and service interactions.

### Contract Tests
Located in `tests/contract/`, validating API compliance.

### Performance and Accuracy Tests
- Performance tester: `src/utils/performance_tester.py`
- Accuracy validator: `src/utils/accuracy_validator.py`
- Test dataset: `tests/accuracy/test_dataset.json`

## Support and Contacts

For issues or questions regarding this implementation, refer to:
- Code documentation and comments in the source files
- API documentation at `/docs` endpoint
- This handoff document for system overview and troubleshooting