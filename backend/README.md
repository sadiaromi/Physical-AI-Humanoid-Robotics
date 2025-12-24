# RAG Chatbot for Book Content Querying

This project implements a Retrieval-Augmented Generation (RAG) chatbot that allows users to query book content and receive accurate answers based on the text in the book.

## Features

- Query book content with natural language questions
- Support for multiple books
- Selected text querying mode (ask about only the highlighted text)
- Conversational context maintenance
- Secure API with authentication and rate limiting
- FastAPI-based backend with async support
- Embeddable in digital books
- Comprehensive monitoring and logging

## Tech Stack

- Python 3.11
- FastAPI for the web framework
- Cohere for embeddings and text generation
- Qdrant for vector storage
- Neon Serverless Postgres for metadata storage
- Pydantic for data validation
- pytest for testing

## Prerequisites

- Python 3.11 or higher
- Docker and Docker Compose (for containerized deployment)
- Cohere API key
- Qdrant Cloud account and cluster
- Neon Serverless Postgres account

## Setup

1. Clone the repository
2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   pip install -r requirements-dev.txt
   ```
4. Create a `.env` file based on `.env.example` and fill in your API keys:
   ```bash
   cp .env.example .env
   # Edit .env with your actual keys
   ```
5. Run the application:
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

## Running with Docker

### Development
```bash
docker-compose up --build
```

### Production
Use the production deployment script:
```bash
./deploy.sh deploy
```

## API Documentation

Once the application is running, visit `http://localhost:8000/docs` for the interactive API documentation.

## Usage

### Ingest a Book
Use the CLI tool to ingest a book:
```bash
python -m src.cli.ingest_cli ingest --file path/to/book.txt --title "Book Title" --author "Author Name"
```

### Query Book Content
Make authenticated requests to the query endpoint:
```bash
curl -X POST "http://localhost:8000/query" \
     -H "Content-Type: application/json" \
     -H "Authorization: Bearer YOUR_API_KEY" \
     -d '{
       "question": "What is the main concept of chapter 1?",
       "book_id": "your-book-id",
       "selected_text": null
     }'
```

### Query with Selected Text
To query only specific text:
```bash
curl -X POST "http://localhost:8000/query" \
     -H "Content-Type: application/json" \
     -H "Authorization: Bearer YOUR_API_KEY" \
     -d '{
       "question": "Explain this concept",
       "book_id": "your-book-id",
       "selected_text": "The concept of machine learning is..."
     }'
```

## Testing

Run all tests:
```bash
pytest
```

Run tests with coverage:
```bash
pytest --cov=src --cov-report=html
```

## Project Structure

```
backend/
├── src/
│   ├── models/          # Data models
│   ├── services/        # Business logic
│   ├── api/             # API endpoints
│   ├── config/          # Configuration
│   ├── utils/           # Utility functions
│   └── cli/             # CLI tools
├── tests/               # Test files
│   ├── unit/            # Unit tests
│   ├── integration/     # Integration tests
│   └── contract/        # Contract tests
├── docs/                # Documentation files
├── requirements.txt     # Production dependencies
├── requirements-dev.txt # Development dependencies
├── .env.example        # Environment variables template
├── Dockerfile          # Docker configuration
├── Dockerfile.production # Production Dockerfile
├── docker-compose.yml  # Docker Compose configuration
├── deploy.sh           # Production deployment script
└── README.md           # This file
```

## Performance Requirements

- 95% of queries respond in under 5 seconds
- 95%+ accuracy on 50+ test queries
- Secure authentication and rate limiting

## Documentation

- `docs/embedding-guide.md` - Guide for embedding the chatbot in digital books
- `docs/selected-text-feature.md` - Documentation for the selected text feature
- `docs/setup-guide.md` - 1-hour reproducible setup guide
- `docs/testing-strategy.md` - Testing approach and methodology
- `docs/accuracy-validation.md` - Approach to validate response accuracy
- `docs/monitoring-guide.md` - Implementation of monitoring for Qdrant and Neon
- `docs/security-testing.md` - Security testing procedures
- `docs/code-review-checklist.md` - Code review checklist against specifications
- `docs/monitoring-dashboard.md` - Monitoring dashboard for query performance
- `docs/handoff-documentation.md` - Complete handoff documentation

## Support

For issues or questions, please refer to the documentation in the `docs/` directory or contact the development team.