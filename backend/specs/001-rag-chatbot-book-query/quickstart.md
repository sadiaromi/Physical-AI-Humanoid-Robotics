# Quickstart Guide: RAG Chatbot Backend

## Prerequisites
- Python 3.11 or higher
- Docker and Docker Compose (for local development)
- Cohere API key
- Qdrant Cloud account and cluster
- Neon Serverless Postgres account

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd backend
   ```

2. **Create virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   pip install -r requirements-dev.txt
   ```

4. **Configure environment variables**
   ```bash
   cp .env.example .env
   ```
   
   Edit `.env` and set:
   - `COHERE_API_KEY` - Your Cohere API key
   - `NEON_DATABASE_URL` - Your Neon Postgres connection string
   - `QDRANT_API_KEY` - Your Qdrant Cloud API key
   - `QDRANT_HOST` - Your Qdrant Cloud cluster URL
   - `QDRANT_PORT` - Default is 6333, change if needed
   - `API_KEY` - For securing API endpoints

5. **Run database migrations** (if applicable)
   ```bash
   python -m src.cli.ingest_cli init_db
   ```

## Ingest a Book

1. **Prepare your book content** as a text file or PDF

2. **Run the ingestion script**
   ```bash
   python -m src.cli.ingest_cli ingest --file path/to/book.txt --title "Book Title" --author "Author Name"
   ```

3. **Monitor the ingestion process**
   The script will:
   - Split the book into chunks (500-800 tokens)
   - Generate embeddings using Cohere
   - Store chunks in Qdrant vector database
   - Store metadata in Neon Postgres

## Start the API Server

1. **Run the FastAPI server**
   ```bash
   uvicorn src.api.main:app --reload --port 8000
   ```

2. **Access the API documentation**
   - Open http://localhost:8000/docs in your browser
   - The Swagger UI provides interactive API documentation

## Test the API

1. **Query the chatbot with an authenticated request**
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

2. **Query with specific text selected**
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

## Run Tests

1. **Run all tests**
   ```bash
   pytest
   ```

2. **Run with coverage**
   ```bash
   pytest --cov=src --cov-report=html
   ```

## Docker Deployment

1. **Build and run with Docker Compose**
   ```bash
   docker-compose up --build
   ```

## Troubleshooting

- **API Rate Limits**: If you encounter rate limits, verify your Cohere plan allows sufficient requests
- **Qdrant Connection**: Ensure your Qdrant Cloud cluster is accessible and credentials are correct
- **Database Connection**: Verify your Neon connection string is correct and database is active
- **Embedding Issues**: Check that your Cohere API key is valid and has embedding permissions