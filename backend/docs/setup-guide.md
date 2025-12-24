# 1-Hour Reproducible Setup Guide

This guide provides step-by-step instructions to set up the RAG Chatbot for Book Content Querying from scratch in under 1 hour.

## Prerequisites

- Python 3.11 or higher
- Docker and Docker Compose
- Git
- An environment where you can install Python packages

## Step 1: Clone the Repository (5 minutes)

```bash
git clone https://github.com/[your-org]/humanoid-robotics-book.git
cd humanoid-robotics-book/backend
```

## Step 2: Set Up Python Environment (10 minutes)

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install dependencies
pip install -r requirements.txt
pip install -r requirements-dev.txt
```

## Step 3: Configure Environment Variables (10 minutes)

```bash
# Copy the example environment file
cp .env.example .env
```

Now edit the `.env` file with your actual values:
- `COHERE_API_KEY`: Your Cohere API key (get from https://dashboard.cohere.ai/api-keys)
- `NEON_DATABASE_URL`: Your Neon Postgres connection string (get from Neon dashboard)
- `QDRANT_API_KEY`: Your Qdrant Cloud API key (get from Qdrant dashboard)
- `QDRANT_HOST`: Your Qdrant Cloud cluster URL (get from Qdrant dashboard)
- `API_KEY`: A secure API key for your application

## Step 4: Run with Docker Compose (15 minutes)

The application can run entirely with Docker Compose, which will handle all services:

```bash
# Build and run services
docker-compose up --build
```

This will start:
- The FastAPI application
- Any required services defined in docker-compose.yml

## Step 5: Alternative: Run Locally (15 minutes)

If you prefer to run locally instead of with Docker:

```bash
# Make sure your virtual environment is activated
# and your .env file is properly configured

# Run the application directly
uvicorn src.api.main:app --reload --port 8000
```

## Step 6: Verify Installation (5 minutes)

1. Open your browser and navigate to `http://localhost:8000/docs`
2. You should see the FastAPI auto-generated documentation
3. Test an endpoint manually through the UI, or use curl:

```bash
curl -X GET "http://localhost:8000/health" -H "accept: application/json"
```

## Step 7: Ingest a Sample Book (10 minutes)

To test the full functionality, ingest a sample book:

```bash
# Using the CLI tool
python -m src.cli.ingest_cli ingest --file path/to/your/book.txt --title "Sample Book" --author "Sample Author"
```

## Troubleshooting

### Common Issues:

1. **Dependency installation fails**
   - Ensure you're using Python 3.11+
   - Try using a fresh virtual environment

2. **Connection errors to databases**
   - Verify your API keys and connection strings in `.env`
   - Ensure services are running (Docker containers or external services)

3. **Port already in use**
   - Kill any existing processes on port 8000: `lsof -ti:8000 | xargs kill -9` (macOS/Linux)
   - Or change the port in the uvicorn command: `--port [new_port]`

4. **API rate limits or authentication errors**
   - Verify your Cohere, Qdrant, and Neon credentials
   - Check that your API plan supports the required operations

## Next Steps

- Review the API documentation at `http://localhost:8000/docs`
- Try querying your ingested books
- See the embedding guide for integrating with frontend applications
- Review the selected text feature guide for advanced usage options

## Performance Testing

Run the performance tests to ensure the system meets requirements:

```bash
python -m pytest tests/performance/  # if performance tests exist
# or
python src/utils/performance_tester.py  # for manual performance testing
```

This setup has been tested on:
- Windows 10/11 with Python 3.11
- macOS 12+ with Python 3.11
- Ubuntu 20.04+ with Python 3.11

Estimated total time: 60 minutes or less