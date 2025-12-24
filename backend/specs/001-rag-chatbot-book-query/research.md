# Research Summary: RAG Chatbot for Book Content Querying

## Phase 0: Technical Research and Decision Log

### Decision 1: Cohere Model Selection
- **Decision**: Use Cohere's embed-english-v3.0 for embeddings and command-r-plus for generation
- **Rationale**: Required by specification (Cohere API key provided), high accuracy for RAG applications
- **Alternatives considered**: OpenAI embeddings (prohibited by constraints), other Cohere models

### Decision 2: Vector Database Configuration
- **Decision**: Use Qdrant Cloud Free Tier with 2048-dimensional vectors
- **Rationale**: Required by specification, supports high-dimensional embeddings from Cohere
- **Alternatives considered**: Pinecone, Weaviate (prohibited by budget constraints)

### Decision 3: Database Schema for Book Content
- **Decision**: Store book metadata in Neon Postgres, content chunks in Qdrant
- **Rationale**: Separation of structured metadata vs. vector embeddings optimizes for different access patterns
- **Alternatives considered**: All in one database (less optimal for vector searches)

### Decision 4: Text Chunking Strategy
- **Decision**: Overlapping chunks of 500-800 tokens with 100-token overlap
- **Rationale**: Balances context preservation with retrieval precision for book content
- **Alternatives considered**: Fixed-length sentences, paragraph-based chunks

### Decision 5: API Authentication Method
- **Decision**: API key-based authentication
- **Rationale**: Simplicity and alignment with backend service requirements; can scale to JWT later
- **Alternatives considered**: JWT tokens, OAuth2 (more complex for initial implementation)

### Decision 6: Rate Limiting Implementation
- **Decision**: Token bucket algorithm for rate limiting
- **Rationale**: Fair and predictable rate limiting that handles bursts appropriately
- **Alternatives considered**: Fixed window counter, sliding window log

### Decision 7: FastAPI Application Structure
- **Decision**: Modular approach with separate route files and service layer
- **Rationale**: Maintains clean separation of concerns and testability
- **Alternatives considered**: Monolithic approach (harder to maintain)

### Decision 8: Testing Strategy
- **Decision**: 100% coverage with unit, integration, and contract tests
- **Rationale**: Required by specification, ensures quality and reliability
- **Alternatives considered**: Lower coverage requirements (insufficient for production)

### Decision 9: Embedding Generation Process
- **Decision**: Batch processing for initial ingestion, single embeddings for queries
- **Rationale**: Efficient for large book content during ingestion while keeping queries responsive
- **Alternatives considered**: Real-time embedding for all content (too slow for ingestion)

### Decision 10: Caching Strategy
- **Decision**: No caching for initial implementation (to ensure accuracy)
- **Rationale**: RAG responses vary significantly, caching may serve stale/incorrect responses
- **Alternatives considered**: Redis caching (adds complexity without clear benefit)