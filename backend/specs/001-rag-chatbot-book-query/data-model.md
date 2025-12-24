# Data Model: RAG Chatbot for Book Content Querying

## Entities and Relationships

### Book
- **book_id**: UUID (primary key)
- **title**: String (required, max 500 chars)
- **author**: String (max 200 chars)
- **isbn**: String (up to 17 chars, optional)
- **publication_date**: Date (optional)
- **language**: String (default "en")
- **created_at**: DateTime (auto-generated)
- **updated_at**: DateTime (auto-generated)
- **status**: Enum ["processing", "ready", "archived"]
- **metadata**: JSON (additional book info)

### BookContentChunk
- **chunk_id**: UUID (primary key)
- **book_id**: UUID (foreign key to Book)
- **content**: Text (the actual text chunk, max 10000 chars)
- **chunk_index**: Integer (sequential number for ordering)
- **section_title**: String (optional, max 200 chars)
- **page_number**: Integer (optional)
- **token_count**: Integer (number of tokens in chunk)
- **vector_id**: String (ID in Qdrant vector database)
- **created_at**: DateTime (auto-generated)

### QuerySession
- **session_id**: UUID (primary key)
- **user_id**: String (for tracking, optional if anonymous)
- **book_id**: UUID (foreign key to Book)
- **created_at**: DateTime (auto-generated)
- **updated_at**: DateTime (auto-generated)
- **expires_at**: DateTime (session expiration)

### QueryHistory
- **query_id**: UUID (primary key)
- **session_id**: UUID (foreign key to QuerySession)
- **question**: Text (user's question)
- **response**: Text (AI-generated response)
- **context_chunks**: Array of UUID (vector IDs used to generate response)
- **timestamp**: DateTime (auto-generated)
- **was_helpful**: Boolean (user feedback, optional)

## Relationships
- Book (1) → BookContentChunk (Many) via book_id
- Book (1) → QuerySession (Many) via book_id
- QuerySession (1) → QueryHistory (Many) via session_id

## Validation Rules
- Book.title must not be empty
- BookContentChunk.content must be between 100-8000 characters
- BookContentChunk.chunk_index must be unique within a book
- QuerySession.expires_at must be at least 1 hour after created_at
- All foreign key relationships must reference existing records

## State Transitions
- Book.status transitions: ["processing" → "ready"], ["ready" → "archived"]
- Book.content cannot be modified after status becomes "ready"