# Embedding Guide: RAG Chatbot in Digital Books

This guide explains how to embed the RAG Chatbot into digital books without affecting layout or performance.

## Prerequisites

- Access to the RAG Chatbot API
- Valid API key for authentication
- Knowledge of the target book's ID in the system

## Frontend Integration

The chatbot can be embedded in digital books using a JavaScript widget that communicates with the backend API. Here's how to implement it:

### 1. Widget Initialization

```html
<div id="chatbot-container"></div>
<script src="path/to/chatbot-widget.js"></script>
<script>
  const chatbot = new RAGChatbot({
    containerId: 'chatbot-container',
    apiUrl: 'https://your-api-endpoint.com',
    apiKey: 'your-api-key',
    bookId: 'target-book-uuid'
  });
  
  chatbot.init();
</script>
```

### 2. User Interaction

The widget supports the following interactions:

- **Full Book Queries**: Users can ask questions about the entire book content
- **Selected Text Queries**: When users highlight text and activate the chatbot, it will only use that text to answer questions
- **Context Preservation**: Conversations maintain context within a session

### 3. Performance Considerations

The widget is designed to be lightweight and not impact page load times:

- Asynchronous loading of JavaScript assets
- Caching of frequently accessed content chunks
- Lazy loading of chat history to avoid unnecessary API calls

## API Integration

For direct API integration, use the following endpoints:

### Query Endpoint
```
POST /query
Headers:
  Content-Type: application/json
  Authorization: Bearer {API_KEY}
  
{
  "question": "What is the main concept of chapter 1?",
  "book_id": "your-book-id",
  "selected_text": null  // Optional: when null, uses full book context
}
```

### Response Format
```json
{
  "response": "The main concept of chapter 1 is...",
  "context_used": ["chunk_id_1", "chunk_id_2"],
  "session_id": "session-uuid"
}
```

## Styling

The widget can be customized to match your book's design:

```css
.rag-chatbot-container {
  /* Customize appearance */
  border: 1px solid #ccc;
  border-radius: 8px;
  max-width: 500px;
  position: fixed;
  bottom: 20px;
  right: 20px;
}

.rag-chatbot-header {
  /* Customize header */
  background-color: #f5f5f5;
  padding: 10px;
  border-top-left-radius: 8px;
  border-top-right-radius: 8px;
}
```

## Security

- All API requests must include a valid API key
- Rate limiting is enforced to prevent abuse
- User query history is not stored to maintain privacy
- SSL encryption is required for all communications

## Troubleshooting

### Widget Not Loading
- Verify that the API endpoint is accessible
- Check that the API key is valid
- Ensure the book ID exists in the system

### Slow Response Times
- Check your network connection
- Verify that the Cohere API is responding promptly
- Confirm that the Qdrant vector database is accessible