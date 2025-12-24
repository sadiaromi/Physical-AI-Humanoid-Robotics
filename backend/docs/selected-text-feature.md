# Selected Text Feature Guide

This document explains how to use the selected text feature of the RAG Chatbot, which allows users to ask questions about only the specific text they have selected.

## Overview

The selected text feature enables users to highlight specific portions of text in a digital book and ask targeted questions about only that selection, rather than the entire book content. This is particularly useful for:

- Getting clarifications on complex passages
- Understanding specific concepts in detail
- Focusing on particular sections during study sessions

## How It Works

1. **Text Selection**: User selects text in the digital book interface
2. **Chatbot Activation**: User activates the chatbot to ask a question about the selected text
3. **Processing**: The system sends only the selected text along with the question to the backend
4. **Response Generation**: The backend generates a response based solely on the provided text selection

## API Usage

When using the selected text feature through the API, include the `selected_text` parameter:

```json
{
  "question": "Explain this concept",
  "book_id": "your-book-id",
  "selected_text": "The concept of machine learning involves algorithms that can learn from and make predictions on data..."
}
```

### Request Parameters

- `question` (required): The question about the selected text
- `book_id` (required): The ID of the book containing the selected text
- `selected_text` (optional): When provided, the system will only use this text to answer the question

### Response Behavior

When `selected_text` is provided:

- The RAG engine bypasses content retrieval from the vector database
- Only the provided text is used as context for response generation
- The response will be based solely on the provided text selection
- Faster response times due to shorter context processing

## Implementation Examples

### Frontend JavaScript

```javascript
// Get selected text from the user's selection
function getSelectedText() {
  return window.getSelection().toString();
}

// Send to the chatbot API with selected text
function queryWithSelectedText(question, bookId) {
  const selectedText = getSelectedText();
  
  const requestBody = {
    question: question,
    book_id: bookId,
    selected_text: selectedText || null
  };
  
  // Make the API call
  fetch('/query', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': 'Bearer ' + apiKey
    },
    body: JSON.stringify(requestBody)
  })
  .then(response => response.json())
  .then(data => {
    // Handle the response
    displayChatbotResponse(data.response);
  });
}
```

## Performance Benefits

The selected text feature offers several performance advantages:

- **Faster Processing**: No need to retrieve content from the vector database
- **Reduced Context**: Smaller context size can lead to more focused responses
- **Lower Costs**: Potentially reduced API usage for Cohere and Qdrant

## Limitations

- The response is limited to understanding the selected text only
- Complex questions requiring broader context from the book may not be answered as well
- Very short text selections may not provide sufficient context for meaningful responses

## Best Practices

1. **Appropriate Selection Size**: Select enough text to provide context for your question, typically at least a sentence or paragraph
2. **Clear Questions**: Formulate specific questions about the selected text
3. **Verification**: Remember to verify information from the chatbot against the original text

## Privacy Considerations

- Selected text is not stored permanently in the system
- Only the current session temporarily holds the selected text for processing
- No user data is retained after the session expires