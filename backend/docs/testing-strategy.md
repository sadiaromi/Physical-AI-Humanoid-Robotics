# Test Suite for RAG Chatbot Backend

This project includes several types of tests to ensure quality and reliability:

## Unit Tests
Located in: `tests/unit/`
- Test individual functions and methods in isolation
- Validate business logic in services
- Test utility functions
- Test data models

## Integration Tests
Located in: `tests/integration/`
- Test API endpoints with mocked external services
- Test database operations
- Test service interactions
- Test end-to-end workflows

## Contract Tests
Located in: `tests/contract/`
- Validate API compliance with OpenAPI specification
- Ensure backward compatibility
- Validate request/response schemas

## Performance Tests
Located in: `tests/performance/`
- Test response times
- Test concurrent request handling
- Validate rate limiting
- Assess resource consumption

## Test Execution Commands

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test types
pytest tests/unit/
pytest tests/integration/
pytest tests/contract/

# Run with verbose output
pytest -v

# Run specific test file
pytest tests/unit/test_embedding_service.py
```

## Test Requirements

All tests must:
1. Follow pytest conventions
2. Include proper assertions
3. Have descriptive names
4. Include docstrings explaining test purpose
5. Clean up any created resources
6. Not have external dependencies where possible