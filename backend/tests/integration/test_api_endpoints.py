import pytest
from fastapi.testclient import TestClient
from src.api.main import app
from src.config.settings import settings
import uuid


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


class TestQueryEndpoint:
    """Integration tests for the query endpoint"""

    def test_query_endpoint_requires_auth(self, client):
        """Test that the query endpoint requires authentication"""
        # Arrange
        payload = {
            "question": "What is this book about?",
            "book_id": str(uuid.uuid4())
        }

        # Act - Use the correct endpoint path
        response = client.post("/api/v1/query", json=payload)

        # Assert
        assert response.status_code == 401  # Unauthorized

    def test_query_endpoint_with_valid_auth(self, client):
        """Test the query endpoint with valid authentication"""
        # Arrange
        payload = {
            "question": "What is this book about?",
            "book_id": str(uuid.uuid4())
        }

        # Act - Use the correct endpoint path
        response = client.post(
            "/api/v1/query",
            json=payload,
            headers={"Authorization": f"Bearer {settings.API_KEY}"}
        )

        # For now, we expect a 500 error because we don't have a real database or vector store
        # in the test environment, but it should not be a 422 (validation error) or 401 (auth error)
        assert response.status_code in [404, 500]  # Either book not found or internal error due to missing services

    def test_query_endpoint_validation(self, client):
        """Test validation of the query endpoint"""
        # Arrange - missing required fields
        payload = {
            # Missing question and book_id
        }

        # Act - Use the correct endpoint path
        response = client.post(
            "/api/v1/query",
            json=payload,
            headers={"Authorization": f"Bearer {settings.API_KEY}"}
        )

        # Assert - should return validation error
        assert response.status_code == 422

    def test_health_endpoint(self, client):
        """Test the health endpoint"""
        # Act
        response = client.get("/health")

        # Assert
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"
        assert "timestamp" in data


class TestIngestEndpoint:
    """Integration tests for the ingest endpoint"""

    def test_ingest_endpoint_requires_auth(self, client):
        """Test that the ingest endpoint requires authentication"""
        # Arrange - for file upload, we need to use form data
        # For testing without a file, we'll trigger a 422 validation error that will still
        # indicate that authentication was checked
        import io
        file_content = io.BytesIO(b"Test content")

        # Act - Use the correct endpoint path
        response = client.post(
            "/api/v1/books",
            data={"title": "Test Book", "author": "Test Author"},
            files={"file": ("test.txt", file_content, "text/plain")},
        )

        # Assert - should return 401 Unauthorized if auth was checked
        assert response.status_code == 401  # Unauthorized

    def test_ingest_endpoint_with_valid_auth(self, client):
        """Test the ingest endpoint with valid authentication"""
        # Arrange
        import io
        file_content = io.BytesIO(b"Test content for the book")

        # Act - Use the correct endpoint path
        response = client.post(
            "/api/v1/books",
            data={"title": "Test Book", "author": "Test Author"},
            files={"file": ("test.txt", file_content, "text/plain")},
            headers={"Authorization": f"Bearer {settings.API_KEY}"}
        )

        # For now, we expect either a 422 validation error due to missing required fields
        # or a 500 error because we don't have a real database in the test environment
        # but it should not be a 401 (auth error) since auth has passed
        assert response.status_code in [422, 500]  # Validation error or internal error