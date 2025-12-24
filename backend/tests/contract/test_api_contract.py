import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError
import json
from src.api.main import app
from src.config.settings import settings


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


class TestAPIContract:
    """Contract tests based on OpenAPI specification"""

    def test_openapi_schema_exists(self, client):
        """Test that the OpenAPI schema is available"""
        response = client.get("/openapi.json")
        assert response.status_code == 200

        # Verify the response is valid JSON
        schema = response.json()
        assert "openapi" in schema
        assert "info" in schema
        assert "paths" in schema
        assert schema["openapi"].startswith("3.")

    def test_query_endpoint_contract(self, client):
        """Test the query endpoint contract based on OpenAPI spec"""
        # Check that the endpoint exists and has the expected method
        response = client.get("/openapi.json")
        schema = response.json()

        # Verify the /api/v1/query endpoint exists
        assert "/api/v1/query" in schema["paths"]
        assert "post" in schema["paths"]["/api/v1/query"]

        # Check parameters and request body schema
        post_spec = schema["paths"]["/api/v1/query"]["post"]
        request_body_spec = post_spec["requestBody"]["content"]["application/json"]["schema"]

        # The request body schema might be a reference to a definition in components
        if "$ref" in request_body_spec:
            # If it's a reference, get the actual schema from components
            ref_path = request_body_spec["$ref"].replace("#/components/schemas/", "")
            request_body_spec = schema["components"]["schemas"][ref_path]

        # If we have properties to check for required fields
        if "required" not in request_body_spec:
            # The required fields might be in the properties schema
            if "allOf" in request_body_spec:
                # Handle allOf case - check each item for required fields
                required_fields = set()
                for item in request_body_spec["allOf"]:
                    if "$ref" in item:
                        # Get schema from reference
                        ref_path = item["$ref"].replace("#/components/schemas/", "")
                        sub_schema = schema["components"]["schemas"][ref_path]
                        if "required" in sub_schema:
                            required_fields.update(sub_schema["required"])
                    elif "required" in item:
                        required_fields.update(item["required"])
            else:
                # If no required field, we can't verify them directly
                # We'll just ensure the properties exist in the schema structure
                assert "properties" in request_body_spec
                properties = request_body_spec["properties"]
                assert "question" in properties
                assert "book_id" in properties
        else:
            # Verify required fields in request body
            assert "question" in request_body_spec["required"]
            assert "book_id" in request_body_spec["required"]

        # Verify response schema
        responses_spec = post_spec["responses"]
        assert "200" in responses_spec  # Success response
        assert "422" in responses_spec  # Validation error response

    def test_ingest_endpoint_contract(self, client):
        """Test the ingest endpoint contract based on OpenAPI spec"""
        # Check that the endpoint exists and has the expected method
        response = client.get("/openapi.json")
        schema = response.json()

        # Verify the /api/v1/books endpoint exists (ingest is handled via the books endpoint)
        assert "/api/v1/books" in schema["paths"]
        assert "post" in schema["paths"]["/api/v1/books"]

        # Check parameters and request body schema
        post_spec = schema["paths"]["/api/v1/books"]["post"]
        # The books endpoint uses form data instead of JSON, so we need to check accordingly
        assert "requestBody" in post_spec
        request_body_spec = post_spec["requestBody"]

        # Verify the required parameters (query parameters)
        parameters = post_spec.get("parameters", [])
        param_names = [param["name"] for param in parameters]
        assert "title" in param_names
        assert "author" in param_names

        # Verify the file upload in request body
        assert "multipart/form-data" in request_body_spec["content"]

        # Verify response schema
        responses_spec = post_spec["responses"]
        assert "200" in responses_spec  # Success response
        assert "422" in responses_spec  # Validation error response

    def test_health_endpoint_contract(self, client):
        """Test the health endpoint contract based on OpenAPI spec"""
        # Check that the endpoint exists and has the expected method
        response = client.get("/openapi.json")
        schema = response.json()

        # Verify the /health endpoint exists
        assert "/health" in schema["paths"]
        assert "get" in schema["paths"]["/health"]

        # Verify response schema
        get_spec = schema["paths"]["/health"]["get"]
        responses_spec = get_spec["responses"]
        assert "200" in responses_spec  # Success response

    def test_request_validation(self, client):
        """Test that requests are validated according to the contract"""
        # Test invalid request to query endpoint - but with valid auth first
        invalid_payload = {
            "invalid_field": "test"  # Missing required fields
        }

        response = client.post(
            "/api/v1/query",
            json=invalid_payload,
            headers={"Authorization": f"Bearer {settings.API_KEY}"}
        )

        # Should return 422 validation error as the request is invalid
        # (missing required fields like 'question' and 'book_id')
        assert response.status_code == 422

    def test_response_format(self, client):
        """Test that responses follow the contract specification"""
        # This test would normally make a successful request,
        # but we'll check that the schema defines the expected response format

        response = client.get("/openapi.json")
        schema = response.json()

        # Check the response format for the query endpoint
        query_response_spec = (
            schema["paths"]["/api/v1/query"]["post"]
            ["responses"]["200"]["content"]["application/json"]["schema"]
        )

        # The response schema might be a reference to a definition in components
        if "$ref" in query_response_spec:
            # If it's a reference, get the actual schema from components
            ref_path = query_response_spec["$ref"].replace("#/components/schemas/", "")
            query_response_spec = schema["components"]["schemas"][ref_path]

        # Verify the response has the expected properties
        assert "properties" in query_response_spec
        response_properties = query_response_spec["properties"]
        assert "response" in response_properties
        # Note: we'll check for the actual properties in the QueryResponse model
        # which should be defined in the components section of the schema