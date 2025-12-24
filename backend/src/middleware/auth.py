from fastapi import HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from src.config.settings import settings
from functools import wraps
import logging

logger = logging.getLogger(__name__)


class APIKeyValidator:
    def __init__(self):
        self.api_key = settings.api_key
        self.security = HTTPBearer()

    def validate_api_key(self, credentials: HTTPAuthorizationCredentials = None):
        """
        Validate the API key from the Authorization header
        """
        if credentials and credentials.credentials == self.api_key:
            return True
        else:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid API key"
            )


# Create a global instance
api_key_validator = APIKeyValidator()


def require_api_key(request: Request):
    """
    Dependency to enforce API key validation on specific endpoints
    """
    auth_header = request.headers.get("Authorization")

    logger.info(f"Authorization header: {auth_header}")

    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid Authorization header"
        )

    token = auth_header[7:].strip()  # Remove "Bearer " prefix and strip whitespace
    expected_token = settings.api_key.strip()  # Strip whitespace from expected token

    logger.info(f"Received token: {token[:10]}...")  # Log just the first 10 chars for security
    logger.info(f"Expected token: {expected_token[:10]}...")  # Log just the first 10 chars for security
    logger.info(f"Token lengths - Received: {len(token)}, Expected: {len(expected_token)}")  # Log token lengths for debugging

    # More detailed logging to help debug character-by-character differences
    logger.info(f"Full received token length: {len(token)}")
    logger.info(f"Full expected token length: {len(expected_token)}")

    # Compare a few more characters to see where they differ (if they do)
    min_len = min(len(token), len(expected_token))
    for i in range(min(min_len, 20)):  # Check first 20 characters
        if token[i] != expected_token[i]:
            logger.warning(f"Character difference at position {i}: received '{token[i]}' vs expected '{expected_token[i]}'")
            break

    if token != expected_token:
        logger.warning("API key validation failed - tokens do not match")
        logger.warning(f"Full received token (first 50 chars): {repr(token[:50])}")
        logger.warning(f"Full expected token (first 50 chars): {repr(expected_token[:50])}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key"
        )

    # If we get here, the API key is valid
    logger.info("API Key validation successful")
    return True