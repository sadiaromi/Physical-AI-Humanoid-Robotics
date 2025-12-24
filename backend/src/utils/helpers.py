import logging
import uuid
from typing import Optional
from src.config.settings import settings


def setup_logging():
    """
    Set up basic logging configuration
    """
    logging.basicConfig(
        level=logging.DEBUG if settings.debug else logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger instance with the specified name
    """
    return logging.getLogger(name)


def generate_uuid() -> str:
    """
    Generate a new UUID string
    """
    return str(uuid.uuid4())


def truncate_text(text: str, max_length: int = 100) -> str:
    """
    Truncate text to a maximum length, adding ellipsis if truncated
    """
    if len(text) <= max_length:
        return text
    return text[:max_length-3] + "..."


def get_token_count(text: str) -> int:
    """
    Approximate token count using a simple method (words + punctuation)
    This is a simplified approach - in production, use a proper tokenizer
    """
    # Simple approximation: split on whitespace and common punctuation
    import re
    tokens = re.split(r'\s+|[,.!?;:]', text)
    # Filter out empty strings
    tokens = [token for token in tokens if token.strip()]
    return len(tokens)