import time
from typing import Dict
from fastapi import Request, HTTPException, status
from src.config.settings import settings
from collections import defaultdict


class RateLimiter:
    def __init__(self):
        # Dictionary to store request times for each user/IP
        # Key: user identifier, Value: list of request timestamps
        self.requests: Dict[str, list] = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed based on rate limits
        
        Args:
            identifier: A string identifying the requester (e.g., IP address or user ID)
            
        Returns:
            True if the request is allowed, False otherwise
        """
        now = time.time()
        window_start = now - settings.rate_limit_window
        
        # Remove requests that are outside the current window
        self.requests[identifier] = [
            req_time for req_time in self.requests[identifier] 
            if req_time > window_start
        ]
        
        # Check if the number of requests is within the limit
        if len(self.requests[identifier]) < settings.rate_limit_requests:
            # Add the current request
            self.requests[identifier].append(now)
            return True
        else:
            # Rate limit exceeded
            return False


# Create a global rate limiter instance
rate_limiter = RateLimiter()


def check_rate_limit(request: Request) -> bool:
    """
    Dependency to check rate limits for an incoming request
    Uses the client's IP address as the identifier
    """
    # Get the client's IP address from the request
    client_ip = request.client.host
    
    # Check if the request is allowed based on rate limits
    if not rate_limiter.is_allowed(client_ip):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Rate limit exceeded"
        )
    
    return True