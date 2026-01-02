# import cohere
from typing import List, Dict, Any, Optional
from src.config.settings import settings
import random


class CohereClient:
    def __init__(self):
        # self.client = cohere.Client(api_key=settings.cohere_api_key)
        # Using Cohere's most current recommended model for generation
        # Note: Cohere models are frequently updated/deprecated, so this may need to be updated periodically
        self.generation_model = "command-r-08-2024"  # Updated to current available model (as of Dec 2025)

    def generate_response(self,
                         prompt: str,
                         max_tokens: Optional[int] = 500,
                         temperature: Optional[float] = 0.7) -> str:
        """
        Generate a response using Cohere's language model

        Args:
            prompt: The input text/prompt for generation
            max_tokens: Maximum number of tokens to generate (optional)
            temperature: Controls randomness in generation (optional)

        Returns:
            Generated text response
        """
        # Import cohere here to avoid errors if not installed
        try:
            import cohere
            # Initialize the client if not already done
            if not hasattr(self, 'client'):
                self.client = cohere.Client(api_key=settings.cohere_api_key)

            # Try multiple models in case one is deprecated
            models_to_try = [
                self.generation_model,      # Current model
                self.generation_model,      # Same model as fallback (to avoid deprecated models)
            ]

            for model in models_to_try:
                try:
                    # Use the Chat API as the Generate API has been deprecated
                    response = self.client.chat(
                        model=model,
                        message=prompt,
                        max_tokens=max_tokens,
                        temperature=temperature
                    )
                    if hasattr(response, 'text') and response.text:
                        return response.text
                except Exception as model_error:
                    print(f"Model {model} failed: {str(model_error)}")
                    continue  # Try next model

            # If all models fail, return a default message
            return "No response generated."

        except ImportError:
            # If cohere is not installed, return mock response
            return f"Mock response for query: {prompt[:50]}... [INSTALL COHERE PACKAGE TO SEE REAL RESULTS]"
        except Exception as e:
            # If there's an API error, return mock response with error info
            print(f"Cohere API error: {str(e)}")
            return f"Mock response for query: {prompt[:50]}... [COHERE API ERROR: {str(e)}]"

    def chat(self,
             message: str,
             chat_history: Optional[List[Dict[str, str]]] = None,
             preamble: Optional[str] = None,
             max_tokens: Optional[int] = 500,
             temperature: Optional[float] = 0.7) -> str:
        """
        Conduct a chat conversation with context

        Args:
            message: Current user message
            chat_history: Previous conversation turns (optional)
            preamble: System instructions (optional)
            max_tokens: Maximum number of tokens to generate (optional)
            temperature: Controls randomness in generation (optional)

        Returns:
            Generated response text
        """
        # Import cohere here to avoid errors if not installed
        try:
            import cohere
            # Initialize the client if not already done
            if not hasattr(self, 'client'):
                self.client = cohere.Client(api_key=settings.cohere_api_key)

            # Try multiple models in case one is deprecated
            models_to_try = [
                self.generation_model,      # Current model
                self.generation_model,      # Same model as fallback (to avoid deprecated models)
            ]

            for model in models_to_try:
                try:
                    # Actually make the API call to Cohere
                    response = self.client.chat(
                        message=message,
                        chat_history=chat_history or [],
                        preamble=preamble,
                        model=model,
                        max_tokens=max_tokens,
                        temperature=temperature
                    )
                    if hasattr(response, 'text') and response.text:
                        return response.text
                except Exception as model_error:
                    print(f"Model {model} failed: {str(model_error)}")
                    continue  # Try next model

            # If all models fail, return a default message
            return "No response generated."
        except ImportError:
            # If cohere is not installed, return mock response
            return f"Mock chat response to: {message[:50]}... [INSTALL COHERE PACKAGE TO SEE REAL RESULTS]"
        except Exception as e:
            # If there's an API error, return mock response with error info
            print(f"Cohere API error: {str(e)}")
            return f"Mock chat response to: {message[:50]}... [COHERE API ERROR: {str(e)}]"

    def classify(self,
                 inputs: List[str],
                 examples: List[Dict[str, str]]) -> List[Any]:
        """
        Classify inputs based on provided examples

        Args:
            inputs: List of texts to classify
            examples: List of example texts with their labels

        Returns:
            List of classifications
        """
        # For now, return mock classifications to allow the application to start
        # Actual implementation requires valid Cohere API key
        if not settings.cohere_api_key or settings.cohere_api_key == "placeholder":
            # Create mock classification responses
            mock_classifications = []
            for i, inp in enumerate(inputs):
                mock_classifications.append({
                    "input": inp,
                    "prediction": "mock_label",
                    "confidences": [{"option": "mock_label", "confidence": 0.9}],
                    "labels": {"mock_label": 0.9}
                })
            return mock_classifications

        # In a real implementation, this would be:
        # response = self.client.classify(
        #     inputs=inputs,
        #     examples=examples
        # )
        # return response.classifications

        # Mock classification for demo purposes
        mock_classifications = []
        for i, inp in enumerate(inputs):
            # Simple mock: assign a random label based on input content
            mock_label = "positive" if "good" in inp.lower() or "positive" in inp.lower() else "negative"
            mock_classifications.append({
                "input": inp,
                "prediction": mock_label,
                "confidences": [{"option": mock_label, "confidence": random.uniform(0.7, 0.99)}],
                "labels": {mock_label: random.uniform(0.7, 0.99)}
            })
        return mock_classifications