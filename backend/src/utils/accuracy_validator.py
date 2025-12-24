import asyncio
import json
from typing import List, Dict, Any, Tuple
from src.services.rag_engine import RAGEngine
from src.services.cohere_client import CohereClient


class AccuracyValidator:
    """
    Class to validate the accuracy of RAG responses against expected answers
    """
    
    def __init__(self, rag_engine: RAGEngine, cohere_client: CohereClient):
        self.rag_engine = rag_engine
        self.cohere_client = cohere_client
    
    async def load_test_dataset(self, dataset_path: str) -> List[Dict[str, Any]]:
        """
        Load the test dataset from a JSON file
        Expected format: [{"question": "...", "book_id": "...", "expected_answer": "...", "category": "..."}, ...]
        """
        with open(dataset_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    
    async def evaluate_response_accuracy(self, question: str, expected_answer: str, generated_answer: str) -> float:
        """
        Evaluate the accuracy of a generated response against the expected answer
        Returns a similarity score between 0 and 1
        """
        # Use Cohere's embed model to get embeddings for both answers
        embeddings = await self.cohere_client.get_embeddings([expected_answer, generated_answer])
        expected_embedding, generated_embedding = embeddings[0], embeddings[1]
        
        # Calculate cosine similarity
        similarity = self.cosine_similarity(expected_embedding, generated_embedding)
        return similarity
    
    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors
        """
        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(b * b for b in vec2) ** 0.5
        
        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0
        
        return dot_product / (magnitude1 * magnitude2)
    
    async def run_accuracy_validation(self, test_dataset: List[Dict[str, Any]], threshold: float = 0.8) -> Dict[str, Any]:
        """
        Run the accuracy validation on the test dataset
        """
        results = []
        total_tests = len(test_dataset)
        passed_tests = 0
        
        for i, test_case in enumerate(test_dataset):
            question = test_case["question"]
            book_id = test_case["book_id"]
            expected_answer = test_case["expected_answer"]
            
            # Get response from RAG engine
            rag_response = await self.rag_engine.query(question, book_id)
            generated_answer = rag_response["response"]
            
            # Evaluate accuracy
            similarity_score = await self.evaluate_response_accuracy(question, expected_answer, generated_answer)
            
            is_accurate = similarity_score >= threshold
            if is_accurate:
                passed_tests += 1
            
            results.append({
                "question": question,
                "expected_answer": expected_answer,
                "generated_answer": generated_answer,
                "similarity_score": similarity_score,
                "is_accurate": is_accurate,
                "category": test_case.get("category", "general")
            })
            
            print(f"Test {i+1}/{total_tests}: {'PASS' if is_accurate else 'FAIL'} (Score: {similarity_score:.2f})")
        
        accuracy_percentage = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
        
        return {
            "total_tests": total_tests,
            "passed_tests": passed_tests,
            "accuracy_percentage": accuracy_percentage,
            "threshold": threshold,
            "results": results,
            "is_target_met": accuracy_percentage >= 95.0
        }


async def main():
    """
    Example usage of the accuracy validator
    """
    # This would need actual initialized services in a real implementation
    # For now, we'll show the structure
    print("Accuracy validation system ready.")
    print("To run validation, you would:")
    print("1. Initialize the RAG engine with real services")
    print("2. Load a test dataset from a JSON file")
    print("3. Run the validation using the AccuracyValidator class")
    print("4. Review the results to ensure 95%+ accuracy target is met")


if __name__ == "__main__":
    asyncio.run(main())