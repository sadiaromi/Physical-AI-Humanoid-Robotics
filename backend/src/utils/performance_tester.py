import asyncio
import time
import requests
import uuid
from concurrent.futures import ThreadPoolExecutor
from typing import List, Dict, Any


class PerformanceTester:
    """
    Performance testing for RAG Chatbot API
    """
    
    def __init__(self, base_url: str, api_key: str):
        self.base_url = base_url
        self.api_key = api_key
        self.headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}"
        }
    
    def _make_query_request(self, question: str, book_id: str) -> Dict[str, Any]:
        """
        Make a single query request to the API
        """
        start_time = time.time()
        
        payload = {
            "question": question,
            "book_id": book_id
        }
        
        response = requests.post(
            f"{self.base_url}/query",
            headers=self.headers,
            json=payload
        )
        
        end_time = time.time()
        response_time = end_time - start_time
        
        return {
            "response_time": response_time,
            "status_code": response.status_code,
            "response": response.json() if response.status_code == 200 else None
        }
    
    def test_response_times(self, questions: List[str], book_id: str, iterations: int = 10) -> List[Dict[str, Any]]:
        """
        Test response times for a series of questions
        """
        results = []
        
        for i in range(iterations):
            question = questions[i % len(questions)]  # Cycle through questions
            result = self._make_query_request(question, book_id)
            results.append(result)
            
            print(f"Query {i+1}/{iterations}: {result['response_time']:.2f}s")
        
        return results
    
    def test_concurrent_requests(self, questions: List[str], book_id: str, num_concurrent: int = 5) -> List[Dict[str, Any]]:
        """
        Test concurrent request handling
        """
        with ThreadPoolExecutor(max_workers=num_concurrent) as executor:
            futures = [
                executor.submit(self._make_query_request, questions[i % len(questions)], book_id)
                for i in range(num_concurrent)
            ]
            
            results = [future.result() for future in futures]
        
        return results
    
    def analyze_results(self, results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Analyze performance test results
        """
        response_times = [r["response_time"] for r in results]
        status_codes = [r["status_code"] for r in results]
        
        # Calculate metrics
        avg_response_time = sum(response_times) / len(response_times)
        max_response_time = max(response_times)
        min_response_time = min(response_times)
        percentile_95 = sorted(response_times)[int(0.95 * len(response_times))]
        
        success_count = sum(1 for code in status_codes if 200 <= code < 300)
        success_rate = success_count / len(status_codes) * 100
        
        # Check requirements
        meets_response_time_requirement = all(rt < 5.0 for rt in response_times)
        meets_95th_percentile_requirement = percentile_95 < 5.0
        
        return {
            "avg_response_time": avg_response_time,
            "max_response_time": max_response_time,
            "min_response_time": min_response_time,
            "percentile_95": percentile_95,
            "success_rate": success_rate,
            "meets_response_time_requirement": meets_response_time_requirement,
            "meets_95th_percentile_requirement": meets_95th_percentile_requirement,
            "total_requests": len(results),
            "successful_requests": success_count
        }


def main():
    # Example usage of the performance tester
    BASE_URL = "http://localhost:8000"  # Replace with your API URL
    API_KEY = "your-api-key"  # Replace with your API key
    BOOK_ID = "test-book-id"  # Replace with a valid book ID
    
    tester = PerformanceTester(BASE_URL, API_KEY)
    
    # Sample questions for testing
    sample_questions = [
        "What is the main concept of this book?",
        "Can you summarize the key points in chapter 1?",
        "Explain the machine learning algorithm described in this text.",
        "What are the implications of the research findings?",
        "How does this concept apply in practice?"
    ]
    
    print("Testing individual response times...")
    single_results = tester.test_response_times(sample_questions, BOOK_ID, iterations=10)
    single_analysis = tester.analyze_results(single_results)
    print(f"Single request analysis: {single_analysis}")
    
    print("\nTesting concurrent requests...")
    concurrent_results = tester.test_concurrent_requests(sample_questions, BOOK_ID, num_concurrent=5)
    concurrent_analysis = tester.analyze_results(concurrent_results)
    print(f"Concurrent request analysis: {concurrent_analysis}")
    
    # Overall assessment
    overall_pass = (
        single_analysis["meets_95th_percentile_requirement"] and
        concurrent_analysis["meets_95th_percentile_requirement"]
    )
    
    print(f"\nPerformance test result: {'PASS' if overall_pass else 'FAIL'}")
    print(f"Single request 95th percentile: {single_analysis['percentile_95']:.2f}s")
    print(f"Concurrent request 95th percentile: {concurrent_analysis['percentile_95']:.2f}s")
    print(f"Single request success rate: {single_analysis['success_rate']:.2f}%")
    print(f"Concurrent request success rate: {concurrent_analysis['success_rate']:.2f}%")


if __name__ == "__main__":
    main()