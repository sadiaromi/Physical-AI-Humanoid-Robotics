from pydantic import BaseModel, Field
from typing import List, Optional
from uuid import UUID


class QueryRequest(BaseModel):
    question: str = Field(..., description="The question to ask about the book content", max_length=1000)
    book_id: str = Field(..., description="The ID of the book to query")
    selected_text: Optional[str] = Field(None, description="Optional text that the user has selected; if provided, the query will focus only on this text")


class SourceInfo(BaseModel):
    chunk_id: Optional[str] = None
    section_title: Optional[str] = ""
    page_number: Optional[int] = None


class QueryResponse(BaseModel):
    response: str = Field(..., description="The AI-generated response to the query")
    sources: List[SourceInfo] = Field(..., description="List of content chunks used to generate the response")
    query_id: Optional[str] = Field(None, description="The ID of the query for tracking purposes")