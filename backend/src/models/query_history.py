from sqlalchemy import Column, Integer, String, DateTime, Text, UUID, ForeignKey, Boolean
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from uuid import uuid4
from src.config.database import Base


class QueryHistory(Base):
    __tablename__ = "query_history"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid4)
    session_id = Column(PG_UUID(as_uuid=True), ForeignKey("query_sessions.id"), nullable=False)
    question = Column(Text, nullable=False)  # User's question (will be handled carefully per privacy requirement)
    response = Column(Text, nullable=False)  # AI-generated response
    context_chunks = Column(String, nullable=True)  # Array of UUIDs as JSON string (vector IDs used to generate response)
    timestamp = Column(DateTime, server_default=func.now())
    was_helpful = Column(Boolean, nullable=True)  # User feedback, optional

    def __repr__(self):
        return f"<QueryHistory(id={self.id}, session_id={self.session_id}, timestamp={self.timestamp})>"