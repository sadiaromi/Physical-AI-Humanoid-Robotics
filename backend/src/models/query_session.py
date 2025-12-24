from sqlalchemy import Column, Integer, String, DateTime, Text, UUID, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from uuid import uuid4
from src.config.database import Base


class QuerySession(Base):
    __tablename__ = "query_sessions"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid4)
    user_id = Column(String(100), nullable=True)  # For tracking, optional if anonymous
    book_id = Column(PG_UUID(as_uuid=True), ForeignKey("books.id"), nullable=False)
    created_at = Column(DateTime, server_default=func.now())
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now())
    expires_at = Column(DateTime, nullable=False)  # Session expiration

    def __repr__(self):
        return f"<QuerySession(id={self.id}, user_id={self.user_id}, book_id={self.book_id})>"