from sqlalchemy import Column, Integer, String, DateTime, Text, UUID, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from uuid import uuid4
from src.config.database import Base


class BookContentChunk(Base):
    __tablename__ = "book_content_chunks"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid4)
    book_id = Column(PG_UUID(as_uuid=True), ForeignKey("books.id"), nullable=False)
    content = Column(Text, nullable=False)  # The actual text chunk
    chunk_index = Column(Integer, nullable=False)  # Sequential number for ordering
    section_title = Column(String(200), nullable=True)
    page_number = Column(Integer, nullable=True)
    token_count = Column(Integer, nullable=True)  # Number of tokens in chunk
    vector_id = Column(String, nullable=True)  # ID in Qdrant vector database
    created_at = Column(DateTime, server_default=func.now())

    def __repr__(self):
        return f"<BookContentChunk(id={self.id}, book_id={self.book_id}, chunk_index={self.chunk_index})>"