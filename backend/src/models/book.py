from sqlalchemy import Column, Integer, String, DateTime, Text, UUID
from sqlalchemy.sql import func
from sqlalchemy.dialects.postgresql import UUID as PG_UUID
from uuid import uuid4
from src.config.database import Base


class Book(Base):
    __tablename__ = "books"

    id = Column(PG_UUID(as_uuid=True), primary_key=True, default=uuid4)
    title = Column(String(500), nullable=False)
    author = Column(String(200), nullable=True)
    isbn = Column(String(17), nullable=True)  # ISBN-13 is 13 digits + 4 hyphens
    publication_date = Column(DateTime, nullable=True)
    language = Column(String(10), default="en")
    created_at = Column(DateTime, server_default=func.now())
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now())
    status = Column(String(20), default="processing")  # processing, ready, archived
    book_metadata = Column(Text)  # JSON stored as text

    def __repr__(self):
        return f"<Book(id={self.id}, title='{self.title}', author='{self.author}')>"