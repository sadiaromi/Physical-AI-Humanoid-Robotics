from src.config.database import engine, Base
from src.models.book import Book
from src.models.book_content_chunk import BookContentChunk
from src.models.query_session import QuerySession
from src.models.query_history import QueryHistory


def init_db():
    """
    Initialize the database by creating all tables
    """
    print("Creating database tables...")
    Base.metadata.create_all(bind=engine)
    print("Database tables created successfully!")


if __name__ == "__main__":
    init_db()