<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections are new
Removed sections: N/A
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Integrated RAG Chatbot for Book Content Querying Constitution

## Core Principles

### Accuracy through Robust Retrieval and Generation
All responses must be generated based on retrieved book content or user-provided selected text, with no hallucination. This principle ensures the chatbot provides reliable, factual answers directly grounded in source material.

### Seamless Integration with Published Book
Ensuring the chatbot serves as an embedded interactive feature in digital book formats without disrupting user experience. The chatbot must enhance the reading experience rather than detract from it.

### Modularity and Maintainability of Code
Prioritizing clean architecture for future updates and component independence. Code components must be modular, well-documented, and maintainable to support long-term evolution.

### User-Centric Design for Book Interaction
Focusing on intuitive interaction for querying book content or selected text snippets. The interface and user journey must be simple and accessible for readers of all technical backgrounds.

### Ethical AI Usage and Privacy Protection
Avoiding biases in generation and ensuring privacy in handling user inputs. The system must implement responsible AI practices and protect user data at all times.

### Performance and Scalability Standards
Response time under 5 seconds for queries, scalable to handle multiple users. The system must maintain fast, reliable responses even under load.

## Key Standards and Technology Stack

Technology stack requirements:
- Backend: FastAPI for API endpoints
- Database: Neon Serverless Postgres for metadata storage
- Vector storage: Qdrant Cloud free tier for vector embeddings
- Embeddings/Text Generation: Cohere API exclusively (no OpenAI)
- Development tools: SpecifyKit Plus for project specification and scaffolding, Qwen CLI for local model testing and integration

Security requirements:
- Authentication for API endpoints
- Data encryption in transit
- Compliance with data privacy standards
- No logging for user queries

Quality requirements:
- 100% unit test coverage for core components
- Integration tests for RAG pipeline
- User acceptance testing for chatbot functionality
- Comprehensive documentation (README, API docs via Swagger, inline comments)

## Development Workflow and Constraints

Technology usage constraints:
- API usage: Exclusively Cohere API keys for all LLM-related tasks; no OpenAI or other providers
- Database and Storage: Limited to Neon Serverless Postgres free tier capacity and Qdrant Cloud Free Tier for vectors
- Development environment: Build with SpecifyKit Plus for structured project setup and Qwen CLI for CLI-based workflows
- Budget: Zero-cost for core infrastructure (free tiers only; additional tools must be open-source or free)

Timeline and compatibility:
- Timeline: Complete development and embedding within 4-6 weeks, assuming part-time effort
- Compatibility: Chatbot must work in web-based book formats (e.g. HTML embed) and support mobile responsiveness

Success criteria:
- Chatbot accurately answers 95%+ of test questions based on book content, verified through a dataset of 50+ queries
- Handles user-selected text queries independently, ignoring broader context when specified
- Successful embedding in the published book without breaking layout or performance
- Passes code review for quality, security, and adherence to tools
- Zero critical bugs in production simulation, with positive feedback from 5+ beta testers
- Full documentation and reproducible setup, allowing redeployment in under 1 hour

## Governance
The constitution governs all development decisions and overrides any conflicting practices. Amendments to the constitution must be documented and approved by the project lead. All pull requests and reviews must verify compliance with the technology stack, accuracy standards, and ethical AI usage guidelines outlined in this document.

**Version**: 1.0.0 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-16