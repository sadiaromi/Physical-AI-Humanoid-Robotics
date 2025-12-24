---
id: 1
title: Create-project-constitution-RAG-chatbot
stage: constitution
date: 2025-12-16
surface: agent
model: gpt-4o
feature: rag-chatbot
branch: main
user: Sadia
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "ai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution Project: Integrated RAG Chatbot Development for Book Content Querying

Core principles:
- Accuracy in responses through robust retrieval and generation, ensuring answers are faithful to the book's content or user-selected text

- Seamless integration of the chatbot within the published book (e.g:as an embedded interactive feature in a digital format)

-Modularity and maintainability of code, prioritining clean archietecture for future updates

-User-centric design, focusing on intutitive interaction for querying book content or selected text snippets

-Ethical AI usage,avoiding biases in generation and ensuring privacy in handling user inputs

Key Standards:

 -All responses must be generated based society on retrieved book content or user-provided selected text, with no hallucination

-Codebase structure: Use FastAPI for backend API, Neon Serverless Postgress for metadata storage, Qdrant Cloud free Tier for vector embeddings, Cohere API for embeddings and test generation (no OpenAI usage)

-Development tools: Implement using SpecifyKit Plus for project specification and scaffolding, and Qwen CLI for local model testing and integration

-Security: Implement authentication for API endpoints, data encryption in transit, and compliance with data privacy standards(e.g:no logging for user queries)

-Performance: Response time under 5 seconds for queries, scalable to handle multiple users

-Testing: 100% unit test coverage for core components, integration tests for RAG pipeline, and user acceptance testing for chatbot functionality

-Documentation: Comprehensive README,API docs via Swagger, and inline code comments

Constraints:
-API usage: Exclusively Cohere API keys for all LLM- related tasks (embeddings, generation, agents if applicable); no OpenAI or other providers

-Database and Storage: Limited to Neon Serveless Postgress free tier capacity and Qdrant Cloud Free Tier for vectors(monitor usage to avoid exceeding limits)

-Development environment: Build with SpecifyKit Plus for structured project setup and Qwen CLI for CLI-based workflows and model interactions

-Budget: Zero-cost for core infrastructure (free tiers only; any additional tools must be open-source or free

-Timeline: Complete development and embedding within 4-6 weeks, assuming part-time effort

-Compatbility: Chatbot must work in web-based book formats (e.g.HTMLS embed) and support mobile responsiveness

Sucess Criteria:
- Chatbot accurately answers 95%+ of test questions based on book content, verified through a dataset of 50+ queries

- Hnadles user-selected text queries independently, ignoring broder context when specified 

-Successful enbedding in the published book  without breaking layout or performance

-Passes code review for quality, security, and adherence to tools (Cohere, SpecifyKit Plus, Qwen CLI)

-Zero critical bugs in production simulation, with positive feedback from 5+ beta testers

-Full documentation and reproducible setup, allowing redeployment in under 1 hour
```

## Response snapshot

Updated the project constitution with principles for the Integrated RAG Chatbot for Book Content Querying, including accuracy in responses, seamless integration, modularity, user-centric design, and ethical AI usage. Also detailed key standards for technology stack, security, performance, testing, and documentation, along with development constraints and success criteria.

## Outcome

- ✅ Impact: Created project constitution that establishes clear principles and guidelines for RAG chatbot development
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: Updated .specify/memory/constitution.md with new project principles
- 🔁 Next prompts: Ready to proceed with project implementation following the constitution
- 🧠 Reflection: The new constitution will guide all future development decisions for the RAG chatbot project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin project planning based on the established constitution