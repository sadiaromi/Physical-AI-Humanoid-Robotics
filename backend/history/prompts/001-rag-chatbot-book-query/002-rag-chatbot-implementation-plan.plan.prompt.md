---
id: "002"
title: "RAG Chatbot Implementation Plan"
stage: "plan"
date_iso: "2025-02-17"
surface: "agent"
model: "qwen"
feature: "rag-chatbot-book-query"
branch: "001-rag-chatbot-book-query"
user: "user"
command: "/sp.plan"
labels: ["plan", "rag", "chatbot", "architecture"]
links:
  spec: "specs/001-rag-chatbot-book-query/spec.md"
  ticket: null
  adr: null
  pr: null
files: 
  - "specs/001-rag-chatbot-book-query/plan.md"
  - "specs/001-rag-chatbot-book-query/research.md"
  - "specs/001-rag-chatbot-book-query/data-model.md"
  - "specs/001-rag-chatbot-book-query/quickstart.md"
  - "specs/001-rag-chatbot-book-query/contracts/openapi.yaml"
tests: []
outcome: "Successfully created implementation plan for RAG chatbot"
evaluation: "Plan includes all required phases with proper technical context and architecture decisions"
---

# Prompt History Record

## Prompt

```
$ARGUMENTS
```

/sp.plan Integrated RAG Chatbot Development - Final Plan (Backend Only - Cohere + FastAPI + Neon + Frontend will be added later in docusaurus book

1. Project Analysis (Summary)

Objective: Build a high-accuracy RAG Chatbot for a digital book using only Cohere
Scope: Backend API + Ingestion pipeline (no frontend in this phase 1-6)
Integration: Will be done later inside docusaurus-based book
LLM Providers: Cohere only (key: sqB7ygQv30JL2qrrb0OXmdh0vgyVtPQx3nQzPHcM)
Vector 00: Qdrant Cloud Free Tier (cluster ID: e1eacd0e-a2be-45ee-b8fc-32ef44dbf209)
Metadata : Neon Serverless Postgres (provided connection string)
Tools: SpecifyKit Plus (scaffolding) + Qwen CLI (local coding & testing)
Budget: 100% free tiers only
Timeline: 4-6 weeks part-time

2. Development Plan (6 phases)

Phase 1 - Project Setup & Scaffolding (week 1)
-Clone & run SpecifyKit Plus with both specs
-Generate full project structure (backend/, ingestion/, rag/, api/, tests/)
-Create .env with Cohere key, Neon URL, Qdrant key & URL
-Install deps: fastapi, uvicorn, cohere, qdrant-client, asyncpg, sqlalchemy, python-dotenv
-Run empty FastAPI server + 200 OK 

Phase 2 - Book Ingestion Pipeline (week 1-2)
-Write chunking script (500-800 tokens, overlapping)
-Generate Cohere embeddings (embed-english-v3.0)
-Create Qdrant Collection(vector size 2024)
-Upsert vectors + payload (chunk text +metadata)
-Store metadata in Neon Postgres (id, chunk_text, section, page etc.)
-One-click ingestion script ready

Phase 3 - RAG Engine (week 2-3)
- Retriever class > Cohere embed query > Qdrant serch top-k 
-Generater class > Cohere (command-I-plus / command-I with strict context prompt
-Two modes:
   -Normal mode > search whole book (Qdrant)
   -Selected-text mode > use only user-provided text (no retrieval)
-Strong system prompt to prevent hallucination
-Local testing with Qwen CLI for prompt testing

Phase 4 - Secure FatsAPI Backend (week 3-4)
-POST /query { "question"; "...", 'selected_text"; "..." (optional) }
-API key or JWT authentication
-Rate limiting + async endpoints
-No logging of user questions
-Swagger docs auto-generated
-Response time target < 5 seconds

Phase 5 - Frontend Integration
-Deferred - will be done later directly inside docusaurus book
-Only note added: "Frontend integration will be done in Docusarus later"

Phase 6 - Testing, Monitoring & Documentation (week 5-6)
-100% will test coverage (pytest)
-Integration tests for both RAG modes
-50+ golden query dataset + target 95%+ accuracy
-Loud & security testing
-Complete README +deployment guide
-Monitoring script for Qdrant/Neon usage
-Final code review against specs

Week 1 > Finish 1 + Start Phase 2
Week 2 > Finish Phase 2 + Start Phase 3
Week 3 > Finish Phase 3 + Start Phase 4
Week 4 > Finish Phase 4
Week 5-6 > Finish 6 (Testing + Docs)

Total estimated effort: 00-90 hours + easily achievable in 4-6 weeks part-time

SUCCESS CRITERIA (Will be estimated)

-API returns correct answers in 5s
-95% accuracy on 50+ test queries
-Selected-tetx mode works perfectly isolated
-No OpenAI usage anywhere
-All code runs on free tiers without exceeding limits
-Full documentation + 1-hour reproduclible setup
-Ready for Docusaurus embedding later

Project is now perfectly scoped for backend-only development using Cohere + free tiers.
Frontend phase - wil be added directly in Docusaurus book later.

## Response

Successfully created the implementation plan for the RAG Chatbot feature, including:

1. Implementation plan (plan.md) - Technical context, project structure, and architecture
2. Research summary (research.md) - Technical decisions and alternatives considered
3. Data model (data-model.md) - Entity relationships and validation rules
4. Quickstart guide (quickstart.md) - Setup and usage instructions
5. API contracts (contracts/openapi.yaml) - OpenAPI specification for the API endpoints
6. Agent context update - Updated QWEN.md with new technology stack

All Phase 0 and Phase 1 planning artifacts have been generated as required by the workflow.