---
id: 0001
title: Initialize Constitution
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: master
user: User
command: /sp.constitution
labels: ["governance", "initialization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

```text
Project: AI-Native Textbook on Physical AI & Humanoid Robotics

Core Principles:
- Accuracy: All technical content must reference official documentation (ROS 2, NVIDIA Isaac, Gazebo, Unity) or reliable robotics/AI sources.
- Clarity: Text must be understandable for technical learners (undergraduate/graduate level) while remaining precise.
- Reproducibility: All code examples, simulations, and hardware instructions must be runnable and verifiable.
- Pedagogical Rigor: Concepts explained step-by-step with examples, diagrams, and exercises.
- Safety: Hardware and lab setup guidance must prioritize safe practices.

Key Standards:
- Chapters must follow the course modules:
  1. Robotic Nervous System (ROS 2)
  2. Digital Twin (Gazebo & Unity)
  3. AI-Robot Brain (NVIDIA Isaac)
  4. Vision-Language-Action (VLA) and Capstone Project
- Code examples: Python + ROS 2, compatible with Ubuntu 22.04 and NVIDIA Isaac Sim
- Hardware guidance: Include specifications, alternatives, and cost breakdowns
- Simulations: Include Gazebo, Unity, Isaac Sim examples with step-by-step setup
- Citation format: IEEE or consistent academic referencing
- Visuals: Include diagrams of robot architecture, sensor layouts, perception pipelines
- Writing clarity: Flesch-Kincaid grade 10-12

Constraints:
- Word count: 50,000–70,000 words (full textbook)
- Chapters: Minimum 13 chapters corresponding to weekly breakdown
- Code snippets must run without errors in simulation or on edge kits
- Book format: Markdown compatible with Docusaurus and Spec-Kit Plus

Success Criteria:
- All chapters written with accurate, traceable content
- Code examples and simulations are runnable
- Diagrams and visuals support conceptual understanding
- Book deployable to GitHub Pages or Vercel
- Ready for hackathon submission including demo video and repository
```

## Response snapshot

The project constitution has been updated to version 1.0.0. Suggested commit message: `docs: initialize constitution to v1.0.0 (initial principles, standards, constraints, and governance)`

## Outcome

- ✅ Impact: Constitution file created and updated.
- 🧪 Tests: N/A
- 📁 Files: 1 file modified: .specify/memory/constitution.md
- 🔁 Next prompts: User to review and potentially commit the changes.
- 🧠 Reflection: Successfully initialized the project constitution based on user input. Handled placeholder replacement and versioning.

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: N/A
