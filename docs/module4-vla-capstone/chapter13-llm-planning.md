---
id: chapter13-llm-planning
title: "Chapter 13: LLM-based Planning & Multimodal Reasoning"
sidebar_label: "Chapter 13: LLM-based Planning"
---

## Introduction

In the previous chapter, we gave our robot the ability to hear and parse simple commands. Now, we will give it a brain. This chapter delves into the exciting world of Large Language Model (LLM) based planning, where we transition from simple keyword matching to sophisticated, context-aware reasoning. We will explore how to use LLMs to generate complex, multi-step task plans from natural language and how to make those plans robust by integrating real-time visual feedback.

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Explain how Large Language Models (LLMs) can function as reasoning engines for high-level robotic task planning.
*   Construct a detailed prompt for an LLM that effectively combines a user's command, the robot's current state, and its available actions.
*   Develop a Python script to parse a structured plan (e.g., in JSON format) from an LLM's text response.
*   Understand the concept of an "Action Graph" and how it can be used to map LLM-generated steps to robot-executable commands like ROS 2 Actions.
*   Explain the importance of "closing the loop" by integrating real-time sensor feedback into the planning process to create more robust and adaptive robots.
*   Simulate the execution of an LLM-generated plan by calling a sequence of mock robot functions.

## Required Skills and Tools

### Prerequisite Skills

*   **Strong Python Programming**: You should be confident with object-oriented concepts, including creating and using classes and methods.
*   **API Concepts**: A basic understanding of what an API (Application Programming Interface) is and how web requests work.
*   **JSON Familiarity**: You should be comfortable reading and manipulating data in JSON format.

### Tools & Software

*   **Python**: Version 3.8 or higher.
*   **Code Editor**: A code editor like VS Code, PyCharm, or Sublime Text.
*   **LLM API Access**: To run the code with a live LLM, you will need an API key from a provider like OpenAI.
*   **Python Libraries**:
    *   `openai`: The library for interacting with the OpenAI API. (Note: other LLM provider libraries can also be used with minor code adjustments).

## Weekly Breakdown

This chapter is designed to be completed within one week.

*   **Day 1-2: Theory and Prompt Engineering**
    *   **Activity**: Read the chapter sections on LLM-based planning and converting LLM output. Set up your environment by installing the `openai` library.
    *   **Assessment**: In a separate text file, write your own multi-part prompt that includes a new user command (e.g., "Find my keys"), a mock robot state, and a list of available actions.

*   **Day 3-4: Implementation**
    *   **Activity**: Implement the `Robot` and `LLMPlanner` classes from the `simple_llm_planner.py` example. Write the code for constructing the prompt from your text file and for parsing the mock LLM response.
    *   **Assessment**: Your script should be able to successfully generate a prompt and parse the hardcoded JSON response into a list of action dictionaries.

*   **Day 5-6: Plan Execution and (Optional) Live API Integration**
    *   **Activity**: Implement the `execute_plan` method. Run the full simulation and observe how the mock robot's state changes as it executes the plan step-by-step.
    *   **Assessment**: The script should run end-to-end, printing each step of the plan and the corresponding changes in the robot's state.
    *   **Optional Challenge**: If you have an LLM API key, replace the `mock_llm_call` function with a real API call and see if it can generate a valid plan for a new, creative command.

*   **Day 7: Review and Reflection**
    *   **Activity**: Review your completed code, especially the `execute_plan` loop. Think about the concept of "closing the loop" with vision feedback as described in the chapter.
    *   **Assessment**: Write a short paragraph explaining why a pre-planned, static sequence of actions is brittle and how the "Observe, Orient, Decide, Act" (OODA) loop would make a robot more robust in a real-world scenario.

## Lab Setup Requirements

This lab builds on the standard Python environment from the previous chapter.

1.  **Project Folder and Virtual Environment**:
    *   Create a new project folder (e.g., `chapter13-lab`) and set up a Python virtual environment inside it.
        ```bash
        python -m venv venv
        source venv/bin/activate  # On Windows use `venv\Scripts\activate`
        ```

2.  **Install Required Packages**:
    *   With your virtual environment activated, install the `openai` library:
        ```bash
        pip install openai
        ```

3.  **Set Up API Key (Optional but Recommended)**:
    *   To complete the optional challenge of using a live LLM, you need an API key from a provider like OpenAI.
    *   It is crucial **not to hardcode your API key** in your script. The best practice is to set it as an environment variable.
    *   **On macOS/Linux**:
        ```bash
        export OPENAI_API_KEY='your-api-key-here'
        ```
        (Add this line to your `~/.bash_profile` or `~/.zshrc` file to make it permanent).
    *   **On Windows**:
        ```powershell
        $env:OPENAI_API_KEY='your-api-key-here'
        ```
        (To set it permanently, search for "Edit the system environment variables" in the Start Menu).

4.  **Verify Setup**:
    *   Create a new file named `verify_setup.py`.
    *   Add the following code, which checks for the library and optionally for the API key:
        ```python
        import os
        try:
            import openai
            print("OpenAI library is installed correctly.")
        except ImportError:
            print("Error: The 'openai' library is not installed.")
            
        api_key = os.getenv("OPENAI_API_KEY")
        if api_key and len(api_key) > 10: # Simple check if key seems plausible
            print("OpenAI API key is configured in environment variables.")
        else:
            print("Note: OpenAI API key is not configured. You will only be able to run the mock example.")
        ```
    *   Run `python verify_setup.py`. If it confirms the library is installed, your environment is ready for the main exercise.

## From Intent to Action: LLM-based Task Planning

The keyword-based parser from Chapter 12 was effective but brittle. It struggles with complex sentences, synonyms, or any phrasing that deviates from its predefined triggers. LLMs offer a powerful solution. By leveraging their vast world knowledge and reasoning capabilities, we can build a planner that is far more flexible and intelligent.

The core idea is to use an LLM as a "reasoning engine." We provide the LLM with:
1.  **A high-level command** from the user (e.g., "get me a drink from the fridge").
2.  **Context about the environment**, such as the robot's current location and a list of known objects.
3.  **A list of the robot's capabilities**, described as simple functions (e.g., `navigate_to(location)`, `pick_up(object)`, `open(object)`).

Given this prompt, we ask the LLM to generate a step-by-step plan to accomplish the command, using only the functions we provided.

For the command "get me a drink from the fridge," the LLM might generate the following plan:
1.  `navigate_to('fridge')`
2.  `open('fridge')`
3.  `find_object('drink')`
4.  `pick_up('drink')`
5.  `close('fridge')`
6.  `navigate_to('user')`
7.  `place_object('drink')`

This approach is powerful because it allows the robot to break down complex goals into a sequence of simple, executable actions.

## Converting LLM Output into Robot-Executable Plans

The raw text output from an LLM is not yet a robot-executable plan. It needs to be converted into a structured format that our robotics framework, ROS 2, can understand. One common approach is to represent the plan as an **Action Graph**, where nodes are actions and edges represent the sequence.

### The Action Graph

An Action Graph is a directed graph where each node represents a specific ROS 2 Action that the robot should execute. For example, the `navigate_to('fridge')` step from the LLM could be mapped to a `NavigateTo` ROS 2 Action Goal, with `'fridge'` as the target.

Our Python application will be responsible for:
1.  **Parsing the LLM's text-based plan**: This can be done with regular expressions or by instructing the LLM to output the plan in a structured format like JSON.
2.  **Mapping each step to a ROS 2 Action Client**: The application will have a dictionary of available action clients (e.g., one for navigation, one for manipulation).
3.  **Executing the graph**: The application will call the actions in the correct sequence, waiting for each one to complete successfully before starting the next. This ensures that the robot doesn't try to open the fridge before it has navigated to it.

## "Closing the Loop": Integrating Vision Feedback with LLM Reasoning

A plan generated by an LLM is based on its knowledge of the world up to its training cut-off. It has no awareness of the robot's *current* environment. What happens if the fridge door is already open? Or if the drink is not on the shelf the robot expected?

This is where **multimodal reasoning** and "closing the loop" become critical. We need to feed real-time information from the robot's sensors, especially its cameras, back into the LLM's reasoning process.

### A Dynamic Planning Loop

Instead of generating a static plan upfront, we can create a dynamic planning loop:

1.  **Observe**: The robot uses its camera to perceive the current state of the world. A Vision-Language Model (VLM) can be used to convert this visual input into a text description (e.g., "The fridge door is closed. A red can is on the top shelf.").
2.  **Orient**: The LLM is prompted with the user's command, the robot's available actions, and the *current visual description* of the scene.
3.  **Decide**: The LLM chooses the *single best next action* to take based on the current situation. For example, if the fridge is closed, the next action is `open('fridge')`. If it's already open, the LLM might decide the next action is `find_object('drink')`.
4.  **Act**: The robot executes that single action.
5.  **Repeat**: The loop starts over, with the robot observing the new state of the world.

This iterative process makes the robot far more robust. It can react to unexpected events, recover from failures, and adapt its plan on the fly, moving it one step closer to true autonomous behavior. In our code example, we will simulate this loop by providing text-based feedback to the LLM planner.
