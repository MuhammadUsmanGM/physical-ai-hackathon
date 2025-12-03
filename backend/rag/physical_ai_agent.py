"""
Physical AI & Humanoid Robotics Assistant using OpenAI Agent SDK with Gemini API
"""

from agents import Agent, Runner, AsyncOpenAI, set_default_openai_client, set_tracing_disabled, OpenAIChatCompletionsModel
from dotenv import load_dotenv
from .agent_tools import retrieve_physical_ai_context, answer_physical_ai_question, get_course_modules, initialize_rag_system
import os

# Load env variable from .env file
load_dotenv()

# Get API key and base URL from .env file
gemini_api_key = os.getenv("GEMINI_API_KEY")
gemini_base_url = os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")
gemini_model = os.getenv("GEMINI_MODEL", "gemini-1.5-pro-latest")

# Configure external AsyncOpenAI client using Gemini endpoint
external_client = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url=gemini_base_url
)

# Set client as default for framework
set_default_openai_client(external_client)

# Disable tracing for clear output
set_tracing_disabled(True)

# Set model to use Gemini via OpenAI Agents
model = OpenAIChatCompletionsModel(
    model=gemini_model,  # Using configurable Gemini model
    openai_client=external_client
)

# Initialize RAG system
initialize_rag_system()

def create_physical_ai_assistant():
    """
    Create a Physical AI & Humanoid Robotics assistant agent
    """
    agent = Agent(
        name="Physical_AI_Assistant",
        instructions="""You are an expert assistant for the Physical AI & Humanoid Robotics course.
        You have access to the textbook content and can answer questions about:
        - Introduction to Physical AI and Embodied Intelligence
        - The Robotic Nervous System (ROS 2)
        - The Digital Twin (Gazebo & Unity)
        - The AI-Robot Brain (NVIDIA Isaac™)
        - Vision-Language-Action (VLA)

        Use the available tools to retrieve relevant information from the textbook before answering questions.
        Be helpful, accurate, and reference specific modules when relevant.
        If you cannot find the information in the textbook, acknowledge this limitation.""",
        model=model,
        functions=[
            retrieve_physical_ai_context,
            answer_physical_ai_question,
            get_course_modules
        ]
    )
    return agent

def chat_with_physical_ai_assistant(user_query: str):
    """
    Chat with the Physical AI assistant
    """
    agent = create_physical_ai_assistant()
    result = Runner.run_sync(
        agent,
        user_query
    )
    return result.final_output

# Example usage
def main():
    print("Physical AI & Humanoid Robotics Assistant")
    print("Ask me anything about the course!")
    print("-" * 50)

    # Example queries
    queries = [
        "What is Physical AI?",
        "Explain ROS 2 architecture and how it differs from ROS 1",
        "How does NVIDIA Isaac Sim work?",
        "What are the main modules in this course?"
    ]

    for query in queries:
        print(f"\nQuery: {query}")
        result = chat_with_physical_ai_assistant(query)
        print(f"Response: {result}")
        print("-" * 50)

if __name__ == "__main__":
    main()