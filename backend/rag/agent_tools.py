"""
Agent tools for the Physical AI RAG system
"""

from agents import function_tool
from .rag_system import RAGSystem
from typing import Dict, Any, List
import json

# Initialize the RAG system
rag_system = RAGSystem()

def initialize_rag_system():
    """Initialize the RAG system with documents"""
    try:
        rag_system.load_documents_to_vector_store()
    except Exception as e:
        print(f"Error loading documents to RAG system: {e}")


@function_tool
def retrieve_physical_ai_context(query: str) -> str:
    """
    Retrieve relevant information from the Physical AI & Humanoid Robotics textbook based on user query.
    Use this when user asks about concepts from the Physical AI course.
    """
    try:
        # Load documents if not already loaded
        if not rag_system._loaded:
            initialize_rag_system()
        
        # Retrieve relevant context
        results = rag_system.retrieve_relevant_context(query, top_k=3)
        
        if not results:
            return "No relevant information found in the textbook."
            
        # Format results for the agent
        formatted_output = []
        for i, result in enumerate(results, 1):
            title = result.get("title", "Untitled")
            content = result.get("content", "")[:500]
            module = result.get("module", "Unknown")
            formatted_output.append(f"Result {i}:\nTitle: {title}\nModule: {module}\nContent: {content}\n")
        
        return "\n".join(formatted_output)
    except Exception as e:
        return f"Error retrieving context: {str(e)}"


@function_tool
def answer_physical_ai_question(query: str, module: str = None) -> str:
    """
    Answer questions about Physical AI & Humanoid Robotics using textbook knowledge.
    Use this when you need to answer specific questions about the course content.
    """
    try:
        # Load documents if not already loaded
        if not rag_system._loaded:
            initialize_rag_system()
        
        if module:
            # Query within specific module
            response = rag_system.query_by_module(query, module, top_k=3)
        else:
            # General query
            response = rag_system.chat_with_gemini(query)
        
        return response
    except Exception as e:
        return f"Error answering question: {str(e)}"


@function_tool
def get_course_modules() -> str:
    """
    Get a list of available modules in the Physical AI & Humanoid Robotics course.
    Use this to understand what topics are covered.
    """
    modules = [
        "Introduction-to-Physical-AI",
        "The-Robotic-Nervous-System", 
        "The-Digital-Twin",
        "The-AI-Robot-Brain",
        "Vision-Language-Action"
    ]
    return f"Available Modules: {', '.join(modules)}"