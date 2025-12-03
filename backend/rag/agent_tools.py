"""
Agent tools for the Physical AI RAG system
"""

from agents import AgentFunction
from .rag_system import RAGSystem
from typing import Dict, Any
import json

# Initialize the RAG system
rag_system = RAGSystem()

def initialize_rag_system():
    """Initialize the RAG system with documents"""
    try:
        rag_system.load_documents_to_vector_store()
    except Exception as e:
        print(f"Error loading documents to RAG system: {e}")


@AgentFunction(
    name="retrieve_physical_ai_context",
    description="Retrieve relevant information from the Physical AI & Humanoid Robotics textbook based on user query. Use this when user asks about concepts from the Physical AI course.",
    arguments={"query": {"type": "string", "description": "The query to search for in the Physical AI textbook"}}
)
def retrieve_physical_ai_context(query: str) -> Dict[str, Any]:
    """
    Retrieve relevant context from the Physical AI textbook based on the query.
    """
    try:
        # Load documents if not already loaded
        if not rag_system._loaded:
            initialize_rag_system()
        
        # Retrieve relevant context
        results = rag_system.retrieve_relevant_context(query, top_k=3)
        
        # Format results for the agent
        formatted_results = []
        for result in results:
            formatted_results.append({
                "title": result.get("title", ""),
                "content": result.get("content", "")[:500],  # Limit content length
                "module": result.get("module", ""),
                "path": result.get("path", "")
            })
        
        return {
            "query": query,
            "results": formatted_results,
            "count": len(formatted_results)
        }
    except Exception as e:
        return {
            "error": f"Error retrieving context: {str(e)}",
            "results": [],
            "count": 0
        }


@AgentFunction(
    name="answer_physical_ai_question",
    description="Answer questions about Physical AI & Humanoid Robotics using textbook knowledge. Use this when you need to answer specific questions about the course content.",
    arguments={
        "query": {"type": "string", "description": "The question to answer using Physical AI textbook knowledge"},
        "module": {"type": "string", "description": "Optional module name to limit the search"}
    }
)
def answer_physical_ai_question(query: str, module: str = None) -> str:
    """
    Answer a specific question using the Physical AI textbook knowledge.
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


@AgentFunction(
    name="get_course_modules",
    description="Get a list of available modules in the Physical AI & Humanoid Robotics course. Use this to understand what topics are covered.",
    arguments={}
)
def get_course_modules() -> Dict[str, list]:
    """
    Get the list of available modules in the course.
    """
    modules = [
        "Introduction-to-Physical-AI",
        "The-Robotic-Nervous-System", 
        "The-Digital-Twin",
        "The-AI-Robot-Brain",
        "Vision-Language-Action"
    ]
    return {
        "modules": modules,
        "count": len(modules)
    }