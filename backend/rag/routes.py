from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
import logging
from .rag_system import RAGSystem

router = APIRouter(tags=["chat"])

# Initialize RAG system
rag_system = RAGSystem()

# Pydantic models
class QueryRequest(BaseModel):
    query: str
    context: Optional[str] = None  # Added for text selection feature
    top_k: Optional[int] = 5
    module: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    relevant_chunks: Optional[List[dict]] = None
    sources: Optional[List[str]] = None

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: QueryRequest):
    """
    Main chat endpoint that processes user queries using the AI agent
    """
    try:
        from .physical_ai_agent import chat_with_physical_ai_assistant_async
        
        # Use the async agent to process the query
        # Pass context if available (agent needs to support it, or we prepend it)
        query_text = request.query
        if request.context:
            query_text = f"Context: {request.context}\n\nQuestion: {request.query}"
            
        response = await chat_with_physical_ai_assistant_async(query_text)
        
        return ChatResponse(response=response)
    
    except Exception as e:
        logging.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/query-by-module/{module_name}", response_model=ChatResponse)
async def query_by_module_endpoint(module_name: str, request: QueryRequest):
    """
    Query endpoint limited to a specific module
    """
    try:
        response = rag_system.query_by_module(
            query=request.query,
            module_name=module_name,
            top_k=request.top_k
        )
        return ChatResponse(response=response)
    
    except Exception as e:
        logging.error(f"Error processing module query: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/modules")
async def get_modules():
    """
    Get list of available modules in the textbook
    """
    modules = [
        "Introduction-to-Physical-AI",
        "The-Robotic-Nervous-System", 
        "The-Digital-Twin",
        "The-AI-Robot-Brain",
        "Vision-Language-Action"
    ]
    return {"modules": modules}

@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "rag_loaded": rag_system._loaded}
