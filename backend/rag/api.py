"""
FastAPI implementation for the RAG system
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import logging
from .rag_system import RAGSystem

# Initialize app
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="Retrieval-Augmented Generation API for Physical AI textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify your frontend domains
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG system
rag_system = RAGSystem()

# Pydantic models
class QueryRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5
    module: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    relevant_chunks: Optional[List[dict]] = None
    sources: Optional[List[str]] = None

class ToolCall(BaseModel):
    name: str
    arguments: dict

# API Endpoints
@app.on_event("startup")
async def startup_event():
    """
    Startup event - documents are already loaded in Qdrant
    """
    logging.info("API server started - using existing Qdrant collection")
    # Documents are already in Qdrant from the ingestion script
    # No need to reload them on every startup

@app.get("/")
async def root():
    """
    Root endpoint
    """
    return {"message": "Physical AI & Humanoid Robotics RAG API"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: QueryRequest):
    """
    Main chat endpoint that processes user queries using the AI agent
    """
    try:
        from .physical_ai_agent import chat_with_physical_ai_assistant_async
        
        # Use the async agent to process the query
        response = await chat_with_physical_ai_assistant_async(request.query)
        
        return ChatResponse(response=response)
    
    except Exception as e:
        logging.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/query-by-module/{module_name}", response_model=ChatResponse)
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

@app.get("/modules")
async def get_modules():
    """
    Get list of available modules in the textbook
    """
    # In a real implementation, this would come from document metadata
    modules = [
        "Introduction-to-Physical-AI",
        "The-Robotic-Nervous-System", 
        "The-Digital-Twin",
        "The-AI-Robot-Brain",
        "Vision-Language-Action"
    ]
    return {"modules": modules}

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "rag_loaded": rag_system._loaded}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)