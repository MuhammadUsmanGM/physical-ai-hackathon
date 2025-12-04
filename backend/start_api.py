"""
Start the FastAPI server for the chat API
"""

import uvicorn
from rag.api import app

if __name__ == "__main__":
    print("Starting Physical AI Chat API...")
    print("API will be available at: http://localhost:8000")
    print("API docs at: http://localhost:8000/docs")
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)
