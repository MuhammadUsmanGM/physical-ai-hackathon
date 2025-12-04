"""
Start the FastAPI server for the chat API
"""

import uvicorn
import os

if __name__ == "__main__":
    print("Starting Physical AI Chat API...")
    port = int(os.getenv("PORT", 8000))
    print(f"API will be available at: http://0.0.0.0:{port}")
    print(f"API docs at: http://0.0.0.0:{port}/docs")
    # Use server:app which has the database lifecycle handler
    uvicorn.run("server:app", host="0.0.0.0", port=port, reload=False)
