"""
Configuration for the RAG system
"""

import os
from dotenv import load_dotenv

load_dotenv()

# Qdrant configuration
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-cluster-url.qdrant.tech:6333")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_docs")

# Gemini configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
GEMINI_MODEL = os.getenv("GEMINI_MODEL", "gemini-1.5-pro-latest")  # Updated to gemini-1.5-pro

# Documents directory
DOCUMENTS_DIR = os.getenv("DOCUMENTS_DIR", "../../book/docs/")