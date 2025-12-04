"""
Script to ingest documents into the Qdrant vector store
"""

import logging
from rag.rag_system import RAGSystem

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    print("Initializing RAG System for Ingestion...")
    
    try:
        rag = RAGSystem()
        print("Starting ingestion process...")
        rag.load_documents_to_vector_store()
        print("\nIngestion complete! Documents are now embedded in Qdrant Cloud.")
        
    except Exception as e:
        logger.error(f"Ingestion failed: {e}")
        print(f"\nError: {e}")

if __name__ == "__main__":
    main()
