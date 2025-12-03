"""
Main RAG system that connects Qdrant vector store with Gemini API
"""

import logging
from typing import List, Dict, Any
from .config import COLLECTION_NAME, QDRANT_URL, QDRANT_API_KEY
from .document_loader import DocumentLoader
from .vector_store.qdrant_client import QdrantVectorStore
from .embeddings import get_embedding_for_text

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RAGSystem:
    """
    RAG (Retrieval-Augmented Generation) system for Physical AI textbook
    """

    def __init__(self):
        self.vector_store = QdrantVectorStore(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            collection_name=COLLECTION_NAME
        )
        self.document_loader = DocumentLoader()
        self._loaded = False

    def load_documents_to_vector_store(self):
        """
        Load documents from the textbook and add them to the vector store
        """
        if self._loaded:
            logger.info("Documents already loaded")
            return

        logger.info("Loading documents...")
        documents = self.document_loader.load_documents()

        logger.info("Generating embeddings and adding to vector store...")
        self.vector_store.add_documents(documents, get_embedding_for_text)

        self._loaded = True
        logger.info(f"Successfully loaded {len(documents)} documents to vector store")

    def retrieve_relevant_context(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the vector store based on the query
        """
        query_embedding = get_embedding_for_text(query)
        results = self.vector_store.similarity_search(query_embedding, k=top_k)

        return results

    def answer_question(self, query: str, context_chunks: List[Dict[str, Any]] = None) -> str:
        """
        Return context chunks for the agent to handle the response generation
        The actual response generation will be handled by the agent using Gemini
        """
        if context_chunks is None:
            # Retrieve relevant context if not provided
            context_chunks = self.retrieve_relevant_context(query)

        # Format context for return
        context_text = ""
        for i, chunk in enumerate(context_chunks):
            context_text += f"\n{i+1}. Title: {chunk['title']}\n   Content: {chunk['content'][:500]}...\n   Module: {chunk['module']}\n   Path: {chunk['path']}\n"

        if context_text:
            return f"Here is the relevant information from the Physical AI textbook:\n{context_text}"
        else:
            return "I couldn't find relevant information in the textbook for your query. Please try rephrasing or ask about a different topic from the course."

    def chat_with_gemini(self, query: str) -> str:
        """
        Main method to handle chat queries - retrieves context and returns for agent to handle
        """
        if not self._loaded:
            self.load_documents_to_vector_store()

        # Get relevant context
        context_chunks = self.retrieve_relevant_context(query)

        # Return context to be handled by the agent
        return self.answer_question(query, context_chunks)

    def query_by_module(self, query: str, module_name: str, top_k: int = 3) -> str:
        """
        Query specifically within a module of the textbook
        """
        # Get embedding for the query
        query_embedding = get_embedding_for_text(query)

        # Search through retrieved chunks for the specific module
        all_chunks = self.vector_store.similarity_search(query_embedding, k=top_k*2)

        # Filter for the specific module
        filtered_chunks = [chunk for chunk in all_chunks if module_name.lower() in chunk['module'].lower()]

        # If we don't have enough results from the specific module, add more general results
        while len(filtered_chunks) < top_k and len(all_chunks) > len(filtered_chunks):
            # Add next best results if needed
            next_best = [chunk for chunk in all_chunks if chunk not in filtered_chunks]
            filtered_chunks.extend(next_best[:top_k-len(filtered_chunks)])

        # Format response
        context_text = ""
        for i, chunk in enumerate(filtered_chunks[:top_k]):
            context_text += f"\n{i+1}. Title: {chunk['title']}\n   Content: {chunk['content'][:500]}...\n   Module: {chunk['module']}\n   Path: {chunk['path']}\n"

        if context_text:
            return f"Here is information about '{query}' from the {module_name} module:\n{context_text}"
        else:
            return f"I couldn't find relevant information in the {module_name} module. Please try a different module or topic."


if __name__ == "__main__":
    # Test the RAG system
    rag = RAGSystem()

    # Example query
    query = "What is ROS 2 and how does it differ from ROS 1?"
    response = rag.chat_with_gemini(query)
    print(f"Query: {query}")
    print(f"Response: {response}")