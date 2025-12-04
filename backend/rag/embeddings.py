"""
Embedding generation module for the RAG system using Local Embeddings (Sentence Transformers)
"""

import logging
from typing import List
from sentence_transformers import SentenceTransformer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LocalEmbeddingGenerator:
    """
    Embedding generator using local Sentence Transformers model
    """
    
    def __init__(self, model_name: str = "all-MiniLM-L6-v2"):
        """
        Initialize the embedding generator with a local model
        """
        self.model_name = model_name
        logger.info(f"Loading local embedding model: {model_name}...")
        self.model = SentenceTransformer(model_name)
        logger.info(f"Initialized local embedding generator: {model_name}")
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        """
        try:
            embeddings = self.model.encode(texts)
            return embeddings.tolist()
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            # Return zero vectors in case of error (384 dimensions for MiniLM)
            return [[0.0] * 384 for _ in texts]


# Global instance
embedding_generator = LocalEmbeddingGenerator()


def get_embedding_for_text(text: str) -> List[float]:
    """
    Get embedding for a single text
    """
    return embedding_generator.generate_embeddings([text])[0]


def get_embeddings_for_texts(texts: List[str]) -> List[List[float]]:
    """
    Get embeddings for multiple texts
    """
    return embedding_generator.generate_embeddings(texts)