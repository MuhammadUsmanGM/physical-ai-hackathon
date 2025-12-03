"""
Embedding generation module for the RAG system
"""

import logging
from typing import List
from transformers import AutoTokenizer, AutoModel
import torch
import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EmbeddingGenerator:
    """
    Embedding generator using Hugging Face transformer models
    """
    
    def __init__(self, model_name: str = "sentence-transformers/all-MiniLM-L6-v2"):
        """
        Initialize the embedding generator with a transformer model
        """
        self.model_name = model_name
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModel.from_pretrained(model_name)
        
        # Move model to GPU if available
        if torch.cuda.is_available():
            self.model = self.model.cuda()
        
        self.model.eval()
        logger.info(f"Initialized embedding model: {model_name}")
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        """
        embeddings = []
        
        for text in texts:
            try:
                embedding = self._generate_single_embedding(text)
                embeddings.append(embedding)
            except Exception as e:
                logger.error(f"Error generating embedding for text: {e}")
                # Return zero vector in case of error
                embeddings.append([0.0] * 384)  # Default size for MiniLM
        
        return embeddings
    
    def _generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        # Tokenize input text
        inputs = self.tokenizer(
            text,
            return_tensors="pt",
            padding=True,
            truncation=True,
            max_length=512
        )
        
        # Move tensors to GPU if available
        if torch.cuda.is_available():
            inputs = {key: val.cuda() for key, val in inputs.items()}
        
        # Generate embeddings
        with torch.no_grad():
            outputs = self.model(**inputs)
            # Mean pooling to get the sentence embedding
            embedding = outputs.last_hidden_state.mean(dim=1).cpu().numpy()
        
        # Convert to list and return
        return embedding.flatten().tolist()


class MockEmbeddingGenerator:
    """
    Mock embedding generator for testing without heavy model loading
    """
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate mock embeddings for a list of texts
        """
        import hashlib
        
        embeddings = []
        for text in texts:
            # Generate a deterministic hash-based vector
            hash_obj = hashlib.md5(text.encode())
            hex_dig = hash_obj.hexdigest()
            
            # Convert hex to float values
            embedding = []
            for i in range(0, len(hex_dig), 2):
                hex_pair = hex_dig[i:i+2]
                val = int(hex_pair, 16) / 255.0  # Normalize to [0, 1]
                val = val * 2 - 1  # Scale to [-1, 1]
                embedding.append(val)
            
            # Pad to 384 dimensions (common for MiniLM)
            while len(embedding) < 384:
                embedding.append(0.0)
            embedding = embedding[:384]
            
            embeddings.append(embedding)
        
        return embeddings


# Global instance - use mock by default for lightweight operation
embedding_generator = MockEmbeddingGenerator()


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