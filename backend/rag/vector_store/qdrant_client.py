"""
Qdrant client implementation for vector storage of Physical AI textbook content
"""

import logging
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http.models import (
    PointStruct,
    VectorParams,
    Distance,
    SearchRequest,
    Filter,
    FieldCondition,
    MatchValue
)
from uuid import uuid4
import hashlib

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QdrantVectorStore:
    """
    Qdrant vector store for storing and retrieving Physical AI textbook content
    """
    
    def __init__(self, url: str, api_key: str, collection_name: str):
        self.client = QdrantClient(url=url, api_key=api_key, prefer_grpc=True)
        self.collection_name = collection_name
        self.vector_size = 768  # Using embedding size compatible with most models
        
        # Initialize collection if not exists
        self._init_collection()
    
    def _init_collection(self):
        """
        Initialize the Qdrant collection with proper vector parameters
        """
        try:
            collections_response = self.client.get_collections()
            existing_collections = [collection.name for collection in collections_response.collections]
            
            if self.collection_name not in existing_collections:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")
                
        except Exception as e:
            logger.error(f"Error initializing collection: {e}")
            raise
    
    def add_documents(self, documents: List[Dict[str, str]], embeddings_func=None):
        """
        Add documents to the vector store with embeddings
        If embeddings_func is provided, use it to create embeddings
        Otherwise, use a mock embedding function for now
        """
        points = []
        
        for doc in documents:
            # Generate embeddings if function is provided
            if embeddings_func:
                embedding = embeddings_func(doc['content'])
            else:
                # Mock embedding function (using hash to create pseudo-vector)
                embedding = self._mock_embedding(doc['content'])
            
            # Create unique ID for the document
            doc_id = str(uuid4())
            
            point = PointStruct(
                id=doc_id,
                vector=embedding,
                payload={
                    'content': doc['content'],
                    'title': doc['title'],
                    'id': doc['id'],
                    'path': doc['path'],
                    'module': doc['module'],
                    'source_file': doc['source_file']
                }
            )
            points.append(point)
        
        # Upload points to Qdrant
        self.client.upload_points(
            collection_name=self.collection_name,
            points=points
        )
        
        logger.info(f"Added {len(points)} documents to collection")
    
    def _mock_embedding(self, text: str) -> List[float]:
        """
        Mock embedding function for testing (replace with real embedding model later)
        Creates a pseudo-vector based on the text hash
        """
        # This is a dummy embedding function - would replace with real model
        hash_obj = hashlib.md5(text.encode())
        hex_dig = hash_obj.hexdigest()
        
        # Convert hex digest to numbers in range [-1, 1]
        embedding = []
        for i in range(0, len(hex_dig), 2):
            hex_pair = hex_dig[i:i+2]
            val = int(hex_pair, 16) / 255.0  # Normalize to [0, 1]
            val = val * 2 - 1  # Scale to [-1, 1]
            embedding.append(val)
        
        # Pad or truncate to fixed size
        while len(embedding) < self.vector_size:
            embedding.append(0.0)
        embedding = embedding[:self.vector_size]
        
        return embedding
    
    def similarity_search(self, query_embedding: List[float], k: int = 5) -> List[Dict]:
        """
        Perform similarity search in the vector store
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=k
        )
        
        results = []
        for hit in search_result:
            results.append({
                'content': hit.payload['content'],
                'title': hit.payload['title'],
                'id': hit.payload['id'],
                'path': hit.payload['path'],
                'module': hit.payload['module'],
                'score': hit.score
            })
        
        return results
    
    def search_by_metadata(self, filters: Dict[str, str], k: int = 5) -> List[Dict]:
        """
        Search documents by metadata filters (e.g., by module)
        """
        conditions = []
        for key, value in filters.items():
            condition = FieldCondition(
                key=key,
                match=MatchValue(value=value)
            )
            conditions.append(condition)
        
        search_request = SearchRequest(
            vector=[0.0] * self.vector_size,  # Dummy vector for metadata search
            filter=Filter(must=conditions),
            limit=k
        )
        
        result = self.client.search(
            collection_name=self.collection_name,
            search_request=search_request
        )
        
        results = []
        for hit in result:
            results.append({
                'content': hit.payload['content'],
                'title': hit.payload['title'],
                'id': hit.payload['id'],
                'path': hit.payload['path'],
                'module': hit.payload['module'],
                'score': hit.score
            })
        
        return results


if __name__ == "__main__":
    # This would be used for testing when credentials are available
    print("Qdrant client module loaded")