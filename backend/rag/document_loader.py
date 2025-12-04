"""
Document loading module for Physical AI textbook
"""

import os
import glob
from pathlib import Path
import logging
from typing import List, Dict
import markdown
from bs4 import BeautifulSoup
import yaml

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DocumentLoader:
    """
    Loads documents from the book/docs directory and extracts content
    """
    
    def __init__(self, documents_dir: str = None):
        if documents_dir is None:
            # Get the directory of the current file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # Go up two levels to reach the root, then into book/docs
            # backend/rag/ -> backend/ -> root/ -> book/docs
            self.documents_dir = os.path.join(current_dir, "../../book/docs/")
        else:
            self.documents_dir = documents_dir
    
    def load_documents(self) -> List[Dict[str, str]]:
        """
        Load all markdown documents from the docs directory
        """
        documents = []
        
        # Find all markdown files recursively
        md_files = glob.glob(os.path.join(self.documents_dir, "**/*.md"), recursive=True)
        logger.info(f"Found {len(md_files)} markdown files")
        
        for file_path in md_files:
            try:
                doc_data = self._process_file(file_path)
                if doc_data:
                    documents.append(doc_data)
            except Exception as e:
                logger.error(f"Error processing file {file_path}: {e}")
        
        logger.info(f"Loaded {len(documents)} documents")
        return documents
    
    def _process_file(self, file_path: str) -> Dict[str, str]:
        """
        Process a single markdown file and extract content with metadata
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        # Parse frontmatter and content
        parts = content.split('---', 2)
        frontmatter = {}
        markdown_content = content
        
        if len(parts) >= 3:
            # Has frontmatter
            try:
                frontmatter = yaml.safe_load(parts[1])
                markdown_content = parts[2]
            except:
                markdown_content = content
        else:
            # No frontmatter, use filename as ID
            pass
        
        # Convert markdown to plain text for indexing
        html = markdown.markdown(markdown_content)
        soup = BeautifulSoup(html, 'html.parser')
        text_content = soup.get_text(separator=' ')
        
        # Extract document ID and title
        doc_id = frontmatter.get('id', Path(file_path).stem)
        title = frontmatter.get('title', Path(file_path).stem.replace('-', ' ').title())
        
        # Create relative path for the document
        rel_path = os.path.relpath(file_path, self.documents_dir)
        
        return {
            'id': doc_id,
            'title': title,
            'content': text_content.strip(),
            'path': rel_path,
            'source_file': file_path,
            'module': self._extract_module_info(rel_path)
        }
    
    def _extract_module_info(self, rel_path: str) -> str:
        """
        Extract module information from the file path
        """
        parts = rel_path.split(os.sep)
        if len(parts) > 0:
            # Get the first part which should be the module
            first_part = parts[0]
            if first_part.startswith('0'):
                # Numeric prefix indicates module directory
                module_name = first_part.split('-', 1)[-1]  # Remove number prefix
                return module_name.replace('-', ' ').title()
        return "General"


if __name__ == "__main__":
    loader = DocumentLoader()
    docs = loader.load_documents()
    print(f"Loaded {len(docs)} documents")
    for doc in docs[:3]:  # Print first 3 for preview
        print(f"- {doc['title']} ({doc['id']})")