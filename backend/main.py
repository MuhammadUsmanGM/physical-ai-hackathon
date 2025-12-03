"""
Main entry point for the Physical AI & Humanoid Robotics RAG system
This integrates the Qdrant-based RAG system with the Gemini-powered agent
"""

from rag.physical_ai_agent import create_physical_ai_assistant, chat_with_physical_ai_assistant
from rag.rag_system import RAGSystem

def main():
    """
    Main function to run the Physical AI assistant
    """
    print("Initializing Physical AI & Humanoid Robotics Assistant...")
    
    # Initialize the RAG system
    rag_system = RAGSystem()
    
    print("\nPhysical AI & Humanoid Robotics Assistant Ready!")
    print("Ask me anything about Physical AI, ROS 2, Gazebo, Unity, NVIDIA Isaac, or VLA!")
    print("Type 'quit' to exit.\n")
    
    while True:
        user_input = input("You: ")
        
        if user_input.lower() in ['quit', 'exit', 'bye']:
            print("Assistant: Goodbye! Feel free to ask more questions about Physical AI anytime.")
            break
        
        if user_input.strip():
            print("Assistant:", chat_with_physical_ai_assistant(user_input))
            print()

if __name__ == "__main__":
    main()