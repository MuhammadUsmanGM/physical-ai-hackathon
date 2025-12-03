#!/usr/bin/env python3
"""
Setup script for the Physical AI & Humanoid Robotics RAG system
"""

import subprocess
import sys
import os

def install_requirements():
    """
    Install required Python packages
    """
    print("Installing required packages...")
    
    # Change to the rag directory
    rag_dir = os.path.join(os.path.dirname(__file__), "rag")
    requirements_path = os.path.join(rag_dir, "requirements.txt")
    
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", requirements_path])
        print("Successfully installed requirements!")
    except subprocess.CalledProcessError as e:
        print(f"Error installing requirements: {e}")
        sys.exit(1)

def main():
    """
    Main setup function
    """
    print("Setting up Physical AI & Humanoid Robotics RAG system...")
    
    # Install requirements
    install_requirements()
    
    # Create .env file if it doesn't exist
    env_path = os.path.join(os.path.dirname(__file__), ".env")
    env_example_path = os.path.join(os.path.dirname(__file__), ".env.example")
    
    if not os.path.exists(env_path):
        print(f"Creating .env file from example...")
        with open(env_example_path, 'r') as example_file:
            env_content = example_file.read()
        
        with open(env_path, 'w') as env_file:
            env_file.write(env_content)
        
        print("Created .env file. Please update it with your actual API keys.")
    
    print("\nSetup complete!")
    print("1. Update the .env file with your actual API keys")
    print("2. Run the application using: python main.py")

if __name__ == "__main__":
    main()