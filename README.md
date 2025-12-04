# Physical AI & Humanoid Robotics Textbook 🤖📚

An interactive, AI-powered educational platform designed to teach the next generation of robotics engineers about Physical AI, ROS 2, and Humanoid Robotics.

![Project Banner](book/static/img/logo.png)

## 🚀 Overview

This project is a next-generation digital textbook that combines comprehensive technical content with advanced AI capabilities. Unlike traditional static textbooks, this platform offers a personalized, interactive learning experience tailored to the user's background and skill level.

It features an integrated **RAG (Retrieval-Augmented Generation) Chatbot** that acts as a 24/7 teaching assistant, capable of answering questions based specifically on the textbook's content and even explaining selected text in real-time.

## ✨ Key Features

- **🧠 AI Teaching Assistant**: A context-aware chatbot powered by Google Gemini and Qdrant that answers questions using the textbook as its knowledge base.
- **🎯 Content Personalization**: Dynamic content adjustment based on the user's programming and hardware experience (Beginner, Intermediate, Advanced).
- **📝 Text Selection Q&A**: Select any text in the book to instantly ask the AI assistant for clarification or deeper explanation.
- **🌐 Urdu Translation**: Built-in support for translating chapters into Urdu to make knowledge more accessible.
- **🔐 User Onboarding**: A multi-step onboarding process that creates a personalized profile to tailor the learning journey.
- **🎨 Stunning UI/UX**: A modern, responsive interface built with Docusaurus, featuring animated backgrounds, glassmorphism effects, and dark mode support.

## 🛠️ Tech Stack

### Frontend
- **Framework**: [Docusaurus](https://docusaurus.io/) (React-based static site generator)
- **Styling**: CSS Modules, Custom CSS variables for theming
- **Interactivity**: React Hooks, Custom Components

### Backend
- **API Framework**: [FastAPI](https://fastapi.tiangolo.com/) (Python)
- **Database**: [Neon Serverless Postgres](https://neon.tech/) (User data & profiles)
- **Vector Database**: [Qdrant](https://qdrant.tech/) (RAG knowledge base)
- **AI Model**: Google Gemini 2.0 Flash (via OpenAI-compatible client)
- **Authentication**: JWT (JSON Web Tokens)

### Development Tools
- **AI Assistant**: Claude Code (Anthropic) - Used for systematic development and code generation.

## 🏁 Getting Started

### Prerequisites
- Node.js (v18+)
- Python (v3.10+)
- Neon Postgres Account
- Google Gemini API Key
- Qdrant API Key

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/MuhammadUsmanGM/physical-ai-hackathon.git
   cd physical-ai-hackathon
   ```

2. **Setup Backend**
   ```bash
   cd backend
   pip install -r rag/requirements.txt
   # Create .env file with your API keys and Database URL
   python start_api.py
   ```

3. **Setup Frontend**
   ```bash
   cd book
   npm install
   npm start
   ```

4. **Visit the App**
   Open [http://localhost:3000](http://localhost:3000) to view the textbook.

---

*Built with ❤️ for the Future of Robotics Education.*
