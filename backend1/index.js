const express = require('express');
const cors = require('cors');
const { GoogleGenerativeAI } = require('@google/generative-ai');
const { QdrantClient } = require('@qdrant/js-client-rest');
require('dotenv').config();

const app = express();
const port = process.env.PORT || 8000;

// Enable trust proxy for Vercel
app.enable('trust proxy');

// Middleware
app.use(cors({
  origin: function(origin, callback) {
    // Allow requests with no origin (like mobile apps or curl requests)
    if (!origin) return callback(null, true);
    
    const allowedOrigins = [
      'http://localhost:3000', 
      'http://localhost:3001', 
      'http://127.0.0.1:3000', 
      'http://localhost:5173', 
      'https://physical-ai-hackathon.vercel.app',
      'https://muhammadusmangm.github.io' // GitHub Pages
    ];
    
    // Check if origin matches or is a subdomain of allowed origins (optional, but strict list is safer)
    if (allowedOrigins.indexOf(origin) !== -1 || origin.endsWith('.vercel.app') || origin.endsWith('.github.io')) {
      return callback(null, true);
    }
    
    console.log('Blocked by CORS:', origin);
    return callback(null, true); // Fallback for development/debugging
  },
  credentials: true,
  methods: ['GET', 'POST', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization']
}));

app.use(express.json());

// Initialize Clients
const genAI = new GoogleGenerativeAI(process.env.GOOGLE_API_KEY);
const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

const COLLECTION_NAME = 'physical_ai_textbook'; // Make sure this matches your Qdrant collection

// Helper to get embeddings
async function getEmbedding(text) {
  const model = genAI.getGenerativeModel({ model: "embedding-001" });
  const result = await model.embedContent(text);
  return result.embedding.values;
}

// Chat Endpoint
app.post('/chat', async (req, res) => {
  try {
    const { query, context: userContext } = req.body;

    if (!query) {
      return res.status(400).json({ error: 'Query is required' });
    }

    console.log(`[Chat] Received query: ${query}`);

    // 1. Generate Embedding for the query
    const queryEmbedding = await getEmbedding(query);

    // 2. Search Qdrant for relevant context
    let retrievedContext = "";
    try {
      const searchResult = await qdrant.search(COLLECTION_NAME, {
        vector: queryEmbedding,
        limit: 3,
        with_payload: true,
      });

      retrievedContext = searchResult
        .map(item => item.payload?.content || "")
        .join("\n\n");
        
      console.log(`[Qdrant] Found ${searchResult.length} relevant chunks`);
    } catch (dbError) {
      console.error('[Qdrant] Search failed:', dbError.message);
      // Continue without context if DB fails (graceful degradation)
    }

    // 3. Construct Prompt
    // Combine user-selected context (if any) with retrieved context
    const combinedContext = `
    ${userContext ? `USER SELECTED CONTEXT:\n${userContext}\n` : ''}
    ${retrievedContext ? `RETRIEVED KNOWLEDGE:\n${retrievedContext}\n` : ''}
    `;

    const prompt = `
    You are an expert AI teaching assistant for a "Physical AI & Humanoid Robotics" course.
    Use the following context to answer the student's question. 
    If the answer is not in the context, use your general knowledge but mention that it's outside the course material.
    Be concise, encouraging, and accurate.

    CONTEXT:
    ${combinedContext}

    STUDENT QUESTION:
    ${query}
    `;

    // 4. Generate Response
    const model = genAI.getGenerativeModel({ model: "gemini-pro" });
    const result = await model.generateContent(prompt);
    const response = await result.response;
    const text = response.text();

    res.json({ response: text });

  } catch (error) {
    console.error('[Chat] Error:', error);
    res.status(500).json({ 
      error: 'Internal Server Error', 
      details: error.message 
    });
  }
});

// Root Endpoint
app.get('/', (req, res) => {
  res.json({ 
    status: 'ok', 
    message: 'Physical AI Chatbot Backend is running',
    endpoints: {
      chat: 'POST /chat',
      health: 'GET /health'
    }
  });
});

// Health Check
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'chatbot-backend' });
});

if (require.main === module) {
  app.listen(port, () => {
    console.log(`Chatbot Backend running on port ${port}`);
  });
}

module.exports = app;

