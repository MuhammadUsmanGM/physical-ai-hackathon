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
      'https://muhammadusmangm.github.io'
    ];
    
    // Check if origin matches or is a subdomain of allowed origins
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

// Debug Middleware: Log all requests
app.use((req, res, next) => {
  console.log(`[Request] ${req.method} ${req.path} - Origin: ${req.headers.origin}`);
  next();
});

// Initialize Clients
const apiKey = process.env.GOOGLE_API_KEY || process.env.GEMINI_API_KEY;
const genAI = new GoogleGenerativeAI(apiKey);
const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

const COLLECTION_NAME = process.env.QDRANT_COLLECTION_NAME || 'physical_ai_docs_local';

// Chat Endpoint
app.post('/chat', async (req, res) => {
  try {
    const { query, context: userContext } = req.body;

    // Input validation
    if (!query || typeof query !== 'string') {
      return res.status(400).json({ error: 'Valid query is required' });
    }

    if (query.trim().length === 0) {
      return res.status(400).json({ error: 'Query cannot be empty' });
    }

    if (query.length > 2000) {
      return res.status(400).json({ error: 'Query too long (max 2000 characters)' });
    }

    console.log(`[Chat] Received query: ${query}`);

    // 1. Try to get Context (RAG)
    let retrievedContext = "";
    try {
      // Use text-embedding-004 model (ensure this matches your Qdrant embeddings)
      const embeddingModel = genAI.getGenerativeModel({ model: "text-embedding-004" });
      const embeddingResult = await embeddingModel.embedContent(query);
      const queryEmbedding = embeddingResult.embedding.values;

      // Search Qdrant
      const searchResult = await qdrant.search(COLLECTION_NAME, {
        vector: queryEmbedding,
        limit: 3,
        with_payload: true,
      });

      retrievedContext = searchResult
        .map(item => item.payload?.content || "")
        .join("\n\n");
        
      console.log(`[Qdrant] Found ${searchResult.length} relevant chunks`);
    } catch (ragError) {
      console.warn('[RAG] Retrieval failed (likely quota or DB issue). Proceeding without context.', ragError.message);
      // Fallback: Continue without context
    }

    // 2. Construct Prompt
    const combinedContext = `
${userContext ? `USER SELECTED CONTEXT:\n${userContext}\n` : ''}
${retrievedContext ? `RETRIEVED KNOWLEDGE:\n${retrievedContext}\n` : ''}
    `.trim();

    const prompt = `
You are an expert AI teaching assistant for a "Physical AI & Humanoid Robotics" course.

INSTRUCTIONS:
1. If the user input is a greeting (e.g., "hi", "hello") or a general question, answer naturally and politely using your general knowledge.
2. If the user asks a course-related question, use the provided CONTEXT to answer.
3. If the answer is not in the context, you may use your general knowledge but mention that it is outside the specific course material.
4. Be concise, encouraging, and accurate.
5. Format your responses clearly with proper structure when needed.

${combinedContext ? `CONTEXT:\n${combinedContext}\n` : ''}

STUDENT QUESTION:
${query}
    `.trim();

    // 3. Generate Response using updated model
    const model = genAI.getGenerativeModel({ 
      model: "gemini-2.0-flash" // Updated to stable model
    });
    
    const result = await model.generateContent({
      contents: [{ role: "user", parts: [{ text: prompt }] }],
      safetySettings: [
        { category: "HARM_CATEGORY_HARASSMENT", threshold: "BLOCK_MEDIUM_AND_ABOVE" },
        { category: "HARM_CATEGORY_HATE_SPEECH", threshold: "BLOCK_MEDIUM_AND_ABOVE" },
        { category: "HARM_CATEGORY_SEXUALLY_EXPLICIT", threshold: "BLOCK_MEDIUM_AND_ABOVE" },
        { category: "HARM_CATEGORY_DANGEROUS_CONTENT", threshold: "BLOCK_MEDIUM_AND_ABOVE" },
      ],
      generationConfig: {
        temperature: 0.7,
        topP: 0.8,
        topK: 40,
        maxOutputTokens: 1024,
      }
    });
    
    const response = await result.response;
    const text = response.text();

    if (!text || text.trim().length === 0) {
      throw new Error('Empty response from AI model');
    }

    console.log(`[Chat] Response generated successfully (${text.length} chars)`);
    res.json({ response: text });

  } catch (error) {
    console.error('[Chat] Error:', error);
    
    // More detailed error handling
    let errorMessage = 'Internal Server Error';
    let statusCode = 500;
    
    if (error.message.includes('API key')) {
      errorMessage = 'API key issue. Please check configuration.';
      statusCode = 503;
    } else if (error.message.includes('quota')) {
      errorMessage = 'API quota exceeded. Please try again later.';
      statusCode = 429;
    } else if (error.message.includes('not found')) {
      errorMessage = 'Model not found. Please check model configuration.';
      statusCode = 503;
    }
    
    res.status(statusCode).json({ 
      error: errorMessage, 
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
});

// Root Endpoint
app.get('/', (req, res) => {
  res.json({ 
    status: 'ok', 
    message: 'Physical AI Chatbot Backend is running',
    version: '1.0.0',
    endpoints: {
      chat: 'POST /chat',
      health: 'GET /health',
      testQdrant: 'GET /api/test-qdrant'
    }
  });
});

// Health Check
app.get('/health', (req, res) => {
  const health = {
    status: 'ok',
    service: 'chatbot-backend',
    timestamp: new Date().toISOString(),
    environment: {
      hasApiKey: !!(process.env.GOOGLE_API_KEY || process.env.GEMINI_API_KEY),
      hasQdrantUrl: !!process.env.QDRANT_URL,
      hasQdrantKey: !!process.env.QDRANT_API_KEY,
      collectionName: COLLECTION_NAME
    }
  };
  
  res.json(health);
});

// Qdrant Connection Test
app.get('/api/test-qdrant', async (req, res) => {
  try {
    const collections = await qdrant.getCollections();
    res.json({ 
      status: 'ok', 
      message: 'Qdrant connection successful', 
      collections: collections.collections.map(c => c.name),
      targetCollection: COLLECTION_NAME,
      collectionExists: collections.collections.some(c => c.name === COLLECTION_NAME)
    });
  } catch (err) {
    console.error('Qdrant Test Failed:', err);
    res.status(500).json({ 
      error: 'Qdrant connection failed', 
      details: err.message 
    });
  }
});

// 404 Handler
app.use((req, res) => {
  res.status(404).json({ 
    error: 'Not Found', 
    message: `Route ${req.method} ${req.path} does not exist`,
    availableEndpoints: ['GET /', 'GET /health', 'POST /chat', 'GET /api/test-qdrant']
  });
});

// Global Error Handler
app.use((err, req, res, next) => {
  console.error('[Global Error]:', err);
  res.status(500).json({ 
    error: 'Internal Server Error',
    message: process.env.NODE_ENV === 'development' ? err.message : 'Something went wrong'
  });
});

// Start Server
if (require.main === module) {
  app.listen(port, () => {
    console.log(`🚀 Chatbot Backend running on port ${port}`);
    console.log('Environment Check:');
    console.log(`- GOOGLE_API_KEY: ${process.env.GOOGLE_API_KEY ? '✅ Set' : '❌ MISSING'}`);
    console.log(`- QDRANT_URL: ${process.env.QDRANT_URL ? '✅ Set' : '❌ MISSING'}`);
    console.log(`- QDRANT_API_KEY: ${process.env.QDRANT_API_KEY ? '✅ Set' : '❌ MISSING'}`);
    console.log(`- Collection: ${COLLECTION_NAME}`);
  });
}

module.exports = app;