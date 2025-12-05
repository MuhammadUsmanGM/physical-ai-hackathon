const express = require('express');
const cors = require('cors');
const { betterAuth } = require('better-auth');
const { Pool } = require('pg');
require('dotenv').config();

const app = express();
const port = 4000;

app.enable('trust proxy'); // Required for Vercel/proxies

// Database connection
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

// Initialize Better Auth with proper origin configuration for development
const auth = betterAuth({
  database: pool,
  secret: process.env.BETTER_AUTH_SECRET,
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:4000',
  emailAndPassword: {
    enabled: true
  },
  trustedOrigins: ['http://localhost:3000', 'http://localhost:3001', 'http://127.0.0.1:3000', 'http://localhost:5173', 'https://physical-ai-hackathon.vercel.app'],
  // Auto-migrate database schema
  databaseHooks: {
    onInit: async (ctx) => {
      console.log('Better Auth initialized, checking schema...');
    }
  }
});

// Debug Middleware
app.use((req, res, next) => {
  console.log(`[Request] ${req.method} ${req.path} - Origin: ${req.headers.origin}`);
  next();
});

// Debug Middleware
app.use((req, res, next) => {
  console.log(`[Request] ${req.method} ${req.path} - Origin: ${req.headers.origin}`);
  next();
});

// CORS Middleware - MUST come before routes
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost:3001', 'http://127.0.0.1:3000', 'http://localhost:5173', 'https://physical-ai-hackathon.vercel.app'],
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization']
}));

// Middleware
app.use(express.json());

const { toNodeHandler } = require('better-auth/node');

// Mount Better Auth API
app.all('/api/auth/*', (req, res) => {

  toNodeHandler(auth)(req, res);
});

// Handle preflight requests specifically - REMOVED to let CORS middleware handle it
// app.options('/api/auth/*', (req, res) => {
//   res.sendStatus(204);
// });

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'auth-server' });
});

app.listen(port, () => {
  console.log(`Auth Server running at http://localhost:${port}`);
});
