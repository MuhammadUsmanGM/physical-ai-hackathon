const express = require('express');
const { betterAuth } = require('better-auth');
const { Pool } = require('pg');
require('dotenv').config();

const app = express();
const port = 4000;

// Database connection
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

// Initialize Better Auth
const auth = betterAuth({
  database: pool,
  secret: process.env.BETTER_AUTH_SECRET,
  baseURL: process.env.BETTER_AUTH_URL,
  emailAndPassword: {
    enabled: true
  },
  // Auto-migrate database schema
  databaseHooks: {
    onInit: async (ctx) => {
      console.log('Better Auth initialized, checking schema...');
    }
  }
});

// Middleware
app.use(express.json());

// Mount Better Auth API
app.all('/api/auth/*', auth.handler);

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'auth-server' });
});

app.listen(port, () => {
  console.log(`Auth Server running at http://localhost:${port}`);
});
