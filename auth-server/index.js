import express from 'express';
import cors from 'cors';
import pg from 'pg';
import dotenv from 'dotenv';
import bcrypt from 'bcryptjs';
import jwt from 'jsonwebtoken';

dotenv.config();
const { Pool } = pg;

const app = express();
const port = 4000;

app.enable('trust proxy');

// Database connection
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  },
  connectionTimeoutMillis: 5000, // Fail after 5 seconds if cannot connect
  idleTimeoutMillis: 30000,
  allowExitOnIdle: true, // Allow Node to exit if pool is idle (good for Vercel)
});

// Ensure Users Table Exists
const ensureTableExists = async () => {
  console.log('Checking database connection and table...');
  const createTableQuery = `
    CREATE TABLE IF NOT EXISTS users (
      id SERIAL PRIMARY KEY,
      name VARCHAR(255) NOT NULL,
      email VARCHAR(255) UNIQUE NOT NULL,
      password VARCHAR(255) NOT NULL,
      created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );
  `;
  try {
    await pool.query(createTableQuery);
    console.log('Users table checked/created successfully');
  } catch (err) {
    console.error('Error creating users table:', err);
  }
};

// Don't await this at top level, but log it.
ensureTableExists();

// CORS Middleware
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost:3001', 'http://127.0.0.1:3000', 'http://localhost:5173', 'https://physical-ai-hackathon.vercel.app', 'https://muhammadusmangm.github.io'],
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization']
}));

app.use(express.json());

// Debug Middleware
app.use((req, res, next) => {
  console.log(`[Request] ${req.method} ${req.path} - Origin: ${req.headers.origin}`);
  next();
});

// --- Auth Endpoints ---

// SIGNUP
app.post('/api/auth/signup', async (req, res) => {
  try {
    // Ensure table exists (serverless cold start safety)
    await ensureTableExists();

    const { name, email, password } = req.body;

    if (!name || !email || !password) {
      return res.status(400).json({ error: 'Missing fields' });
    }

    // Check if user exists
    const userCheck = await pool.query('SELECT * FROM users WHERE email = $1', [email]);
    if (userCheck.rows.length > 0) {
      return res.status(400).json({ error: 'User already exists' });
    }

    // Hash password
    const salt = await bcrypt.genSalt(10);
    const hashedPassword = await bcrypt.hash(password, salt);

    // Insert user
    const newUser = await pool.query(
      'INSERT INTO users (name, email, password) VALUES ($1, $2, $3) RETURNING id, name, email, created_at',
      [name, email, hashedPassword]
    );

    const user = newUser.rows[0];

    // Generate Token
    const token = jwt.sign(
      { id: user.id, email: user.email },
      process.env.BETTER_AUTH_SECRET || 'fallback_secret', // Reusing the existing secret var
      { expiresIn: '7d' }
    );

    res.json({ user, token });

  } catch (err) {
    console.error('Signup Error:', err);
    res.status(500).json({ 
      error: 'Server error during signup', 
      details: err.message, // Expose error for debugging
      code: err.code // Expose Postgres error code
    });
  }
});

// LOGIN
app.post('/api/auth/login', async (req, res) => {
  try {
    const { email, password } = req.body;

    if (!email || !password) {
      return res.status(400).json({ error: 'Missing credentials' });
    }

    // Find user
    const result = await pool.query('SELECT * FROM users WHERE email = $1', [email]);
    if (result.rows.length === 0) {
      return res.status(400).json({ error: 'Invalid credentials' });
    }

    const user = result.rows[0];

    // Check password
    const isMatch = await bcrypt.compare(password, user.password);
    if (!isMatch) {
      return res.status(400).json({ error: 'Invalid credentials' });
    }

    // Generate Token
    const token = jwt.sign(
      { id: user.id, email: user.email },
      process.env.BETTER_AUTH_SECRET || 'fallback_secret',
      { expiresIn: '7d' }
    );

    // Remove password from response
    delete user.password;

    res.json({ user, token });

  } catch (err) {
    console.error(err);
    res.status(500).json({ error: 'Server error during login' });
  }
});

// GET CURRENT USER (Session Check)
app.get('/api/auth/me', async (req, res) => {
  try {
    const authHeader = req.headers.authorization;
    if (!authHeader || !authHeader.startsWith('Bearer ')) {
      return res.status(401).json({ error: 'No token provided' });
    }

    const token = authHeader.split(' ')[1];

    const decoded = jwt.verify(token, process.env.BETTER_AUTH_SECRET || 'fallback_secret');
    
    const result = await pool.query('SELECT id, name, email, created_at FROM users WHERE id = $1', [decoded.id]);
    
    if (result.rows.length === 0) {
      return res.status(404).json({ error: 'User not found' });
    }

    res.json({ user: result.rows[0] });

  } catch (err) {
    console.error('Auth check failed:', err.message);
    res.status(401).json({ error: 'Invalid token' });
  }
});

// Root Endpoint
app.get('/', (req, res) => {
  res.json({ 
    status: 'ok', 
    message: 'Auth Server is running (Custom Auth)',
    endpoints: {
      health: 'GET /health',
      signup: 'POST /api/auth/signup',
      login: 'POST /api/auth/login',
      me: 'GET /api/auth/me'
    }
  });
});

// Health check
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'auth-server' });
});

// Start server if not running on Vercel (local development)
if (!process.env.VERCEL) {
  app.listen(port, () => {
    console.log(`Auth Server running at http://localhost:${port}`);
  });
}

export default app;
