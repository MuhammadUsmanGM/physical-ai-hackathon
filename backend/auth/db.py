import os
import asyncpg
from dotenv import load_dotenv

load_dotenv()

# Neon Postgres connection
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://neondb_owner:npg_KgJCN3MyRd8S@ep-proud-fire-adxmay3t-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require")

# Connection pool
pool = None

async def init_db():
    """Initialize database connection pool"""
    global pool
    pool = await asyncpg.create_pool(DATABASE_URL, min_size=1, max_size=10)
    
    # Create users table if it doesn't exist
    async with pool.acquire() as conn:
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id SERIAL PRIMARY KEY,
                email VARCHAR(255) UNIQUE NOT NULL,
                name VARCHAR(255) NOT NULL,
                password_hash VARCHAR(255) NOT NULL,
                programming_languages JSONB DEFAULT '{}',
                hardware_experience JSONB DEFAULT '{}',
                onboarding_completed BOOLEAN DEFAULT FALSE,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')

async def close_db():
    """Close database connection pool"""
    global pool
    if pool:
        await pool.close()

async def get_session(session_token: str):
    """Get session by token from Better Auth table"""
    async with pool.acquire() as conn:
        # Better Auth uses 'session' table
        row = await conn.fetchrow(
            'SELECT * FROM "session" WHERE token = $1',
            session_token
        )
        if row:
            return dict(row)
        return None

async def get_user_by_id(user_id: str):
    """Get user by ID from Better Auth table"""
    async with pool.acquire() as conn:
        # Better Auth uses 'user' table
        row = await conn.fetchrow(
            'SELECT * FROM "user" WHERE id = $1',
            user_id
        )
        if row:
            return dict(row)
        return None

# Legacy functions kept for compatibility but redirected where possible
async def get_user_by_email(email: str):
    """Get user by email from Better Auth table"""
    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            'SELECT * FROM "user" WHERE email = $1',
            email
        )
        if row:
            return dict(row)
        return None

# Note: create_user and update_user are now handled by Node.js Auth Server
# We only need read access in Python backend
