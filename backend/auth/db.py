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

async def get_user_by_email(email: str):
    """Get user by email"""
    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            'SELECT * FROM users WHERE email = $1',
            email
        )
        if row:
            return dict(row)
        return None

async def create_user(user_data: dict):
    """Create a new user"""
    async with pool.acquire() as conn:
        row = await conn.fetchrow('''
            INSERT INTO users (email, name, password_hash, programming_languages, hardware_experience, onboarding_completed)
            VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING *
        ''', 
            user_data['email'],
            user_data['name'],
            user_data['password_hash'],
            user_data.get('programming_languages', {}),
            user_data.get('hardware_experience', {}),
            user_data.get('onboarding_completed', False)
        )
        return dict(row)

async def update_user(email: str, update_data: dict):
    """Update user data"""
    async with pool.acquire() as conn:
        # Build dynamic update query
        set_clauses = []
        values = []
        param_count = 1
        
        for key, value in update_data.items():
            if key != 'email':  # Don't update email
                set_clauses.append(f"{key} = ${param_count}")
                values.append(value)
                param_count += 1
        
        if not set_clauses:
            return await get_user_by_email(email)
        
        values.append(email)
        query = f'''
            UPDATE users 
            SET {', '.join(set_clauses)}
            WHERE email = ${param_count}
            RETURNING *
        '''
        
        row = await conn.fetchrow(query, *values)
        return dict(row) if row else None
