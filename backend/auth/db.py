import os
from motor.motor_asyncio import AsyncIOMotorClient
from dotenv import load_dotenv

load_dotenv()

MONGODB_URI = os.getenv("MONGODB_URI", "mongodb://localhost:27017/physical_ai")
client = AsyncIOMotorClient(MONGODB_URI)
db = client.physical_ai
users_collection = db.users

async def get_user_by_email(email: str):
    user = await users_collection.find_one({"email": email})
    if user:
        user["id"] = str(user["_id"])
    return user

async def create_user(user_data: dict):
    result = await users_collection.insert_one(user_data)
    user_data["id"] = str(result.inserted_id)
    return user_data
