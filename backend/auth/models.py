from pydantic import BaseModel, EmailStr, Field
from typing import Optional, Dict, List
from datetime import datetime
from enum import Enum

class SoftwareBackground(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class HardwareBackground(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class UserBase(BaseModel):
    email: EmailStr
    name: str
    software_background: Optional[SoftwareBackground] = None
    hardware_background: Optional[HardwareBackground] = None
    preferred_language: str = "english"

class UserCreate(BaseModel):
    email: EmailStr
    name: str
    password: str

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class OnboardingData(BaseModel):
    programming_languages: Dict[str, str]  # {"Python": "advanced", "C++": "intermediate"}
    hardware_experience: Dict[str, str]  # {"Arduino": "intermediate", "ROS": "beginner"}

class UserInDB(UserBase):
    hashed_password: str
    programming_languages: Optional[Dict[str, str]] = None
    hardware_experience: Optional[Dict[str, str]] = None
    onboarding_completed: bool = False
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class User(UserBase):
    id: str
    programming_languages: Optional[Dict[str, str]] = None
    hardware_experience: Optional[Dict[str, str]] = None
    onboarding_completed: bool = False
    created_at: datetime

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: Optional[str] = None
