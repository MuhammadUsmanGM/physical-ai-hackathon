from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from jose import JWTError, jwt
from datetime import timedelta, datetime
from .models import UserCreate, UserLogin, Token, User, UserInDB, OnboardingData
from .db import get_user_by_email, create_user, update_user
from .utils import verify_password, get_password_hash, create_access_token, SECRET_KEY, ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES

router = APIRouter(prefix="/auth", tags=["auth"])
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="auth/login")

async def get_current_user(token: str = Depends(oauth2_scheme)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
    except JWTError:
        raise credentials_exception
    
    user = await get_user_by_email(email)
    if user is None:
        raise credentials_exception
    return user

@router.post("/signup", response_model=Token)
async def signup(user: UserCreate):
    db_user = await get_user_by_email(user.email)
    if db_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    hashed_password = get_password_hash(user.password)
    user_in_db = UserInDB(
        email=user.email,
        name=user.name,
        hashed_password=hashed_password,
        onboarding_completed=False
    )
    
    await create_user(user_in_db.dict())
    
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/login", response_model=Token)
async def login(user_credentials: UserLogin):
    user = await get_user_by_email(user_credentials.email)
    if not user or not verify_password(user_credentials.password, user["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user["email"]}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@router.get("/me", response_model=User)
async def read_users_me(current_user: dict = Depends(get_current_user)):
    return current_user

@router.post("/onboarding")
async def complete_onboarding(
    onboarding_data: OnboardingData,
    current_user: dict = Depends(get_current_user)
):
    """Complete user onboarding with programming languages and hardware experience"""
    update_data = {
        "programming_languages": onboarding_data.programming_languages,
        "hardware_experience": onboarding_data.hardware_experience,
        "onboarding_completed": True,
        "updated_at": datetime.utcnow()
    }
    
    await update_user(current_user["email"], update_data)
    
    return {"message": "Onboarding completed successfully"}
