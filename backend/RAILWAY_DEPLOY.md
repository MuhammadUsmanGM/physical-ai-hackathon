# Railway Backend Deployment Guide

## Prerequisites
1. Railway account (sign up at https://railway.app)
2. GitHub repository pushed with latest changes

## Environment Variables to Set in Railway

Copy these from your local `.env` file:

```
DATABASE_URL=postgresql://neondb_owner:npg_KgJCN3MyRd8S@ep-proud-fire-adxmay3t-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require
GEMINI_API_KEY=<your_gemini_api_key>
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
QDRANT_URL=<your_qdrant_url>
QDRANT_API_KEY=<your_qdrant_api_key>
SECRET_KEY=<your_secret_key>
```

## Deployment Steps

1. **Go to Railway**: https://railway.app
2. **Click "New Project"**
3. **Select "Deploy from GitHub repo"**
4. **Choose**: `MuhammadUsmanGM/physical-ai-hackathon`
5. **Root Directory**: Set to `backend`
6. **Add Environment Variables**: 
   - Click on your service
   - Go to "Variables" tab
   - Add all the variables listed above
7. **Deploy**: Railway will automatically detect Python and deploy

## After Deployment

Your backend will be live at: `https://<your-project-name>.up.railway.app`

Update your frontend to use this URL instead of `http://localhost:8000`
