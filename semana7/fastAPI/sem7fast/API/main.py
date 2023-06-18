import fastapi
from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI, File, UploadFile, Request, Body
from fastapi.responses import FileResponse, StreamingResponse
import os
from supabase import create_client, Client
import asyncio
import aiofiles
import time

app = FastAPI()

url: str = "https://qeqhovaiuqfkrjywqayz.supabase.co"
key: str = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6InFlcWhvdmFpdXFma3JqeXdxYXl6Iiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTY4NTAxNjgwOSwiZXhwIjoyMDAwNTkyODA5fQ.0tuA_64ZpGS8olQikZBDzacoWr1Hj-srdCe46-5Mq90"
supabase: Client = create_client(url, key)

bucket_name: str = "ponderadaluana"

@app.get("/list")
async def list():
    res = supabase.storage.from_(bucket_name).list()
    print(res)

@app.post("/upload")
def upload(content: UploadFile = fastapi.File(...)):    
    with open(f"../../../openCV/recebidos/crack{time.time()}.png", 'wb') as f:
        dados = content.file.read()
        f.write(dados)
    return {"status": "ok"}

list_files = os.listdir("../../../openCV/recebidos")

@app.post("/images")
def images():
    for arquivo in list_files:
        with open(os.path.join("../../../openCV/recebidos", arquivo), 'rb+') as f:
            dados = f.read()
            res = supabase.storage.from_(bucket_name).upload(f"{time.time()}_{arquivo}", dados)
    return {"message": "Image uploaded successfully"}
