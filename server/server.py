import socketio
from fastapi import FastAPI
import uvicorn
import asyncio
import redis.asyncio as redis

sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
fastapi_app = FastAPI()
app = socketio.ASGIApp(sio, fastapi_app)

@sio.on('connect')
async def connect(sid, environ):
    print(f'Client {sid} connected')

@sio.on('disconnect')
async def disconnect(sid):
    print(f'Client {sid} disconnected')

async def redis_listener():
    r = redis.Redis(host='localhost', port=6379)
    pubsub = r.pubsub()
    await pubsub.subscribe('vehicle_data')

    async for msg in pubsub.listen():
        if msg['type'] == 'message':
            print(f"🔁 Got from Redis: {msg['data']}")
            await sio.emit('update', {'speed': float(msg['data'])})

@fastapi_app.on_event("startup")
async def startup_event() -> None:
    asyncio.create_task(redis_listener())

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=8000)
