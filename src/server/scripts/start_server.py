#!/usr/bin/env python3
import os
import asyncio
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import StreamingResponse
import rclpy
from ament_index_python.packages import get_package_share_directory

from server_pkg.server_api import APICommand, ServerAPI

api = None
app = FastAPI()

@app.get("/")
async def get_dashboard(request: Request):
    return api.get_dashboard(request)

@app.get("/stats")
async def get_statistics(request: Request):
    pass

@app.post("/api")
async def api_command(message: APICommand):
    pass

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await api.connection_manager.connect(websocket)
    try:
        while True:
            await websocket.send_text(api.websocket_endpoint())
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        api.connection_manager.disconnect(websocket)
    return api.websocket_endpoint()

def main():
    rclpy.init()
    host = "0.0.0.0"
    port = 8000

    pkg_share_fp = get_package_share_directory('server')
    base_fp = os.path.join(pkg_share_fp, 'web')
    static_fp = os.path.join(base_fp, 'static')
    templates_fp = os.path.join(base_fp, 'templates')

    global app, api
    api = ServerAPI(app, templates_fp, static_fp)
    uvicorn.run(app, host=host, port=port)

if __name__ == "__main__":
    main()
