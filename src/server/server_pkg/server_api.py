import rclpy
from pydantic import BaseModel
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi import Request
import time
import rclpy
import threading

from server_pkg.server_utils import ConnectionManager
from server_pkg.server_ros import ServerROS

class APICommand(BaseModel):
    type: str
    data: dict[str, str]

class ServerAPI:
    def __init__(self, app, templates_fp, static_fp):
        self.templates = Jinja2Templates(directory=templates_fp)
        self.connection_manager = ConnectionManager()

        self.dashboard_ros = ServerROS()
        threading.Thread(target=self._ros2_thread, daemon=True).start()
        time.sleep(1.0)

        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        app.mount("/static", StaticFiles(directory=static_fp, follow_symlink=True), name="static")

    def websocket_endpoint(self):
        return f""

    def get_dashboard(self, request: Request):
        return self.templates.TemplateResponse("index.html", {"request": request})

    def change_estimator_callback(self, request: Request):
        if request.method == "POST":
            data = request.json()
            estimator_name = data.get("estimator_name")
            if estimator_name:
                self.dashboard_ros.switch_estimator(estimator_name)
                return {"status": "success", "message": f"Estimator changed to {estimator_name}"}
            else:
                return {"status": "error", "message": "Invalid estimator name"}
        else:
            return {"status": "error", "message": "Invalid request method"}

    def _ros2_thread(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.dashboard_ros)
        try:
            executor.spin()
        finally:
            self.dashboard_ros.destroy_node()
            rclpy.shutdown()
