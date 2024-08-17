from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from ros.node import start_ros_node
import threading

ros_thread = threading.Thread(target=start_ros_node)
ros_thread.start()

from routes import start, navigation, mapping

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Add the origin of your frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(start.router)

app.include_router(navigation.router)

app.include_router(mapping.router)

@app.on_event("shutdown")
def shutdown_event():
    ros_thread.join() 

# uvicorn index:app --reload