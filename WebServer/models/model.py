from pydantic import BaseModel

class Pose(BaseModel):
    x: float
    y: float
    theta: float

class Goal(Pose):
    map_name: str
    name: str

class MapName(BaseModel):
    name: str

class Velocity(BaseModel):
    linear: float
    angular: float