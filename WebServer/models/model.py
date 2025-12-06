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

class Emergency(BaseModel):
    status: bool

class LedStatus(BaseModel):
    status: int

class PointInfo(BaseModel):
    map_name: str
    name: str