#!/usr/bin/env python3
from pymongo import MongoClient
import gridfs
import yaml
import os
import sys
from subprocess import Popen, PIPE
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')
from models.model import Goal

MONGO_URI = "mongodb+srv://jatinpatil2003:iqEtcmVkve9wxP15@mydb.nk83zvt.mongodb.net/"

# mongodb = MongoClient(MONGO_URI)
# mongodb = MongoClient('mongodb', 27017)
mongodb = MongoClient('localhost', 27017)


db = mongodb['autonav']
fs = gridfs.GridFS(db, collection='maps')
pose_collection = db['pose']

parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))

maps_dir = os.path.join(os.getcwd(), 'maps')

print('done connection')

def deleteDatabase():
    try:
        mongodb.drop_database('autonav')
        return True
    except Exception as e:
        print(f"An error occurred: {e}")
        return False

def saveMap(file_name: str):
    pgm_file = maps_dir + '/' + file_name + '.png'
    print(f"\n\n\n\n\n\n Execute Save Map \n\n\n\n\n")
    try:
        for old in fs.find({"filename": f"{file_name}.pgm"}):
            fs.delete(old._id)

        with open(pgm_file, 'rb') as file:
            data = file.read()
            fs.put(data, filename=f'{file_name}.pgm')
    except Exception as e:
        print(f"\n\n\n\n\n\n{e}\n\n\n\n\n")
        return False

    yaml_file = maps_dir + '/' + file_name + '.yaml'
    try:
        for old in fs.find({"filename": f"{file_name}.yaml"}):
            fs.delete(old._id)
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            yaml_str = yaml.dump(data)
            fs.put(yaml_str.encode('utf-8'), filename=f'{file_name}.yaml')
    except Exception as e:
        print(f"\n\n\n\n\n\n{e}\n\n\n\n\n")
        return False

    os.remove(pgm_file)
    os.remove(yaml_file)

    return True

def loadMap(file_name: str):
    pgm_file = maps_dir + '/' + file_name + '.pgm'
    try:
        file_data = fs.find_one({'filename': f'{file_name}.pgm'})
        if file_data:
            with open(pgm_file, 'wb') as file:
                file.write(file_data.read())
    except:
        return False

    yaml_file = maps_dir + '/' + file_name + '.yaml'
    try:
        file_data = fs.find_one({'filename': f'{file_name}.yaml'})
        if file_data:
            yaml_data = file_data.read().decode('utf-8')
            with open(yaml_file, 'w') as file:
                file.write(yaml_data)
    except:
        return False
    
    return True

def listMaps():
    try:
        files = fs.find()
        map_files = set()

        for file in files:
            base_filename = file.filename.rsplit('.', 1)[0]
            map_files.add(base_filename)

        return list(map_files)


    except Exception as e:
        print(f"An error occurred: {e}")
        return []

# saveMap('cafe')
# saveMap('cafe_3d')
# saveMap('new_cafe')
# loadMap('cafe')
# loadMap('cafe_3d')
# loadMap('new_cafe')
# print(listMaps())

def saveGoal(goal: Goal):
    goal_db = {
        'map_name': goal.map_name,
        'name': goal.name,
        'x': goal.x,
        'y': goal.y,
        'theta': goal.theta
    }
    try:
        pose_collection.insert_one(goal_db)
        return True
    except Exception as e:
        print(f"An error occurred: {e}")
        return False

def getGoal(map_name, name):
    try:
        goals = pose_collection.find({"map_name": map_name})
        for goal in goals:
            if goal["name"] == name:
                pose = {
                    "x": goal["x"],
                    "y": goal["y"],
                    "theta": goal["theta"]
                }
        return pose
    except Exception as e:
        return {}
    
def listGoal(map_name):
    try:
        goals = pose_collection.find({"map_name": map_name})
        pose = []
        for goal in goals:
            pose.append(goal["name"])
        return pose
    except Exception as e:
        return []

def updateGoals(map_name: str):
    # Find all goals with map_name "temp"
    temp_goals = list(pose_collection.find({"map_name": "temp"}))

    updated_count = 0

    for goal in temp_goals:
        # Check if a goal with same name already exists in the target map
        existing = pose_collection.find_one({
            "map_name": map_name,
            "name": goal["name"]
        })

        if existing:
            # Update the existing goal with the temp goal's x, y, theta
            pose_collection.update_one(
                {"_id": existing["_id"]},
                {"$set": {"x": goal["x"], "y": goal["y"], "theta": goal["theta"]}}
            )
        else:
            # No conflict: just change map_name
            pose_collection.update_one(
                {"_id": goal["_id"]},
                {"$set": {"map_name": map_name}}
            )
        updated_count += 1

    print(f"Processed {updated_count} goals from 'temp' to '{map_name}'")


def deleteGoals(map_name: str):
    result = pose_collection.delete_many({"map_name": map_name})
    print(f"Deleted {result.deleted_count} goals with map_name={map_name}")

def savePose(name: str, data):
    pose = {
        'map_name': 'temp',
        'name': name,
        'x': data['x'],
        'y': data['y'],
        'theta': data['theta']
    }
    try:
        result = pose_collection.update_one(
            {"map_name": 'temp', "name": name},  # match condition
            {"$set": pose},                        # update with new values
            upsert=True                            # insert if not found
        )
        if result.matched_count > 0:
            print(f"Updated pose: {map_name}:{name}")
        elif result.upserted_id:
            print(f"Inserted new pose with id {result.upserted_id}")
        return True
    except Exception as e:
        print(f"An error occurred: {e}")
        return False
    

# goal = Goal(
#     name='goal_3',
#     map_name='cafe',
#     x=0.3,
#     y=-0.4,
#     theta=1.49
# )

# saveGoal(goal)
# print(getGoal('cafe', 'goal_1'))
# print(listGoal('cafe'))