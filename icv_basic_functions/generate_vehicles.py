#!/usr/bin/env python


import math
import random
import sys
import thread
import time
import glob
import os
import sys
import time
import argparse
import logging

import numpy as np

from datetime import datetime
import time
from nav_msgs.msg import Path

try:
    sys.path.append(glob.glob('./PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Location, Rotation, Transform, Vector3D
from random import randint


global needed_generate
needed_generate = 0
global circle_center
circle_center = carla.Location(-10, -43, 0) # map/circle center
global has_set
has_set = np.zeros(1000000)
global stopped_time
stopped_time = np.zeros(1000000)

x_max = 80
x_min = -90
y_max = 40
y_min = -125

def generate_traffic_098(carla_world, tm, blueprints_ori, spawn_points_ori, delay = 0.05):
    
    global has_set
    blueprints = [x for x in blueprints_ori if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
    blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
    blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('t2')]

    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor
    synchronous_master = True
    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    max_agents = randint(60,80) #70
    recommended_points = [2,3,13,14,153,154,77,78,51,52,65,66,85,86,199,200,71,72,89,90,93,94,166,168,175,181,116,117]
    actor_list = carla_world.get_actors()
    vehicle_list = actor_list.filter("*vehicle*")

    for vehicle in vehicle_list:
        if has_set[vehicle.id] == 0:
            has_set[vehicle.id] = 1
            tm.ignore_vehicles_percentage(vehicle, 30)

    num_agents = len(vehicle_list)
    added_vehicle_num = max_agents - num_agents
    if added_vehicle_num > 2:
        added_vehicle_num = 2

    while len(batch) < added_vehicle_num:
        transform = spawn_points_ori[random.choice(recommended_points)]
        min_d = 100
        for vehicle in vehicle_list:
            d = vehicle.get_location().distance(transform.location)
            if d < min_d:
                min_d = d
            if min_d < 20:
                break
        if min_d < 20:
            continue
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
            
    print("spawn a vehicle")
    carla_client.apply_batch_sync(batch, synchronous_master)

def generate_pedestrians_098(carla_world, tm, blueprintsWalkers):
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor
    synchronous_master = True

    walkers_list = []
    all_id = []
    
    percentagePedestriansRunning = 100.0      # how many pedestrians will run
    percentagePedestriansCrossing = 100.0     # how many pedestrians will walk through the road
    # 1. take all the random locations to spawn
    spawn_points = []
    
    for i in range(100):
        spawn_point = carla.Transform()
        loc = carla_world.get_random_location_from_navigation()
        if (loc != None) and (loc.x < x_max) and (loc.x > x_min) and (loc.y < y_max) and (loc.y > y_min):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
            break
    # 2. we spawn the walker object
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = carla_client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = carla_world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = carla_client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put altogether the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = carla_world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    carla_world.wait_for_tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    carla_world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        print("spawn walks")

        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(carla_world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
    return all_actors

def generate_traffic(carla_world, blueprints_ori, spawn_points_ori, delay = 0.05):
    
    # generate surrounding vehicles   
    blueprint = random.choice(blueprints_ori)
    blueprint.set_attribute('role_name', 'autopilot')

    spawn_points = list(spawn_points_ori) # FIXME(zhcao): DO NOT USE THE SAME VARIABLE NAME WITH THE INPUT !!!!!!!!! 
    
    recommended_points = [1, 2, 3, 5, 7, 11, 12, 18, 20, 21, 22, 43, 44, 65, 71, 72, 75, 76, 77, 78, 83, 84, 102, 103, 110, 111, 118, 119, 124, 135, 136, 150, 158, 168, 173, 176, 203, 205, 238, 239, 249, 250, 274, 275]

    #set_ramdom_vehicle_number
    max_agents = randint(40,80) #70

    actor_list = carla_world.get_actors()
    vehicle_list = actor_list.filter("*vehicle*")
    num_agents = len(vehicle_list)
    # print("num of agents",num_agents)
    if num_agents < max_agents: 
        # random.shuffle(spawn_points)
        spawn_point = spawn_points[random.choice(recommended_points)]
        # if -120 < spawn_point.location.x < 100 and -150 < spawn_point.location.y < 70:
        min_d = 100
        for vehicle in vehicle_list:
            d = vehicle.get_location().distance(spawn_point.location)
            if d < min_d:
                min_d = d
        if min_d > 10:
            new_vehicle = carla_world.try_spawn_actor(blueprint, spawn_point)
            if new_vehicle:
                if "vehicle" in new_vehicle.type_id:
                    print("spawn someone")
                    new_vehicle.set_autopilot(True)
                    return

def removed_stopped_vehicle(carla_world, stopped_time_thres = 20):
    # carla_world = ego_vehicle.get_world()
    global stopped_time

    actor_list = carla_world.get_actors()
    vehicle_list = actor_list.filter("*vehicle*")

    for vehicle in vehicle_list:
        
        if vehicle.attributes['role_name'] == "ego_vehicle":
            continue
        
        if stopped_time[vehicle.id] < -100:
            continue
        
        # x_max = 80
        # x_min = -90
        # y_max = 40
        # y_min = -125

        v_loc = vehicle.get_location()
        if (v_loc.x > x_max) or (v_loc.x < x_min) or (v_loc.y > y_max) or (v_loc.y < y_min):
            print("delete vehicle move too far",v_loc.x, v_loc.y)
            stopped_time[vehicle.id] = -100000
            vehicle.destroy()
            continue
        
        velocity = vehicle.get_velocity()
        dist_from_origin = vehicle.get_location().distance(circle_center)
        if stopped_time[vehicle.id] >= 0:
            if velocity.x < 0.02 and velocity.y < 0.02:
                stopped_time[vehicle.id] = stopped_time[vehicle.id] + 1
            else:
                stopped_time[vehicle.id] = 0

        if stopped_time[vehicle.id] > stopped_time_thres:
            print("delete vehicle stay too long")
            stopped_time[vehicle.id] = -100000
            vehicle.destroy()

        # print("delete someone, vehicle.id=",vehicle.id)
        # print("stopped_time[vehicle.id]=",stopped_time[vehicle.id])
        # print("dist_from_origin=",dist_from_origin)

def removed_pedestrians(carla_world, all_actors):
    for pedestrians in all_actors:
        v_loc = pedestrians.get_location()
        print("into walker delete")
        if (v_loc.x > x_max) or (v_loc.x < x_min) or (v_loc.y > y_max) or (v_loc.y < y_min):
            print("delete pedestrians move too far",v_loc.x, v_loc.y)
            pedestrians.stop()
            carla_client.apply_batch([carla.command.DestroyActor(x) for x in pedestrians])
            continue  

def generate_remove_traffic(carla_world, delay = 0.5):
    # traffic main function
    # generate surrounding vehicles
    global needed_generate
    blueprints = carla_world.get_blueprint_library().filter('vehicle.*')
    spawn_points = carla_world.get_map().get_spawn_points()
    
    if (datetime.now() - needed_generate).seconds > 0.1:
        # Generate surrounding vehicles
        generate_traffic(carla_world, blueprints, spawn_points)  #0 for fix point, 1 for random

        # Generate pedestrians
        # generate_pedestrians(carla_world,carla_client,0)

        # Remove traffic stop and too far
        removed_stopped_vehicle(carla_world, 10/delay)
        # move_stopped_vehicle(carla_world, spawn_points, 10/delay)
        # time.sleep(delay)
        needed_generate = datetime.now()

def generate_remove_traffic_098(carla_world, tm, args, delay = 1):
    # generate surrounding vehicles
    blueprints_vehicle = carla_world.get_blueprint_library().filter('vehicle.*')
    blueprintsWalkers = carla_world.get_blueprint_library().filter(args.filterw)

    spawn_points = carla_world.get_map().get_spawn_points()

    while True:
        generate_traffic_098(carla_world, tm, blueprints_vehicle, spawn_points)
        # pedestrians = generate_pedestrians_098(carla_world, tm, blueprintsWalkers)

        removed_stopped_vehicle(carla_world, 20/delay)
        # removed_pedestrians(carla_world, pedestrians)
        carla_world.wait_for_tick()

        time.sleep(delay)

if __name__ == "__main__":
    carla_client = carla.Client('localhost', 2000)
    carla_client.set_timeout(10.0)
    # carla_world = carla_client.load_world('Town05') # this cause a lot of bugs
    carla_world = carla_client.get_world()
    tm = carla_client.get_trafficmanager(8000)
    needed_generate = datetime.now()

    # Setting simulation FPS
    # settings = carla_world.get_settings()
    # settings.fixed_delta_seconds = None
    # settings.no_rendering_mode = False # close render to make simulation faster
    # carla_world.apply_settings(settings)

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
    '--filterw',
    metavar='PATTERN',
    default='walker.pedestrian.*',
    help='pedestrians filter (default: "walker.pedestrian.*")')
    args = argparser.parse_args()


    generate_remove_traffic_098(carla_world, tm, args)

    del carla_world
    del carla_client


