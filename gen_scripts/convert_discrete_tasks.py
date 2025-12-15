import manavlib.io.xml_io as xml_io
import manavlib.io.movingai_io as mai_io
import manavlib.gen.tasks as agents
import manavlib.common.params as params
import manavlib.common.transform as transform
import manavlib.gen.polygon as polygon
import manavlib.gen.maps as maps
import os
import sys
from urllib.parse import urlparse
from pathlib import Path
from ctp_swap.params import *
import copy
import numpy as np


TASK_POSTFIX = "_task.xml"
CONFIG_POSTFIX = "_config.xml"
TASKS_DIR = "./tasks/"

DISCRETE_TASKS_DIR = "../../../tp-swap/tp-swap-exp/tasks"


maps_names = ["random-32-32-10", "room-32-32-4", "empty-32-32", "room-64-64-16", "den312d", "den520d", "warehouse-20-40-10-2-1", "den404d", "maze-32-32-4"]
# maps_names = ["room-32-32-4"]

# maps_names = ["warehouse-20-40-10-2-1"]



# agents_num = 50
task_num = 250
r_vis = 3.0
ag_size = 0.25
vel_max = 1.0
max_steps = 20000
timestep = 0.1
xy_goal_tolerance = 0.3
cell_reduction_factor = 1

orca_agents_max_num = 10
orca_time_boundary = 1.5
orca_time_boundary_obst = 2.0
orca_r_eps = 0.1

default_ag_params = params.HolonomicAgentParams()
default_ag_params.size = ag_size
default_ag_params.r_vis = r_vis
default_ag_params.vel_max = vel_max


planner_params = ThetaStarParams()
amapf_params = CTPSWAPParams()
amapf_params.amapf = True

mapf_params = CTPSWAPParams()
mapf_params.amapf = False



orca_params = ORCAAlgParams()
orca_params.timestep = timestep
orca_params.sight_radius = r_vis
orca_params.agents_max_num = orca_agents_max_num
orca_params.time_boundary = orca_time_boundary
orca_params.time_boundary_obst = orca_time_boundary_obst
orca_params.r_eps = orca_r_eps
orca_params.max_speed = vel_max



all_alg_params = {

    "orca": [planner_params, mapf_params, orca_params],
    "ctp_swap": [planner_params, amapf_params, orca_params],

}


exp_params = params.ExperimentParams()
exp_params.max_steps = max_steps
exp_params.timestep = timestep
exp_params.xy_goal_tolerance = xy_goal_tolerance


for map_name in maps_names:

    path_to_discrete_tasks = os.path.join(DISCRETE_TASKS_DIR, map_name)
    orig_map_path = os.path.join(path_to_discrete_tasks, "map.xml")

    orig_w, orig_h, orig_cs, grid_map = xml_io.read_xml_map(orig_map_path)
    obstacles = polygon.compute_poligons(grid_map, orig_cs)
    h, w, cs, grid_map = maps.reduce_cellsize(grid_map, orig_cs, cell_reduction_factor)

    path_to_tasks = os.path.join(TASKS_DIR, map_name)
    if not os.path.exists(path_to_tasks):
        os.makedirs(path_to_tasks)
    map_path = os.path.join(path_to_tasks, "map.xml")
    xml_io.create_map_file(map_path, grid_map, cs, obstacles)

    for name, alg_params in all_alg_params.items():

        config_path = os.path.join(path_to_tasks, f"{name}{CONFIG_POSTFIX}")
        xml_io.create_config_file(config_path, alg_params, exp_params)

    for task_id in range(task_num):
        task_file = f"{task_id}{TASK_POSTFIX}"
        orig_task_path = os.path.join(path_to_discrete_tasks, task_file)

        default_agent_params, orig_start_states, orig_goal_states, agents_params = (
            xml_io.read_xml_agents(orig_task_path)
        )

        start_states = np.zeros((len(orig_start_states), 3), dtype=np.float64)
        goal_states = np.zeros((len(orig_goal_states), 3), dtype=np.float64)

        for i in range(len(orig_start_states)):

            start_states[i, 0:2] = transform.convert_ij_to_xy(
                orig_start_states[i], orig_h, orig_cs
            )
            goal_states[i, 0:2] = transform.convert_ij_to_xy(
                orig_goal_states[i], orig_h, orig_cs
            )

        task_path = os.path.join(path_to_tasks, task_file)
        xml_io.create_agents_file(
            task_path, start_states, goal_states, default_ag_params
        )
        print(
            f"Task {task_file} from map {map_name} converted and saved in {path_to_tasks}"
        )
