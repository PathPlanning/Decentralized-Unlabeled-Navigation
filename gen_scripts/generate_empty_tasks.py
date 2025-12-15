import manavlib.io.xml_io as new_io
import manavlib.gen.tasks as agents
import manavlib.common.params as params
import manavlib.gen.maps as grid
import os
import sys


TASK_POSTFIX = "_task.xml"
CONFIG_POSTFIX = "config.xml"
TASK_DIR = "./tasks"
MAP_PATH = os.path.join(TASK_DIR, "map.xml")


class TestAlgParams(params.BaseAlgParams):
    alg_name = "test_alg"
    def __init__(self):
        super().__init__()


agents_num = 50
task_num = 10
h_map = 32
w_map = 32
r_vis = 3
ag_size = 0.3
vel_max = 1.0
max_steps = 1000
timestep = 0.1
xy_goal_tolerance = 0.3

default_ag_params = params.HolonomicAgentParams()
default_ag_params.size = ag_size
default_ag_params.r_vis = r_vis
default_ag_params.vel_max = vel_max

all_alg_params = [TestAlgParams()]

exp_params = params.ExperimentParams()
exp_params.max_steps = max_steps
exp_params.timestep = timestep
exp_params.xy_goal_tolerance = xy_goal_tolerance

cell_size = 1.0
h, w, grid_map = grid.create_empty_grid(w_map, h_map, cell_size)
new_io.create_map_file(MAP_PATH, grid_map, cell_size)


for alg_params in all_alg_params:
    config_path = os.path.join(TASK_DIR, f"{CONFIG_POSTFIX}")
    new_io.create_config_file(config_path, alg_params, exp_params)

for task_id in range(task_num):
    starts, goal = agents.create_random_empty_instance(
        agents_num, h, w, cell_size, True, False
    )
    task_file = f"{task_id}{TASK_POSTFIX}"
    task_path = os.path.join(TASK_DIR, task_file)
    new_io.create_agents_file(task_path, starts, goal, default_ag_params)
    print(f"Task {task_file} generated in {TASK_DIR}")
