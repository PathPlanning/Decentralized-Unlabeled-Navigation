import manavlib.io.xml_io as xml_io
import manavlib.io.movingai_io as mai_io
import manavlib.gen.tasks as agents
import manavlib.common.params as params
import manavlib.gen.maps as grid
import os
import sys
from urllib.parse import urlparse
from pathlib import Path
from manavlib.gen.polygon import compute_poligons


TASK_POSTFIX = "_task.xml"
CONFIG_POSTFIX = "_config.xml"
TASKS_DIR = "./tasks/"
MOVINGAI_DIR = "movingai_maps/"

MAPS_URLS = [
    # Insert URL to MovingAI
    # "https://movingai.com/benchmarks/mapf/random-32-32-10.map.zip",
    # "https://movingai.com/benchmarks/mapf/room-64-64-16.map.zip",
    # "https://movingai.com/benchmarks/mapf/den312d.map.zip",
    "https://movingai.com/benchmarks/mapf/maze-32-32-4.map.zip",
    # "https://movingai.com/benchmarks/mapf/empty-32-32.map.zip"
    # "https://movingai.com/benchmarks/dao/den404d.map.zip",
    # "https://movingai.com/benchmarks/mapf/room-32-32-4.map.zip",
    # "https://movingai.com/benchmarks/mapf/warehouse-20-40-10-2-1.map.zip",
    # "https://movingai.com/benchmarks/mapf/den520d.map.zip"
]

if not os.path.exists(MOVINGAI_DIR):
    os.makedirs(MOVINGAI_DIR)

maps_paths = []
for map_url in MAPS_URLS:
    map_url_obj = urlparse(map_url)
    orig_map_zip_name = os.path.basename(map_url_obj.path)
    map_zip_path = os.path.join(MOVINGAI_DIR, orig_map_zip_name)
    orig_map_name = Path(map_zip_path).stem
    map_path = os.path.join(MOVINGAI_DIR, orig_map_name)
    maps_paths.append(map_path)

    if not os.path.exists(map_path):
        print(f"Map file {map_path} not exists. Downloading...")
        os.system(f"wget {map_url} -P {MOVINGAI_DIR} -q")
        os.system(
            f"cd {MOVINGAI_DIR} && unzip {orig_map_zip_name} >> /dev/null && rm -rf {orig_map_zip_name} && cd - >> /dev/null"
        )
    else:
        print(f"Map file {map_path} exists")

    if not os.path.exists(map_path):
        print(f"Error downloading/finding file {map_path}. Exiting.")
        exit(-1)

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

for map_path in maps_paths:
    print(map_path)
    map_name = Path(map_path).stem
    path_to_tasks = os.path.join(TASKS_DIR, map_name)
    if not os.path.exists(path_to_tasks):
        os.makedirs(path_to_tasks)

    h, w, grid_map = mai_io.read_map_file(map_path)
    obstacles = compute_poligons(grid_map)
    map_path = os.path.join(path_to_tasks, "map.xml")
    xml_io.create_map_file(map_path, grid_map, cell_size, obstacles)
    for alg_params in all_alg_params:
        config_path = os.path.join(
            path_to_tasks, f"{alg_params.alg_name}{CONFIG_POSTFIX}"
        )
        xml_io.create_config_file(config_path, alg_params, exp_params)

    for task_id in range(task_num):
        starts, goals = agents.create_random_grid_map_instance(
            agents_num, grid_map, cell_size, False, False
        )
        task_file = f"{task_id}{TASK_POSTFIX}"
        task_path = os.path.join(path_to_tasks, task_file)
        xml_io.create_agents_file(task_path, starts, goals, default_ag_params)
        print(f"Task {task_file} generated in {path_to_tasks}")
