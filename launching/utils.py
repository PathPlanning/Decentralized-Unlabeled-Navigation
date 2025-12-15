import copy

import ctp_swap._ctp_swap
from matplotlib.pylab import rand
from const import *
import time
import numpy as np
import numpy.typing as npt
from typing import List, Dict, Set, Tuple, Type, Optional
import sys
from os.path import dirname
from ctp_swap.params import *
import ctp_swap
import experiment_utils

from manavlib.common.params import (
    ExperimentParams,
    HolonomicAgentParams,
    BaseAlgParams,
)


def get_algorithms(alg_params: List[BaseAlgParams]):

    navalg_types = {CTPSWAPParams.alg_name, SimpleNavParams.alg_name}
    planner_types = {ThetaStarParams.alg_name, DirectPlannerParams.alg_name}
    follower_types = {ORCAAlgParams.alg_name, DirectFollowerParams.alg_name}

    alg_table = {
        CTPSWAPParams.alg_name: ctp_swap.CTPSWAPAgent,
        SimpleNavParams.alg_name: ctp_swap.SimpleAgent,
        ThetaStarParams.alg_name: ctp_swap.ThetaPlanner,
        DirectPlannerParams.alg_name: ctp_swap.DirectPlanner,
        ORCAAlgParams.alg_name: ctp_swap.ORCAFollower,
        DirectFollowerParams.alg_name: ctp_swap.DirectFollower,
    }

    navalg = None
    planner = None
    follower = None

    for curr_params in alg_params:
        if curr_params.alg_name in navalg_types:
            navalg = (alg_table[curr_params.alg_name], curr_params)
        elif curr_params.alg_name in planner_types:
            planner = (alg_table[curr_params.alg_name], curr_params)
        elif curr_params.alg_name in follower_types:
            follower = (alg_table[curr_params.alg_name], curr_params)

    return [navalg, planner, follower]


class Summary:
    """
    A class to store and summarize the results of an experiment.

    Parameters
    ----------
    agents_num : int, optional
        The number of agents involved in the experiment (default is 0).
    success : bool, optional
        Indicates whether the experiment was successful (default is False).
    collision : int, optional
        The total number of agent-to-agent collisions (default is 0).
    collision_obst : int, optional
        The total number of agent-to-obstacle collisions (default is 0).
    makespan : int, optional
        The total time taken for all agents to reach their goals (default is 0).
    flowtime : int, optional
        The sum of the time steps taken by all agents (default is 0).
    runtime : float, optional
        The total runtime of the experiment in seconds (default is 0).
    mean_groups : float, optional
        The average number of groups formed during the experiment (default is 0).
    mean_groups_size : float, optional
        The average size of the groups formed during the experiment (default is 0).
    """

    def __init__(
        self,
        agents_num: int = 0,
        success: bool = False,
        collision: int = 0,
        collision_obst: int = 0,
        makespan: float = 0.0,
        flowtime: float = 0.0,
        runtime: float = 0.0,
        mean_groups: float = 0.0,
        mean_groups_size: float = 0.0,
    ):
        self.agents_num: int = agents_num
        self.success: bool = success
        self.collision: int = collision
        self.collision_obst: int = collision_obst
        self.makespan: float = makespan
        self.flowtime: float = flowtime
        self.runtime: float = runtime
        self.mean_groups: float = mean_groups
        self.mean_groups_size: float = mean_groups_size

    def __str__(self):
        return f"{self.success:>7} {self.collision:>10} {self.collision_obst:>15} {self.makespan:>10.3f} {self.flowtime:>10.3f} {self.runtime:>10.3f} {self.mean_groups:>15.3f} {self.mean_groups_size:>20.3} {self.agents_num:>10}"

    @staticmethod
    def header():
        success_str = "success"
        collision_str = "collision"
        collision_obst_str = "collision_obst"
        makespan_str = "makespan"
        flowtime_str = "flowtime"
        runtime_str = "runtime"
        ag_num_str = "number"
        mean_groups_str = "mean_groups"
        mean_groups_size_str = "mean_groups_size"
        return f"{success_str:>7} {collision_str:>10} {collision_obst_str:>15} {makespan_str:>10} {flowtime_str:>10} {runtime_str:>10} {mean_groups_str:>15} {mean_groups_size_str:>20} {ag_num_str:>10}"


class Simulation:

    def __init__(
        self,
        start_states: npt.NDArray,
        goal_states: npt.NDArray,
        grid_map: npt.NDArray,
        cell_size: float,
        obstacles: List[List[npt.NDArray]],
        path_planner,
        agents_num: int,
        agents_params: List[HolonomicAgentParams],
        alg_params: BaseAlgParams,
        exp_params: ExperimentParams,
        save_log: bool = True,
    ) -> None:

        self.start_states = start_states
        self.goal_states = goal_states
        self.grid_map = grid_map.astype(np.int8)
        self.cell_size = cell_size
        self.agents_num = agents_num
        self.agents_params = agents_params
        self.alg_params = alg_params
        self.exp_params = exp_params
        self.save_log = save_log
        self.obstacles = obstacles
        self.path_planner = path_planner

    def init_exp(self):

        self.start_states = self.start_states[:, 0:2]
        self.goal_states = self.goal_states[:, 0:2]

        self.max_steps = self.exp_params.max_steps
        self.dt = self.exp_params.timestep
        self.success = False

        if (
            len(self.start_states) < self.agents_num
            or len(self.goal_states) < self.agents_num
        ):
            raise ValueError(
                "Number of agents must be less than or equal to the number of starting or goal states"
            )
        self.current_states = self.start_states[: self.agents_num].copy()
        self.goal_states = [
            self.goal_states[i].astype(np.float64) for i in range(self.agents_num)
        ]
        self.actions = np.zeros((self.agents_num, ACTION_DIM), dtype=np.float64)
        self.goal_assignment = np.zeros((self.agents_num, STATE_DIM), dtype=np.float64)

        self.agents = []
        ag_params = ctp_swap.convert_agent_params(self.agents_params[0])

        (NavAlg, nav_params), (Planner, planner_params), (Follower, follower_params) = (
            get_algorithms(self.alg_params)
        )

        nav_params = ctp_swap.convert_alg_params(nav_params)
        follower_params = ctp_swap.convert_alg_params(follower_params)
        for a_id in range(self.agents_num):
            state = ctp_swap.State()
            state.position = self.current_states[a_id, 0:2]
            state.velocity = np.array([0, 0], dtype=np.float64)

            follower = Follower(ag_params, follower_params, self.obstacles)

            self.agents.append(
                NavAlg(
                    a_id,
                    state,
                    self.goal_states,
                    ag_params,
                    nav_params,
                    self.path_planner,
                    follower,
                )
            )

        self.steps_log = np.zeros(
            (self.max_steps + 1, self.agents_num, STATE_DIM), dtype=np.float64
        )

        self.goal_log = np.zeros(
            (self.max_steps + 1, self.agents_num, STATE_DIM), dtype=np.float64
        )

        self.steps_log[0, :, 0:2] = self.current_states
        self.goal_log[0, :, 0:2] = self.current_states
        self.t = 0
        self.collisions = 0
        self.collisions_obst = 0
        self.groups_sum = 0
        self.grops_sizes_sum = 0
        self.agents_r_vis = [
            self.agents_params[a_id].r_vis for a_id in range(self.agents_num)
        ]
        self.agents_sizes = [
            self.agents_params[a_id].size for a_id in range(self.agents_num)
        ]
        self.neighbors_log = []
        self.goals_reached_time = np.ones(len(self.current_states), dtype=np.int32) * -1

        self.experiment_tool = experiment_utils.Experiment(
            self.agents_num, self.agents_sizes, self.agents_r_vis, self.obstacles
        )

    def run_experiment(self):
        self.init_exp()
        self.start_time = time.time()
        if not self.initialize_agents():
            return
        for step in range(self.max_steps):
            self.update_states_info()
            self.compute_actions()
            self.execute_actions()
            self.check_collisions()
            self.check_collisions_obst()

            if self.collisions_obst:
                break

            self.t += 1
            self.update_log()

            if self.collisions or self.check_success():
                break
        if not self.save_log:
            return self.create_summary()
        elif self.t < self.max_steps:
            self.steps_log = self.steps_log[: self.t + 1]
            self.goal_log = self.goal_log[: self.t + 1]
        return self.create_summary(), self.steps_log, self.goal_log, self.neighbors_log

    def create_summary(self):
        return Summary(
            self.agents_num,
            self.success,
            self.collisions,
            self.collisions_obst,
            self.t * self.dt,
            (
                np.sum(self.goals_reached_time * self.dt)
                if self.success
                else self.t * self.agents_num * self.dt
            ),
            float(time.time() - self.start_time),
            self.groups_sum / self.t,
            self.grops_sizes_sum / self.t,
        )

    def initialize_agents(self) -> bool:
        for agent in self.agents:
            if not agent.initialize():
                return False

        return True

    def update_states_info(self) -> Tuple[int, float]:
        
        for a_id, agent in enumerate(self.agents):
            state = ctp_swap.State()
            state.position = self.current_states[a_id]
            state.velocity = self.actions[a_id]
            agent.update_state_info(state)

        self.experiment_tool.update_state(self.current_states[:, 0:2])
        
        
        neighbors_info, neighbors_ids = self.update_neighbors_info()

        for a_id, agent in enumerate(self.agents):
            agent.update_neighbors_info(neighbors_info[a_id])

        if self.save_log:
            self.neighbors_log.append(neighbors_ids)


    def compute_actions(self) -> npt.NDArray:
        
        for a_id, agent in enumerate(self.agents):
            action = agent.compute_action()
            self.actions[a_id] = action.velocity
            self.goal_assignment[a_id] = action.goal
        
    def execute_actions(self):
        for a_id in range(len(self.actions)):
            self.current_states[a_id, 0:2] = (
                self.current_states[a_id, 0:2] + self.actions[a_id] * self.dt
            )

    def check_collisions(self):
        self.collisions += self.experiment_tool.check_collisions()

    def check_collisions_obst(self) -> int:
        self.collisions_obst += self.experiment_tool.check_collisions_obstacles()

    def check_success(self):

        for a_id, a_pos in enumerate(self.current_states[:, 0:2]):
            goal_pos = self.goal_assignment[a_id]
            if np.linalg.norm(goal_pos - a_pos) < self.exp_params.xy_goal_tolerance:
                if self.goals_reached_time[a_id] == -1:
                    self.goals_reached_time[a_id] = self.t
            else:
                self.goals_reached_time[a_id] = -1
        self.success = np.all(self.goals_reached_time > 0)
        return self.success

    def update_neighbors_info(self):
        groups = self.experiment_tool.compute_connected_groups()
        groups_num = len(groups)

        neighbors_info = [list() for _ in range(self.agents_num)]
        neighbors_ids = [set() for _ in range(self.agents_num)]
        messages = []

        for a_id in range(self.agents_num):
            messages.append(self.agents[a_id].send_message())

        agents_in_group = 0
        for group in groups:
            group_messages = []
            agents_in_group += len(group)
            for a_id in group:
                group_messages.append(messages[a_id])

            for a_id in group:
                neighbors_info[a_id] = group_messages
                neighbors_ids[a_id] = group

        agents_in_group /= groups_num
        self.groups_sum += groups_num
        self.grops_sizes_sum += agents_in_group
        return neighbors_info, neighbors_ids

    def update_log(self):
        if self.save_log:
            self.steps_log[self.t] = self.current_states
            self.goal_log[self.t] = self.goal_assignment

    # def compute_neighbors(self) -> List[Set[int]]:
    #     agents_num = len(self.current_states)
    #     neighbors = [set() for _ in range(agents_num)]
    #     tree = KDTree(self.current_states)

    #     for a_id, pos in enumerate(self.current_states):
    #         n_ind = tree.query_radius([pos], self.agents_r_vis[a_id])
    #         neighbors[a_id] = set(n_ind[0])

    #     return neighbors

    # def compute_neighbors_networks(self, neighbors: List[Set[int]]) -> List[Set[int]]:
    #     groups = []
    #     agents_num = len(neighbors)
    #     considered = set()
    #     for a_id in range(agents_num):
    #         if a_id in considered:
    #             continue
    #         considered.add(a_id)
    #         group = set()
    #         queue = [a_id]
    #         while len(queue):
    #             current = queue.pop()
    #             group.add(current)
    #             for n_id in neighbors[current]:
    #                 if n_id not in group:
    #                     queue.append(n_id)
    #         groups.append(group)
    #         considered.update(group)
    #     return groups
