from PIL import Image, ImageDraw, ImageOps, ImageFont
from PIL import ImageFilter
from random import randint
from typing import List, Optional, Tuple, Union, Set
import numpy.typing as npt
import numpy as np
from const import *
import cv2


def convert_xy_to_ij(xy, grid_h, cs=1.0):
    j = int(xy[0] // cs)
    i = grid_h - int(xy[1] // cs)
    return [i, j]


def convert_ij_to_xy(ij, grid_h, cs):
    i = ij[0]
    j = ij[1]
    x = (j + 0.5) * cs
    y = (grid_h - i - 0.5) * cs
    return np.array([x, y], dtype=np.float64)


def convert_xy_to_pillow(xy, grid_h, cs, scale):
    x = xy[0] * scale
    y = (grid_h * cs - xy[1]) * scale
    return [x, y]


def convert_ij_to_pillow(ij, cs, scale):
    x = (ij[1] + 0.5) * cs * scale
    y = (ij[0] + 0.5) * cs * scale
    return [x, y]


def draw_segment(
    draw_obj: ImageDraw,
    p1,
    p2,
    scale: Union[float, int],
    grid,
    cell_size,
    color=(255, 0, 0),
):
    grid_h, grid_w = grid.shape
    p1 = convert_xy_to_pillow(p1, grid_h, cell_size, scale)
    p2 = convert_xy_to_pillow(p2, grid_h, cell_size, scale)
    draw_obj.line(
        (*p1, *p2),
        fill=color,
        width=int(0.15 * scale),
    )


def draw_object(draw_obj, pos, size, grid_h, cell_size, scale, color, figure="rounded", number = None):

    pos = convert_xy_to_pillow(pos, grid_h, cell_size, scale)
    size = size * scale 
    top_left = (pos[0] - size, pos[1] - size)
    bottom_right = (pos[0] + size, pos[1] + size)

    if number is not None:
        font = ImageFont.load_default()
# draw.text((x, y),"Sample Text",(r,g,b))
        draw_obj.text(pos, str(number),(0,0,0), font=font)

    if figure == "circle":
        draw_obj.ellipse([top_left, bottom_right], fill=color, width=0.0)
    if figure == "rounded":
        draw_obj.rounded_rectangle([top_left, bottom_right], fill=color, width=0.0, radius=1)
    if figure == "rectangle":
        draw_obj.rectangle([top_left, bottom_right], fill=color, width=0.0)


def draw_map(
    draw_obj: ImageDraw,
    grid_map: npt.NDArray,
    cell_size,
    obstacles=None,
    scale: Union[float, int] = 1,
    grid_map_inf = None
):
    grid_h, grid_w = grid_map.shape



    gap = 0.3
    size = cell_size - cell_size * gap
    size = size / 2
    for row in range(grid_h):
        for col in range(grid_w):
            if grid_map[row, col] == MAP_OBSTACLE:
                pos = convert_ij_to_xy((row, col), grid_h, cell_size)
                draw_object(draw_obj, pos, size, grid_h, cell_size, scale, (120, 120, 120))

    if grid_map_inf is not None:
        gap = 0.3
        size = cell_size - cell_size * gap
        size = size / 2
        for row in range(grid_h):
            for col in range(grid_w):
                if grid_map_inf[row, col] == 100 and grid_map[row, col] != MAP_OBSTACLE:
                    pos = convert_ij_to_xy((row, col), grid_h, cell_size)
                    draw_object(draw_obj, pos, size, grid_h, cell_size, scale, (200, 200, 250))

    if obstacles is not None:
        for polygon_points in obstacles:
            for i, p1 in enumerate(polygon_points[:-1]):
                draw_segment(draw_obj, p1, polygon_points[i + 1], scale, grid_map, cell_size)
                draw_object(draw_obj, p1, 0.1, grid_h, cell_size, scale, (200, 0, 0))
            draw_segment(draw_obj, polygon_points[-1], polygon_points[0], scale, grid_map, cell_size)
            draw_object(draw_obj, polygon_points[-1], 0.1, grid_h, cell_size, scale, (200, 0, 0))


def draw_goal(draw_obj: ImageDraw, goal: npt.NDArray, grid_h, cell_size, scale: Union[float, int]):
    fill_color = (231, 76, 60)
    # size = (cell_size - 0.1 * cell_size) / 2
    size = 0.35
    draw_object(draw_obj, goal, size, grid_h, cell_size, scale, fill_color)


def draw_dyn_object(
    draw_obj: ImageDraw,
    path: npt.NDArray,
    ag_param,
    step: int,
    frame_num: int,
    frames_per_step: int,
    scale: Union[float, int],
    cell_size,
    grid_h,
    color: Tuple[int, int, int],
    goal_assignment: npt.NDArray = None,
    neighbors: List[List[npt.NDArray]] = None,
    number=None
):

    path_len = len(path)
    curr_x, curr_y = path[min(path_len - 1, step)]
    next_x, next_y = path[min(path_len - 1, step + min(frame_num, 1))]

    dx = frame_num * (next_x - curr_x) / frames_per_step
    dy = frame_num * (next_y - curr_y) / frames_per_step
    size = ag_param.size
    draw_object(
        draw_obj,
        [curr_x + dx, curr_y + dy],
        size,
        grid_h,
        cell_size,
        scale,
        color,
        "circle",
        number=number
    )
    agent_pos = [curr_x + dx, curr_y + dy]
    
    # agent_pos = (float(curr_j + dj) * scale, float(curr_i + di) * scale)
    if goal_assignment is not None:
        goal_pos = goal_assignment[min(path_len - 1, step)]
        draw_obj.line(
            (*convert_xy_to_pillow(agent_pos, grid_h, cell_size, scale), *convert_xy_to_pillow(goal_pos, grid_h, cell_size, scale)),
            fill=(10, 200, 10),
        )

    if neighbors is None:
        return

    
    
    for neighbor_xy, neighbor_next_xy in neighbors:
        n_dx = frame_num * (neighbor_xy[0] - neighbor_next_xy[0]) / frames_per_step
        n_dy = frame_num * (neighbor_xy[1] - neighbor_next_xy[1]) / frames_per_step
        neighbor_pos = [neighbor_xy[0] + n_dx, neighbor_xy[1] + n_dy]
        draw_obj.line(
            (*convert_xy_to_pillow(agent_pos, grid_h, cell_size, scale), *convert_xy_to_pillow(neighbor_pos, grid_h, cell_size, scale)),
            fill=(200, 200, 200),
        )


def create_frame(
    im: Image.Image,
    scale: int,
    grid_h,
    cell_size,
    step: int,
    quality: int,
    goals: npt.NDArray,
    paths: npt.NDArray,
    ag_params,
    goal_assignment: npt.NDArray,
    agent_colors: List[str],
    neighbors: List[List[npt.NDArray]],
) -> List[Image.Image]:

    frames = []
    for n in range(quality):
        im_copy = im.copy()
        draw_obj = ImageDraw.Draw(im_copy)
        agents_num = len(paths[0])
        for a_id in range(agents_num):
            goal = goals[a_id]
            draw_goal(draw_obj, goal, grid_h, cell_size, scale)

        for a_id in range(agents_num):
            path = paths[:, a_id, :]
            if goal_assignment is not None:
                goal_assignment_agent = goal_assignment[:, a_id, :]
            else:
                None
            agent_color = agent_colors[a_id % len(agent_colors)]
            draw_dyn_object(
                draw_obj,
                path,
                ag_params[a_id],
                step,
                n,
                quality,
                scale,
                cell_size,
                grid_h,
                agent_color,
                goal_assignment_agent,
                neighbors[a_id] if neighbors is not None else None,
                number=a_id
            )
        frames.append(im_copy)
    return frames


def save_animation(images: List[Image.Image], output_filename: str):
    videodims = images[0].size
    fourcc = cv2.VideoWriter_fourcc(*"avc1")
    video = cv2.VideoWriter(output_filename, fourcc, 30, videodims)
    for im in images:
        video.write(cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR))
    video.release()


def draw(
    grid_map: npt.NDArray,
    grid_map_inf: npt.NDArray,
    cell_size,
    obstacles,
    goals: npt.NDArray,
    paths: npt.NDArray,
    sizes,
    goal_assignment: npt.NDArray,
    neighbors: List[List[Set[int]]],
    scale: int,
    output_filename: str = "animated_trajectories",
):

    quality = 1
    grid_h, grid_w = grid_map.shape
    agents_num = len(paths[0])

    agent_colors = [
        "#1f77b4c8",
        "#ff7f0ec8",
        "#2ca02cc8",
        "#d62728c8",
        "#9467bdc8",
        "#8c564bc8",
        "#e377c2c8",
        "#7f7f7fc8",
        "#bcbd22c8",
        "#17becfc8",
    ]

    max_time = max((len(paths[:, a_id, :]) for a_id in range(agents_num)), default=1)
    neighbors_poses = compute_neighbors_poses(paths, neighbors)
    images = []
    im = Image.new("RGBA", (int(grid_w * scale * cell_size), int(grid_h * scale * cell_size)), color=(255, 255, 255))
    draw_orig = ImageDraw.Draw(im)
    draw_map(draw_orig, grid_map, cell_size, obstacles, scale, grid_map_inf)
    for step in range(max_time):
        images.extend(
            create_frame(
                im,
                scale,
                grid_h,
                cell_size,
                step,
                quality,
                goals,
                paths,
                sizes,
                goal_assignment,
                agent_colors,
                neighbors_poses[step] if step < len(neighbors_poses) else None,
            )
        )
    save_animation(images, output_filename)


def compute_neighbors_poses(
    step_log: npt.NDArray, neighbors_ids: List[List[Set[int]]]
) -> List[List[Tuple[npt.NDArray, npt.NDArray]]]:
    """
    Computes the positions of neighbors for each agent at each time step.

    Parameters
    ----------
    step_log : np.ndarray
        The log of agent positions for each time step.
    neighbors_ids : List[List[Set[int]]]
        List of sets of neighbor indices for each agent at each time step.

    Returns
    -------
    List[List[Tuple[np.ndarray, np.ndarray]]]
        A list of lists containing tuples of current and next positions for each neighbor.
    """
    neighbors_poses = []
    for step in range(len(step_log) - 1):
        neighbors_ids_step = neighbors_ids[step]
        neighbors_poses_step = []
        for a_id in range(len(step_log[step])):
            neighbors_poses_agent = []
            for n_id in neighbors_ids_step[a_id]:
                neighbors_poses_agent.append(
                    (step_log[step][n_id], step_log[step + 1][n_id])
                )
            neighbors_poses_step.append(neighbors_poses_agent)
        neighbors_poses.append(neighbors_poses_step)
    return neighbors_poses


def draw_paths(grid_map, grid_map_inf, cell_size, obstacles, planner, scale, goals, goal_id):
    grid_h, grid_w = grid_map.shape
    im = Image.new("RGBA", (int(grid_w * scale * cell_size), int(grid_h * scale * cell_size)), color=(255, 255, 255))
    draw_orig = ImageDraw.Draw(im)
    draw_map(draw_orig, grid_map, cell_size, obstacles, scale, grid_map_inf)



    if planner is not None:

        for i in range(grid_h):
            for j in range(grid_w):
                if grid_map[i, j]:
                    continue

                pos_xy = convert_ij_to_xy((i, j), grid_h, cell_size)

                next = planner.find_next(pos_xy, goals[goal_id])
                
                color =(100, 100, 100)
                
                if next[0] == -1:
                    color = (100, 0, 0)
                    draw_object(
                            draw_orig,
                            pos_xy,
                            0.05,
                            grid_h,
                            cell_size,
                            scale,
                            color,
                            "circle",
                            number=None
                        )
                else:  
                    pos = convert_ij_to_pillow((i, j), cell_size, scale)
                    next_pos = convert_xy_to_pillow(next, grid_h, cell_size, scale)

                    draw_orig.line(
                        (
                            *pos,
                            *next_pos
                        ),
                        fill=color,
                    )

    draw_goal(draw_orig, goals[goal_id], grid_h, cell_size, scale)
    return im
