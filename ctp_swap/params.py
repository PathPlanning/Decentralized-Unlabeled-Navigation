from manavlib.common.params import ExperimentParams, BaseDiscreteAgentParams, BaseAlgParams


class ORCAAlgParams(BaseAlgParams):
    alg_name = "orca_alg"

    def __init__(self):
        super().__init__()
        self.timestep: float = 0.1
        self.sight_radius: float = 0.1
        self.agents_max_num: int = 0
        self.time_boundary: float = 0.1
        self.time_boundary_obst: float = 0.1
        self.r_eps: float = 0.0
        self.max_speed: float = 0.0

class ThetaStarParams(BaseAlgParams):
    alg_name = "thetastar_alg"

    def __init__(self):
        super().__init__()
        
        
class DirectPlannerParams(BaseAlgParams):
    alg_name = "direct_plan_alg"

    def __init__(self):
        super().__init__()
        
class DirectFollowerParams(BaseAlgParams):
    alg_name = "direct_follow_alg"

    def __init__(self):
        super().__init__()

class SimpleNavParams(BaseAlgParams):
    alg_name = "simple_nav_alg"

    def __init__(self):
        super().__init__()

class CTPSWAPParams(BaseAlgParams):
    alg_name = "ctp_swap_alg"

    def __init__(self):
        super().__init__()
        self.amapf: bool = False
        



