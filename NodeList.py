import numpy as np
from scipy.optimize import fminbound
import control
from control.matlab import lsim


class MapConfig:
    Map: np.ndarray = None
    State_Res: [float] = []
    State_UB: [float] = []
    State_LB: [float] = []
    Ctrl_UB: [float] = []
    Ctrl_LB: [float] = []
    Ctrl_Res: [float] = []
    Ctrl_Map: [np.ndarray] = None
    dT: float = 0.5

    def __init__(self, State_LB, State_UB, State_Res: float, Ctrl_LB, Ctrl_UB, Ctrl_Res: float, dT: float, Obs):
        """
        对控制量进行离散
        """
        self.State_LB: [float] = State_LB
        self.State_UB: [float] = State_UB
        self.State_Res: float = State_Res
        self.Ctrl_LB: [float] = Ctrl_LB
        self.Ctrl_UB: [float] = Ctrl_UB
        self.Ctrl_Res: float = Ctrl_Res
        self.dT: float = dT

        self.Map = np.zeros([int((x - y) // self.State_Res) for x, y in zip(self.State_UB[0:3], self.State_LB[0:3])])


class ComputeNode:
    parent_node = None
    parent_ctrl: [float] = []
    state: [float] = []
    f: float = 0.0
    g: float = 0.0
    h: float = 0.0
    t: float = 0.0
    stateInd: [int] = []

    def __init__(self, state: [float], parent_node, parent_ctrl: [float], h: float, cfg: MapConfig):
        self.state = state
        self.parent_ctrl = parent_ctrl
        self.h = h
        self.stateInd = [int(x // y) for x, y in zip(self.state, cfg.State_Res)]

        # 用父节点状态递推
        if parent_node:
            self.parent_node = parent_node
            self.g = parent_node.g + cfg.dT * (1 + sum([x ** 2 for x in parent_ctrl]))
            self.t = parent_node.t + cfg.dT
        self.f = self.g + h

    def get_parent(self):
        return self.parent_node if self.parent_node else False


class NodeList():
    L: [ComputeNode] = []

    def push(self, new_node: ComputeNode, unique_flag=True):
        # 节点查重
        for ind, node in enumerate(self.L):
            if node.stateInd == new_node.stateInd:  # 如果状态编号相同
                # 比较f值
                if new_node.f < node.f:  # 新的f值更小则取代
                    self.L[ind] = new_node
            else:
                self.L.append(new_node)
            pass
        self.L.sort(key=lambda d: d.f, reverse=True)  # 按f值降序排序

    def pop(self, pop_ind=-1) -> ComputeNode:
        """
        默认弹出最小
        :param pop_ind:默认为-1（队尾）
        :return: 指定的Node
        """
        return self.L.pop(pop_ind)

    def contain(self, new_node: ComputeNode) -> (bool, ComputeNode):
        for cnt, node in enumerate(self.L):
            if new_node.state == node.state:
                return True, node
        return False, None  # 未找到则返回空


def get_heuristic(state_now, state_goal) -> float:
    J = lambda t: cost(state_now, state_goal, t)
    t_opt = fminbound(J, 0, 10)
    Jmin = J(t_opt)
    return Jmin


def cost(state_start: np.ndarray, state_goal: np.ndarray, T: float) -> float:
    '''
    :param state_start: 初始状态
    :param state_goal: 目标状态
    :param T: 给定的时间
    :return: 代价函数值J
    '''
    # constant = np.zeros([6, 1])
    ds = state_goal - state_start - np.r_[state_start[3:], np.zeros((3,))]
    A = np.c_[np.r_[-12 * np.eye(3) / (T ** 3), 6 * np.eye(3) / (T ** 2)],
              np.r_[6 * np.eye(3) / (T ** 2), -2 * np.eye(3) / T]]
    constant = np.matmul(A, ds)
    a1, a2, a3, b1, b2, b3 = list(constant)
    return T + (1 / 3 * a1 * a1 * T * T * T + a1 * b1 * T * T + b1 * b1 * T) + (
            1 / 3 * a2 * a2 * T * T * T + a2 * b2 * T * T + b2 * b2 * T) + (
                   1 / 3 * a3 * a3 * T * T * T + a3 * b3 * T * T + b3 * b3 * T)


def hybrid_a_star(state_start, state_goal, cfg: MapConfig):
    Open = NodeList()
    Close = NodeList()
    isInStateBound = lambda state: [True, True, True, True, True, True] == [l < x < u for l, x, u in
                                                                            zip(cfg.State_LB, state, cfg.State_UB)]
    Arrived = lambda pos: sum([(x - y) ** 2 for x, y in zip(pos, state_goal[0:3])]) < 1

    A = np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1],
                  [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
    B = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0],
                  [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1],
                  [0, 0, 0], [0, 0, 0], [0, 0, 0]]).T
    D = np.zeros([3, 3])
    ss_ = control.StateSpace(A, B, C, D)

    arrived_flg = False
    wk_node = ComputeNode(state_start, None, None,
                          get_heuristic(state_start, state_goal), cfg)
    Open.push(wk_node)
    while Open.L:
        wk_node = Open.pop()
        Close.push(wk_node, unique_flag=False)
        if Arrived(wk_node):
            arrived_flg = True
            break
        for ctr_ in cfg.Ctrl_Map:
            ctr_: np.ndarray
            y_out, t, x_out = lsim(ss_, [wk_node.parent_ctrl, ctr_], [0.0, cfg.dT], wk_node.state)
            h = get_heuristic(x_out, state_goal)
            child_node = ComputeNode(x_out[-1], wk_node, ctr_, h, cfg)
            Open.push(child_node)
        pass
    state_history = [wk_node.state]
    ctrl_history = [wk_node.parent_ctrl]
    t_history = [wk_node.t]
    if arrived_flg:
        while wk_node:
            wk_node: ComputeNode = wk_node.parent_node
            state_history.append(wk_node.state)
            ctrl_history.append(wk_node.parent_ctrl)
            t_history.append(wk_node.t)
    return state_history, ctrl_history, t_history


if __name__ == '__main__':
    cfg = MapConfig(State_LB=[0, 0, 0, -2, -2, -2], State_UB=[4, 4, 4, 2, 2, 2], State_Res=0.1,
                    Ctrl_LB=[-2, -2, -2], Ctrl_UB=[2, 2, 2], Ctrl_Res=0.5, dT=1.0, Obs=[])
    state_start = np.array([0, 0, 0, 0, 0, 0])
    state_goal = np.array([3, 3, 3, 0, 0, 0])
    S, C, T = hybrid_a_star(state_start, state_goal, cfg)
