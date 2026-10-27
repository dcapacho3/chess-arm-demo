import numpy as np

from chess_arm_demo import layout as L
from chess_arm_demo.moveit_arm import tool_pose


def test_tool_points_down_and_fingers_close_along_world_y():
    T = tool_pose(0.5, 0.1, L.GRASP_Z)
    assert np.allclose(T[:3, 2], [0, 0, -1])            # tool axis, the fingers' direction, is straight down
    assert np.allclose(np.abs(T[:3, 1]), [0, 1, 0])     # fingers close along the world y axis


def test_flange_sits_one_tcp_offset_above_the_pad():
    T = tool_pose(0.5, 0.1, L.GRASP_Z)
    assert np.allclose(T[:3, 3], [0.5, 0.1, L.GRASP_Z + L.TCP_OFFSET])
