# ========== safebench/gym_carla/envs/phys_safe.py ==========
import numpy as np

# 这些参数也可以写进 yaml 再 import，这里先硬编码
A_LON_MIN_AV   = 4.0     # m/s^2
A_LON_MIN_NPC  = 4.0
A_LON_MAX_AV   = 3.0
A_LON_MAX_NPC  = 3.0
A_LAT_MIN_AV   = 1.5
A_LAT_MIN_NPC  = 1.5


def extreme_safe_longitudinal(v_av, v_npc, same_lane=True):
    if same_lane and v_av <= v_npc:          # 追尾
        d_safe = v_av**2 /(2*A_LON_MIN_AV) - v_npc**2 /(2*A_LON_MAX_NPC)
    elif same_lane and v_av > v_npc:         # cut-in / 前车急刹
        d_safe = v_npc**2 /(2*A_LON_MAX_NPC) + v_av**2 /(2*A_LON_MIN_AV) \
               - v_npc**2 /(2*A_LON_MAX_NPC)
    else:                                    # 对向
        d_safe = v_av**2 /(2*A_LON_MIN_AV) + v_npc**2 /(2*A_LON_MIN_NPC)
    return max(d_safe, 0.0)


def extreme_safe_lateral(dv_lat):
    return dv_lat**2 / (2*(A_LAT_MIN_AV + A_LAT_MIN_NPC))
