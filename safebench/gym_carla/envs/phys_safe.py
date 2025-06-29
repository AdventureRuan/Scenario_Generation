# ========== safebench/gym_carla/envs/phys_safe.py ==========
import numpy as np

# ---------------- 物理解参数（已写入 YAML） --------------
A_LON_DEC_AV   = 4.0   # AV 最大纵向减速度 |a_lon,max,AV|   (m/s^2)
A_LON_DEC_NPC  = 4.0   # NPC 最大纵向减速度 |a_lon,max,NPC|
A_LAT_DEC_AV   = 1.5   # AV 最大横向减速度 |a_lat,max,AV|
A_LAT_DEC_NPC  = 1.5   # NPC 最大横向减速度 |a_lat,max,NPC|
EPS = 1e-6             # 防零除
INF_DIST = 1e6     # 1000 km 已远超 CARLA 地图

# ===========================================================
# 1) 对向(迎面) —— 公式 d_safe,1^{lon}
# ===========================================================
def phys_safe_opposite(v_av: float, v_npc: float) -> float:
    """
    两车对向行驶时，为保证双方都以最大减速度刹停也不会相撞，
    需要保持的最小纵向安全距离：
        d_safe,1^{lon} = V_AV^2/(2 a_AV) + V_NPC^2/(2 a_NPC)
    """
    d = (v_av**2) / (2.0 * (A_LON_DEC_AV + EPS)) \
      + (v_npc**2) / (2.0 * (A_LON_DEC_NPC + EPS))
    return max(d, 0.0)

# ===========================================================
# 2) 同向/追尾/加塞 —— 公式 d_safe,2^{lon} = max(d_stop, d_equal)
# ===========================================================
def _d_equal(v_av: float, v_npc: float) -> float:
    """
    两车同向、都极限刹车且最终速度相等时的位置差（“equal braking）：
        d_equal^{lon} = (V_AV - V_NPC)^2 / (2 (a_AV - a_NPC))
    仅当 a_AV > a_NPC && V_AV > V_NPC 时有效，否则返回 0
    """
    if A_LON_DEC_AV <= A_LON_DEC_NPC + EPS or v_av <= v_npc + EPS:
        return 0.0
    #如果他们的加速度差不多,或者速度差不多,就会炸掉.比如如果速度和加速度都差不多,极限是0,代表绝对安全,符合实际. 2.速度差不多,加速度正的,安全.  3.加速度差不多,炸了,极大值,可能绝对不安全
    return (v_av - v_npc)**2 / (2.0 * (A_LON_DEC_AV - A_LON_DEC_NPC))

def _d_stop(v_av: float, v_npc: float) -> float:
    """
    NPC 先以最大减速度刹停，随后 AV 以最大减速度刹停，
    二者停止后仍不碰撞所需距离（取绝对值保证非负）：
        d_stop^{lon} = | V_NPC^2 / (2 a_AV) - V_AV^2 / (2 a_NPC) |
    """
    return abs((v_npc**2) / (2.0 * (A_LON_DEC_AV + EPS))
             - (v_av **2) / (2.0 * (A_LON_DEC_NPC + EPS)))

def phys_safe_same_lane(v_av: float, v_npc: float) -> float:
    """
    同车道场景下的物理解安全距离，取 d_stop 与 d_equal 的更大者。
    若后车速度 ≤ 前车速度，则无需额外距离（返回 0）。
    """
    if v_av <= v_npc + EPS:
        return 0.0
    return max(_d_stop(v_av, v_npc), _d_equal(v_av, v_npc))



# =======================================================
# 横向安全距离（同向 / 相对） unified
# =======================================================



def phys_safe_lat_same(v_av: float, v_npc: float) -> float:
    """
    横向同向场景安全距离（NPC 想别 AV）。

    若 NPC 的极限横向减速(制动) ≤ AV -> 用 equal-brake 公式。
    否则（NPC 横向能力更强，AV 无法抵消），返回无限大，
    意味 *不存在* 有限安全距离 —— 强制给出极大处罚。
    """
    v_rel = v_npc - v_av
    if v_rel <= EPS:                      # NPC 正在远离，风险 0
        return 0.0

    if A_LAT_DEC_AV > A_LAT_DEC_NPC + EPS:
        # AV 刹车（或反向加速）能力更强，可用 equal-brake,加速度差不多,距离非常大,其实就是绝对不安全,
        return v_rel**2 / (2.0 * (A_LAT_DEC_AV - A_LAT_DEC_NPC))
    else:
        # NPC 更强，AV 无法防御 → 无穷大安全距
        return INF_DIST



def phys_safe_lat_opp(v_av: float, v_npc: float) -> float:
    """
    相对横移（sign 不同或一方近 0），两车各自刹停位移之和：
        d = v_AV^2/(2 a_AV) + v_NPC^2/(2 a_NPC)
    """
    return (v_av**2)  / (2.0 * (A_LAT_DEC_AV  + EPS)) + \
           (v_npc**2) / (2.0 * (A_LAT_DEC_NPC + EPS))


def set_from_cfg(c):
    global A_LON_DEC_AV, A_LON_DEC_NPC, A_LAT_DEC_AV, A_LAT_DEC_NPC, EPS, INF_DIST
    A_LON_DEC_AV  = c['a_lon_dec_av']
    A_LON_DEC_NPC = c['a_lon_dec_npc']
    A_LAT_DEC_AV  = c['a_lat_dec_av']
    A_LAT_DEC_NPC = c['a_lat_dec_npc']
    EPS           = c['eps']
    INF_DIST      = c['INF_DIST']
