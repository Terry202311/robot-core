def build_cmd_vel_packet(vx: float, vy: float, wz: float) -> str:
    return f"CMD,{vx:.3f},{vy:.3f},{wz:.3f}\n"
