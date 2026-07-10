from typing import Optional, Tuple


def build_cmd_vel_packet(vx: float, vy: float, wz: float) -> str:
    """Build a velocity command packet for Arduino."""
    return f"CMD,{vx:.3f},{vy:.3f},{wz:.3f}\n"


def parse_encoder_packet(
    line: str,
) -> Optional[Tuple[int, int, int, int]]:
    """
    Parse:
        ENC,lf,rf,lb,rb

    Returns:
        (lf, rf, lb, rb)

    Returns None when the packet is invalid.
    """
    parts = line.strip().split(',')

    if len(parts) != 5:
        return None

    if parts[0] != 'ENC':
        return None

    try:
        lf = int(parts[1])
        rf = int(parts[2])
        lb = int(parts[3])
        rb = int(parts[4])
    except ValueError:
        return None

    return lf, rf, lb, rb
