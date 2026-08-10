import struct


FRAME_LEN = 11


def check_frame(frame):

    if len(frame) != FRAME_LEN:
        return False


    if frame[0] != 0x55:
        return False


    checksum = sum(frame[:10]) & 0xff


    return checksum == frame[10]



def parse_acc(frame):

    """
    55 51

    ax ay az temp
    """

    ax, ay, az, temp = struct.unpack(
        "<hhhh",
        frame[2:10]
    )


    return ax, ay, az



def parse_gyro(frame):

    """
    55 52

    gx gy gz temp
    """

    gx, gy, gz, temp = struct.unpack(
        "<hhhh",
        frame[2:10]
    )


    return gx, gy, gz