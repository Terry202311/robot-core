from robot_test.motion_test import main
import sys


def run():
    sys.argv += [
        '--ros-args',
        '-p', 'vx:=0.0',
        '-p', 'vy:=0.08',
        '-p', 'wz:=0.0',
        '-p', 'duration:=2.0',
    ]
    main()


if __name__ == '__main__':
    run()
