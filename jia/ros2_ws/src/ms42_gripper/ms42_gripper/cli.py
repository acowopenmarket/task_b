"""Inspect protocol frames without installing ROS or attaching hardware."""

import argparse
from .protocol import move_frame, query_frame


def main(args=None):
    parser = argparse.ArgumentParser(description='MS42DC frame generator; never opens a port')
    parser.add_argument('command', choices=['open', 'close', 'query'])
    parser.add_argument('--id', type=int, default=1)
    parser.add_argument('--angle', type=float, default=1872.0, help='Relative motor degrees')
    parser.add_argument('--speed', type=float, default=20.0, help='Motor rad/s')
    parser.add_argument('--subdivision', type=int, default=32)
    options = parser.parse_args(args)
    try:
        frame = (query_frame(options.id) if options.command == 'query' else
                 move_frame(options.id, options.command == 'close',
                            options.angle, options.speed, options.subdivision))
    except ValueError as exc:
        parser.error(str(exc))
    print(frame.hex(' ').upper())


if __name__ == '__main__':
    main()
