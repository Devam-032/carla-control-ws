"""
CARLA recorder helper (RPC only: never calls get_world, so no world stream is opened).

    python3 rec.py start <file.log>
    python3 rec.py stop
    python3 rec.py info <file.log>
"""
import sys

import carla

c = carla.Client('localhost', 2000)
c.set_timeout(20)
cmd = sys.argv[1]
if cmd == 'start':
    print(c.start_recorder(sys.argv[2], True))
elif cmd == 'stop':
    c.stop_recorder()
    print('stopped')
elif cmd == 'info':
    print(c.show_recorder_file_info(sys.argv[2], False))
