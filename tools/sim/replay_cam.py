"""
Render a CARLA recording to frames with a chase camera attached to the replayed car
(no screen capture, so other windows can never end up in the video).

    python3 replay_cam.py <file.log> <follow_actor_id> <out_dir>

Runs at a fixed 1/30 s step, so frame N is exactly N/30 s of sim time:
encode with `ffmpeg -framerate 30` for true-speed playback.
"""
import os
import re
import sys
import time

import carla

FPS = 30
path, follow_id, out_dir = sys.argv[1], int(sys.argv[2]), sys.argv[3]
os.makedirs(out_dir, exist_ok=True)

c = carla.Client('localhost', 2000)
c.set_timeout(30)
w = c.get_world()

info = c.show_recorder_file_info(path, False)
duration = float(re.search(r'Duration:\s*([\d.]+)', info).group(1))
n_frames = int(duration * FPS)

# Synchronous with a fixed step: we tick, so no frame is ever skipped
s = w.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = 1.0 / FPS
w.apply_settings(s)

c.replay_file(path, 0.0, 0.0, follow_id, False)
w.tick()
w.tick()

cars = [a for a in w.get_actors() if a.type_id.startswith('vehicle.')]
if not cars:
    sys.exit('no replayed vehicle found')
car = cars[0]

bp = w.get_blueprint_library().find('sensor.camera.rgb')
bp.set_attribute('image_size_x', '1280')
bp.set_attribute('image_size_y', '720')
bp.set_attribute('fov', '90')
cam = w.spawn_actor(
    bp,
    carla.Transform(carla.Location(x=-9.0, z=4.0), carla.Rotation(pitch=-12.0)),
    attach_to=car,
    attachment_type=carla.AttachmentType.Rigid,
)

saved = [0]


def on_image(img):
    img.save_to_disk(os.path.join(out_dir, f'{saved[0]:05d}.jpg'))
    saved[0] += 1


cam.listen(on_image)
try:
    for i in range(n_frames):
        w.tick()
        # keep the server at most a few frames ahead of the disk writer
        while saved[0] < i - 5:
            time.sleep(0.005)
    time.sleep(1.0)
finally:
    cam.stop()
    cam.destroy()
    c.stop_replayer(False)
    s.synchronous_mode = False
    s.fixed_delta_seconds = None
    w.apply_settings(s)
print(f'saved {saved[0]} frames ({saved[0] / FPS:.1f} s) to {out_dir}')
