import numpy as np
import matplotlib.pyplot as plt

# =========================
# Parameters
# =========================
R = 5.0                  # Radius of curvature [m]
total_length = 20.0
ds = 0.08                # Waypoint spacing [m]

# =========================
# Derived quantities
# =========================
turn_angle = np.pi / 2
L_turn = R * turn_angle
L_straight = (total_length - L_turn) / 2

# =========================
# Segment 1: Straight (X+)
# =========================
s1 = np.arange(0.0, L_straight, ds)
x1 = s1
y1 = np.zeros_like(s1)
yaw1 = np.zeros_like(s1)

# =========================
# Segment 2: Right turn (ROC = 5 m)
# =========================
theta = np.arange(0.0, turn_angle, ds / R)

xc = L_straight
yc = -R

x2 = xc + R * np.sin(theta)
y2 = yc + R * np.cos(theta)
yaw2 = -theta

# =========================
# Segment 3: Straight (Y-)
# =========================
s3 = np.arange(0.0, L_straight, ds)

x_end = x2[-1]
y_end = y2[-1]

x3 = np.full_like(s3, x_end)
y3 = y_end - s3
yaw3 = np.full_like(s3, -np.pi / 2)

# =========================
# Concatenate full trajectory
# =========================
x = np.concatenate([x1, x2, x3])
y = np.concatenate([y1, y2, y3])
yaw = np.concatenate([yaw1, yaw2, yaw3])

trajectory = np.vstack((x, y, yaw)).T

print(f"Total waypoints: {trajectory.shape[0]}")
print(f"Approx path length: {trajectory.shape[0] * ds:.2f} m")

# =========================
# Plot
# =========================
plt.figure(figsize=(10, 6))
plt.plot(x, y, 'k-', linewidth=2, label='Reference Trajectory')

# Start & End markers
plt.plot(x[0], y[0], 'go', markersize=8, label='Start')
plt.plot(x[-1], y[-1], 'ro', markersize=8, label='End')

# Heading arrows (subsampled)
step = int(1.0 / ds)  # ~1 m spacing
plt.quiver(
    x[::step],
    y[::step],
    np.cos(yaw[::step]),
    np.sin(yaw[::step]),
    scale=20,
    width=0.004,
    color='blue',
    label='Heading'
)

plt.axis('equal')
plt.grid(True)
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Reference Trajectory (Straight + ROC 5 m Turn + Straight)')
plt.legend()
plt.tight_layout()
plt.show()