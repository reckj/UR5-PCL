# %% [markdown]
# UR5e exploration — research/internal track.
# Run cell-by-cell in VS Code (Python interactive) or Jupyter.
# This imports the framework; it never reimplements control logic.

# %%
from ur5e import connect

# %%
# Receive-only connection to URSim. Inspect state, move nothing.
r = connect("sim")
print("connected:", r.connected)
print("robot mode:", r.receive.getRobotMode())

# %%
# Read live state.
q = r.receive.getActualQ()
pose = r.receive.getActualTCPPose()
print("joint angles (rad):", q)
print("tcp pose [x,y,z,rx,ry,rz]:", pose)

# %%
# Always disconnect when done exploring.
r.disconnect()
print("disconnected:", not r.connected)
