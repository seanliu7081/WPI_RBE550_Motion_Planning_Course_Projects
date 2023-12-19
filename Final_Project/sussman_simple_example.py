import pybullet as p
import time
import pybullet_data


p.connect(p.GUI)
p.setGravity(0, 0, -10)


p.setAdditionalSearchPath(pybullet_data.getDataPath())


planeId = p.loadURDF("plane.urdf")

# Function to create a block
def create_block(position, color):
    block_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
    block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=color)
    block_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=block_shape, baseVisualShapeIndex=block_visual, basePosition=position)
    return block_id


block_a = create_block([0, 0, 0.1], [1, 0, 0, 1])  # Red block (A)
block_b = create_block([0.1, 0, 0.05], [0, 1, 0, 1]) # Green block (B)
block_c = create_block([0, 0, 0.2], [0, 0, 1, 1]) # Blue block (C)

# Function to move a block step by step
def move_block(block_id, target_position, lift_height=0.3, steps=300):
    start_position, _ = p.getBasePositionAndOrientation(block_id)
 
    for step in range(1, steps + 1):
        interp_position = [
            start_position[0],
            start_position[1],
            start_position[2] + (lift_height - start_position[2]) * step / steps,
        ]
        p.resetBasePositionAndOrientation(block_id, interp_position, [0, 0, 0, 1])
        p.stepSimulation()
        time.sleep(1/240)


    for step in range(1, steps + 1):
        interp_position = [
            start_position[0] + (target_position[0] - start_position[0]) * step / steps,
            start_position[1] + (target_position[1] - start_position[1]) * step / steps,
            lift_height,
        ]
        p.resetBasePositionAndOrientation(block_id, interp_position, [0, 0, 0, 1])
        p.stepSimulation()
        time.sleep(1/240)


    for step in range(1, steps + 1):
        interp_position = [
            target_position[0],
            target_position[1],
            lift_height - (lift_height - target_position[2]) * step / steps,
        ]
        p.resetBasePositionAndOrientation(block_id, interp_position, [0, 0, 0, 1])
        p.stepSimulation()
        time.sleep(1/240)


for _ in range(100):
    p.stepSimulation()
    time.sleep(1/240)

# Sequence of block movements
move_block(block_c, [0.2, 0, 0.05]) 
move_block(block_b, [0.2, 0, 0.15])
move_block(block_a, [0.2, 0, 0.25])


for _ in range(500):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()
