import pybullet as p
import time
import pybullet_data

# Initialize PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, 0)  # Gravity is turned off

p.setAdditionalSearchPath(pybullet_data.getDataPath())


planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[-0.6, 0, 0])

# create a block
def create_block(position, color):
    block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=color)
    block_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=block_visual, basePosition=position)
    return block_id


blocks = {
    'A': create_block([0, 0, 0.05], [1, 0, 0, 1]),
    'B': create_block([0.2, 0, 0.05], [0, 1, 0, 1]),
    'C': create_block([0, 0, 0.15], [0, 0, 1, 1])
}

# wait 3 s before start
time.sleep(3)

# calculate ik
def calculate_ik(target_pos):
    end_effector_index = 6
    joint_positions = p.calculateInverseKinematics(robotId, end_effector_index, target_pos)
    return joint_positions


block_height = 0.1 
table_height = 0.05

def move_robot_and_block(block_id, target_position):
    block_pos = p.getBasePositionAndOrientation(block_id)[0]
    arm_pos = [block_pos[0], block_pos[1], block_pos[2] + 0.2]
    joint_positions = calculate_ik(arm_pos)
    for j in range(p.getNumJoints(robotId)):
        p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL, joint_positions[j])

    for _ in range(50):
        p.stepSimulation()
        time.sleep(1/240)

    # Move the block to the target position
    p.resetBasePositionAndOrientation(block_id, target_position, [0, 0, 0, 1])
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1/240)

# PDDL plan
pddl_plan = [('unstack', 'C', 'A'), ('put-down', 'C', 'Table'), ('pick-up', 'B'), 
             ('stack', 'B', 'C'), ('pick-up', 'A'), ('stack', 'A', 'B')]

# PDDL actions to PyBullet actions
for action in pddl_plan:
    block_id = blocks[action[1]]
    if action[0] in ['unstack', 'pick-up']:
        # Move the block to a temporary position
        temp_pos = [0.5, 0, table_height + block_height]  # An arbitrary position away from other blocks
        move_robot_and_block(block_id, temp_pos)
    elif action[0] in ['put-down', 'stack']:
        if action[2] == 'Table':
            target_pos = [0.3, 0, table_height]  # Position on the table
        else:
            target_block_id = blocks[action[2]]
            target_block_pos = p.getBasePositionAndOrientation(target_block_id)[0]
            # Stack the block on top of the target block
            target_pos = [target_block_pos[0], target_block_pos[1], target_block_pos[2] + block_height]
        move_robot_and_block(block_id, target_pos)


for _ in range(1000):
    p.stepSimulation()
    time.sleep(1/240)

p.disconnect()





