import pandas as pd

joint_list = []
line_count =0
# with open('/home/ingjae/catkin_ws/src/move_arm/trajectory_csv/test.csv') as traj:

data = pd.read_csv(r"/home/ingjae/catkin_ws/src/move_arm/trajectory_csv/readypose_100hz.csv")
position = pd.DataFrame(data, columns=['.position'])
print (position)

position.to_csv(r"/home/ingjae/catkin_ws/src/move_arm/trajectory_csv/position_readypose_100hz.csv",index=False)