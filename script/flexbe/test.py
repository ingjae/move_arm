joint_list = []
line_count =0
with open('/home/ingjae/catkin_ws/src/move_arm/trajectory_csv/test.csv') as traj:
    while 1:
        data = traj.readline().replace("\n","")
        # print (data)
        if not data: break
        if line_count == 0 :
            header = data.split(",")
        else : 
            joint_list.append(data.split(","))
        line_count += 1
        # print(line_count) 
        # print(data) 
        # print(joint_list)
        
        # client.move_joint(j1,j2,j3,j4,j5,j6

    for num in joint_list:
        # print(num)
        print("------------")

        print(num[0],num[1],num[2],num[3],num[4],num[5])