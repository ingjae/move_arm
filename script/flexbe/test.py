joint_list = []
line_count =0
with open('/home/ingjae/catkin_ws/src/move_arm/trajectory_csv/position_test.csv') as traj:
    while 1:
        data = traj.readline().replace("\n","").replace("(","").replace(")","").replace(" ","").strip("\"")

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
        # print("------------")
        # print(num[0])
        # print(num[5])

        print(float(num[0])+float(num[1])+float(num[2])+float(num[3])+float(num[4])+float(num[5]))