# coding:utf-8
import rospy, rosbag
import os
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import pcl, logging
import argparse
import glob
import load_extrinsic_lidar

def Args_B2P():
    parse = argparse.ArgumentParser(
        usage="python latest.py  -p0 ./pandarp0path -p2 ./pandarp2path -cbs ./miivibag_path sp ./save_path -y ./yaml_path"
    )
    parse.add_argument("-p0", "--pandar0_bag_path", type=str, required=True)
    parse.add_argument("-p2", "--pandar2_bag_path", type=str, required=True)
    parse.add_argument("-cbs", "--camera_bag_path", type=str, required=True)
    parse.add_argument("-sp", "--save_path", type=str, required=True)
    parse.add_argument("-y", "--yaml_path", type=str, required=True)
    args = parse.parse_args()
    return args

def savePC(topic, msg, time, save_path): 
    path=save_path 

    if not os.path.exists(path):
        os.makedirs(path)

    path = path + "/" + "time_" + str(time) + ".pcd"
    pc_list = msg.tolist()
    pcd = pcl.PointCloud_PointXYZI(pc_list)

    pcl.save(pcd, path, format = None, binary = True) # 保存的是binary编码格式


def saveImage(topic, msg, time, save_path):
    path = save_path

    if not os.path.exists(path):
        os.makedirs(path)

    path = save_path + "/" + "time_" + str(time) + ".jpg"
    img_data = np.fromstring(msg.data, np.uint8)
    cv_image = cv2.imdecode(img_data, cv2.IMREAD_COLOR)

    cv2.imwrite(path, cv_image)


def getPC(topic, msg, pc_arr, times_arr, target_topic):
    if topic == str(target_topic):
        msg_time = msg.header.stamp.to_nsec()
        print("msg_time", msg_time)

        times_arr.append(msg_time)
        pc = pc2.read_points(
            msg, skip_nans=True, field_names=("x", "y", "z", "intensity")
        )
        pc_list = []
        for p in pc:
            pc_list.append([p[0], p[1], p[2], p[3]])
        # p = pcl.PointCloud_PointXYZI(pc_list)
        pc = np.array(pc_list)
        pc_arr.append(pc)


def getImage(topic, msg, image_arr, times_arr, target_topic):
    if topic == str(target_topic):
        msg_time = msg.header.stamp.to_nsec()

        times_arr.append(msg_time)
        image_arr.append(msg)


##  dainyunjiuzheng  ##
def alignPc(R, T, pc_arr):
    pc_aligned_arr = []
    for i in range(len(pc_arr)):
        pc = pc_arr[i]
        ppp = pc_arr[i][:, 0:3]
        new_pc = np.dot(R, np.transpose(ppp))
        # T_repeat = (np.expand_dims(T, 0)).repeat(pc.shape[0], 0)
        new_pc = new_pc.transpose() + T
        new_pc_aligned = np.zeros((new_pc.shape[0], new_pc.shape[1] + 1))
        # print ("new_pc_total shape is ", new_pc_total.shape)
        new_pc_aligned[:, 0:3] = new_pc
        new_pc_aligned[:, 3] = pc[:, 3]
        pc_aligned_arr.append(new_pc_aligned)
    return pc_aligned_arr


def concatenatePC(pc_pandar_0_arr, pc_pandar_2_arr,  extrinsic_lidar):
    pc_pandar_0_aligned_arr = alignPc(
        extrinsic_lidar.R_pandar_0, extrinsic_lidar.t_pandar_0, pc_pandar_0_arr
    )
    pc_pandar_2_aligned_arr = alignPc(
        extrinsic_lidar.R_pandar_2, extrinsic_lidar.t_pandar_2, pc_pandar_2_arr
    )

    print("pc_pandar_0_aligned_arr", len(pc_pandar_0_aligned_arr))
    print("pc_pandar_2_aligned_arr", len(pc_pandar_2_aligned_arr))

    pc_all_arr_idx = []

    pc_all_arr_unchongying = []

    min_len = min(len(pc_pandar_0_aligned_arr), len(pc_pandar_2_aligned_arr))
    print("two lidar's min len ", min_len)
    pc_pandar_0_aligned_arr = pc_pandar_0_aligned_arr[:min_len]
    pc_pandar_2_aligned_arr = pc_pandar_2_aligned_arr[:min_len]
    for i in range(len(pc_pandar_0_aligned_arr)):
        pc0 = pc_pandar_0_aligned_arr[i]
        pc2 = pc_pandar_2_aligned_arr[i]

        pc_all = np.concatenate((pc0, pc2), 0)
        pc_all_arr_unchongying.append(pc_all)
        pc_all_arr_idx.append(i)
        # print("I am not chonying ", i)
    print(len(pc_pandar_0_aligned_arr))

    return pc_pandar_0_aligned_arr, pc_pandar_2_aligned_arr, pc_all_arr_unchongying, pc_all_arr_idx


def Compare_Time_Stamp(lidar_stamp, camera_stamp, image_arr):
    img_arr=[]
    time_image_arr=[]
    time_save_arr=[]
    lidar_time_new_arr=[]
    new_lidar_id_arr=[]
    img_ind = 0
    for idx, time in enumerate(lidar_stamp):
        pre_time = camera_stamp[img_ind]
        post_time = camera_stamp[img_ind]
        while post_time < time:
            pre_time = post_time
            img_ind += 1
            post_time = camera_stamp[img_ind]
        if min(abs(time - pre_time), abs(post_time - time)) > 2e8:
            continue
        if abs(time - pre_time) < abs(post_time - time):
            img_arr_i = image_arr[img_ind - 1]
            time_image = pre_time
            image_save_time_name = str(time_image) + "_" + str(time)
            time_new = time
            id_new = id

        else:
            img_arr_i = image_arr[img_ind]
            time_image = post_time
            image_save_time_name = str(time_image) + "_" + str(time)
            time_new = time
            id_new = id


        # time_img_msg = str(post_time)+"_"+str(time)
        # camera_img_msg.append(camera_img_msg)
        time_image_arr.append(time_image)
        img_arr.append(img_arr_i)
        time_save_arr.append(image_save_time_name)
        lidar_time_new_arr.append(time_new)
        new_lidar_id_arr.append(id_new)
    return img_arr, time_image_arr, time_save_arr, lidar_time_new_arr, new_lidar_id_arr


topics = [

    "/pandar_pointcloud",
    "/pandar_pointcloud",
    "/dev/video2/compressed",  #camera l60
    "/dev/video0/compressed",  #camera l120
    "/dev/video6/compressed",  #camera f120
    "/dev/video5/compressed",  #camera f60
    "/dev/video3/compressed", #camera r60
    "/dev/video1/compressed"  #camera_r120
]


if __name__ == "__main__":
    rospy.init_node("listener", anonymous=True)
    logging.basicConfig(level=logging.DEBUG)
    logging.info("trans bag")

    args = Args_B2P()

    pandar0_bag_path =args.pandar0_bag_path
    pandar2_bag_path = args.pandar2_bag_path
    
    camera_bag_path = args.camera_bag_path

    save_path = args.save_path
    yaml_path = args.yaml_path

    extrinsic_lidar = load_extrinsic_lidar.LidarExtrinsic(yaml_path)




    '''文件排序'''
    ##对文件中的bag 进行排序，保证每次度的数据是同一
    bag_name_cameras = [x for x in sorted(glob.glob(camera_bag_path+'/miivi*.bag'))]

    bag_name_p40_0s = [x for x in sorted(glob.glob(pandar0_bag_path+'/lidar_40p*.bag'))]
    bag_name_p40_2s = [x for x in sorted(glob.glob(pandar2_bag_path + '/lidar_40p*.bag'))]
    print("bag_name_p40_0", bag_name_p40_0s)
    print("bag_name_p40_2", bag_name_p40_2s)
    print("bag_name_camera", bag_name_cameras)

    for bagfile_p40_0, bagfile_p40_2, bagfile_cs in zip(bag_name_p40_0s,

                                                        bag_name_p40_2s,
                                                        bag_name_cameras ):

        pc_pandar_0_arr = []
        time_pandar_0_arr = []
        pc_pandar_2_arr = []
        time_pandar_2_arr = []
        pc_pandar_0_aligned_arr = []
        pc_pandar_2_aligned_arr = []
        pc_all_arr_unchongying = []

        image_l60_arr = []
        time_l60_arr = []

        image_l120_arr = []
        time_l120_arr = []

        image_f120_arr = []
        time_f120_arr = []
        
        image_r60_arr =[]
        time_r60_arr = []

        image_r120_arr =[]
        time_r120_arr = []

        image_f60_arr =[]
        time_f60_arr = []


        bag_name_p40_0 = bagfile_p40_0   
        bag_name_p40_2 = bagfile_p40_2
        bag_name_camera = bagfile_cs 

        print("bag_name_p40_0", bag_name_p40_0)
        print("bag_name_p40_2", bag_name_p40_2) 
        print("bag_name_camera", bagfile_cs)
  
        ### frequency  step = 1 : 10hz
        step = 5

        ####读取左侧侧激光 pandar0
        if (os.path.splitext(bag_name_p40_0)[1]==".bag"):
            bag1 = rosbag.Bag(bag_name_p40_0)
            ind = 0
            for topic, msg, t in (bag1.read_messages(topics=topics)):
                ind += 1
                if ind % step != 0:
                    continue
                getPC(topic, msg, pc_pandar_0_arr, time_pandar_0_arr, topics[0])  #getPC0

            bag1.close()
        
        ####读取右侧激光pandar2
        if (os.path.splitext(bag_name_p40_2)[1]==".bag"):
            bag2 = rosbag.Bag(bag_name_p40_2)
            ind1 = 0
            for topic, msg, t in bag2.read_messages(topics=topics):
                ind1 += 1
                if ind1 % step != 0:
                    continue
                getPC(topic, msg, pc_pandar_2_arr, time_pandar_2_arr, topics[1])  #getPC2

            bag2.close()

        ###读取 camera
        if (os.path.splitext(bag_name_camera)[1] == ".bag"):
            bag3 = rosbag.Bag(bag_name_camera)
            for topic, msg, t in bag3.read_messages(topics=topics):

                getImage(topic, msg, image_l60_arr, time_l60_arr, topics[2])  #getImage camear l60
                getImage(topic, msg, image_l120_arr, time_l120_arr, topics[3])  #getImage camear l120
                getImage(topic, msg, image_f60_arr, time_f60_arr, topics[5])  #getImage camear f60
                getImage(topic, msg, image_f120_arr, time_f120_arr, topics[4])  #getImage camear f120
                getImage(topic, msg, image_r60_arr, time_r60_arr, topics[6])  #getImage camear r60
                getImage(topic, msg, image_r120_arr, time_r120_arr, topics[7])  #getImage camear r120

            bag3.close()


        pc_pandar_0_aligned_arr, \
            pc_pandar_2_aligned_arr, \
                pc_all_arr_unchongying, \
                    pc_all_arr_idx = concatenatePC(
            pc_pandar_0_arr, pc_pandar_2_arr, extrinsic_lidar
        )
        print("len of original lidar is", len(pc_pandar_0_arr), len(pc_pandar_2_arr))
        print("lidar aligned ok")

        print("len of alighed lidar is", len(pc_pandar_0_aligned_arr), len(pc_pandar_2_aligned_arr))
        
        print("len of image l60 l120 f120 r60 r120 f60 is", len(image_l60_arr), \
            len(image_l120_arr), \
                 len(image_f120_arr), \
                     len(image_r60_arr), \
                         len(image_r120_arr), \
                              len(image_f60_arr)
                              )
        ##相机录制的长度不一致处理
        '''相机长度不一致'''
        min_len_camera = min(len(image_l60_arr), len(image_l120_arr), len(image_f120_arr), len(image_r60_arr), len(image_r120_arr), len(image_f60_arr))
        
        print('camera min length ', min_len_camera)

        image_l60_arr = image_l60_arr[:min_len_camera]
        image_l120_arr = image_l120_arr[:min_len_camera]
        image_f120_arr = image_f120_arr[:min_len_camera]
        image_r60_arr = image_r60_arr[:min_len_camera]
        image_r120_arr = image_r120_arr[:min_len_camera]
        image_f60_arr = image_f60_arr[:min_len_camera]


        time_l60_arr = time_l60_arr[:min_len_camera]
        time_l120_arr = time_l120_arr[:min_len_camera]
        time_f120_arr = time_f120_arr[:min_len_camera]
        time_r60_arr = time_r60_arr[:min_len_camera]
        time_r120_arr = time_r120_arr[:min_len_camera]
        time_f60_arr = time_f60_arr[:min_len_camera]

        '''创建存贮路径'''
        save_path_pcd_all = save_path + "/" + "pcd_all"
        if not os.path.exists(save_path_pcd_all):
            os.makedirs(save_path_pcd_all)

        # save_file_p0 = save_path + "/" + "lidar_40p_0" + "/" 
        # save_file_p2 = save_path + "/" + "lidar_40p_2" + "/"
        save_file_pc02 = save_path_pcd_all + "/" 
        save_camear_l60 = save_path + "/" + "camera_l60" + "/" 
        save_camear_l120 = save_path + "/" + "camera_l120" + "/"
        save_camear_f120 = save_path + "/" + "camera_f120" + "/" 
        save_camear_r60 = save_path + "/" + "camera_r60" + "/" 
        save_camear_r120 = save_path + "/" + "camera_r120" + "/"    
        save_camear_f60 = save_path + "/" + "camera_f60" + "/" 


        ##compare time-stamp
        for idx, time in enumerate(time_pandar_0_arr):

            # print("-----------------------------")
            # print(len(pc_pandar_0_aligned_arr))
            pc_0 = pc_pandar_0_aligned_arr[idx]
            pc_2 = pc_pandar_2_aligned_arr[idx]
            pc_all = pc_all_arr_unchongying[idx]
            # savePC(topics[0], pc_0, time, save_file_p0)
            # savePC(topics[1], pc_2, time, save_file_p2)
            savePC(topics[0], pc_all, time, save_file_pc02)
            print("save pcd completed") 


            print(idx, time)
            pandar_end = False
            img_ind = 0
            
            pre_time = time_l60_arr[img_ind]
            print("img_ind:  ", img_ind)
            post_time = time_l60_arr[img_ind]
            while post_time < time:
                pre_time = post_time 
                img_ind += 1
                if img_ind >= len(time_l60_arr):
                    pandar_end = True
                    break
                post_time = time_l60_arr[img_ind]
                print(pre_time, post_time)
                print(int(post_time)-int(pre_time))
                if pandar_end:
                    break
                if min(abs(time-pre_time), abs(post_time-time)) > 2e8:   # no matched image
                    continue

            if img_ind >= len(time_l60_arr):
                pandar_end = True
                break


            if abs(time-pre_time) < abs(post_time-time):  #find matched image
                img_msg = image_l60_arr[img_ind-1]
                image_time = str(pre_time) + "_" + str(time)
                saveImage(topics[2], img_msg, image_time, save_camear_l60)
            else:
                img_msg = image_l60_arr[img_ind]
                image_time = str(post_time) + "_" + str(time)
                saveImage(topics[2], img_msg, image_time, save_camear_l60)

            print(img_ind)
            print(len(image_l120_arr))
            
            
            ##l120
            img_ind = 0
            pre_time = time_l120_arr[img_ind]
            post_time = time_l120_arr[img_ind]
            while post_time < time:
                pre_time = post_time
                img_ind += 1
                if img_ind >= len(time_l120_arr):
                    pandar_end = True
                    break
                post_time = time_l120_arr[img_ind]
                if min(abs(time-pre_time), abs(post_time-time)) > 2e8:   # no matched image
                    continue

            if img_ind >= len(time_l120_arr):
                pandar_end = True
                break

            if abs(time-pre_time) < abs(post_time-time):  #find matched image
                img_msg = image_l120_arr[img_ind-1]
                image_time = str(pre_time) + "_" + str(time)
                saveImage(topics[3], img_msg, image_time, save_camear_l120)
            else:
                print("img_ind:    ", img_ind)
                img_msg = image_l120_arr[img_ind]
                image_time = str(post_time) + "_" + str(time)
                saveImage(topics[3], img_msg, image_time, save_camear_l120)


            ###f120
            img_ind = 0

            pre_time = time_f120_arr[img_ind]
            post_time = time_f120_arr[img_ind]
            while post_time < time:
                pre_time = post_time
                img_ind += 1
                if img_ind >= len(time_f120_arr):
                    pandar_end = True
                    break
                post_time = time_f120_arr[img_ind]
                if min(abs(time - pre_time), abs(post_time - time)) > 2e8:  # no matched image
                    continue

            if img_ind >= len(time_f120_arr):
                pandar_end = True
                break

            if abs(time - pre_time) < abs(post_time - time):  # find matched image
                img_msg = image_f120_arr[img_ind - 1]
                image_time = str(pre_time) + "_" + str(time)
                saveImage(topics[4], img_msg, image_time, save_camear_f120)
            else:
                img_msg = image_f120_arr[img_ind]
                image_time = str(post_time) + "_" + str(time)
                saveImage(topics[4], img_msg, image_time, save_camear_f120)

            ###f60
            img_ind = 0

            pre_time = time_f60_arr[img_ind]
            post_time = time_f60_arr[img_ind]
            while post_time < time:
                pre_time = post_time
                img_ind += 1
                if img_ind >= len(time_f60_arr):
                    pandar_end = True
                    break
                post_time = time_f60_arr[img_ind]
                if min(abs(time - pre_time), abs(post_time - time)) > 2e8:  # no matched image
                    continue
            if img_ind >= len(time_f60_arr):
                pandar_end = True
                break

            if abs(time - pre_time) < abs(post_time - time):  # find matched image
                img_msg = image_f60_arr[img_ind - 1]
                image_time = str(pre_time) + "_" + str(time)
                saveImage(topics[5], img_msg, image_time, save_camear_f60)
            else:
                img_msg = image_f60_arr[img_ind]
                image_time = str(post_time) + "_" + str(time)
                saveImage(topics[5], img_msg, image_time, save_camear_f60)


            ##r60
            img_ind = 0

            pre_time = time_r60_arr[img_ind]
            post_time = time_r60_arr[img_ind]
            while post_time < time:
                pre_time = post_time
                img_ind += 1
                if img_ind >= len(time_r60_arr):
                    pandar_end = True
                    break
                post_time = time_r60_arr[img_ind]
                if min(abs(time - pre_time), abs(post_time - time)) > 2e8:  # no matched image
                    continue

            if img_ind >= len(time_r60_arr):
                pandar_end = True
                break

            if abs(time - pre_time) < abs(post_time - time):  # find matched image
                img_msg = image_r60_arr[img_ind - 1]
                image_time = str(pre_time) + "_" + str(time)
                saveImage(topics[6], img_msg, image_time, save_camear_r60)
            else:
                img_msg = image_r60_arr[img_ind]
                image_time = str(post_time) + "_" + str(time)
                saveImage(topics[6], img_msg, image_time, save_camear_r60)

            ##r120
            img_ind = 0

            pre_time = time_r120_arr[img_ind]
            post_time = time_r120_arr[img_ind]
            while post_time < time:
                pre_time = post_time
                img_ind += 1
                if img_ind >= len(time_r120_arr):
                    pandar_end = True
                    break
                post_time = time_r120_arr[img_ind]
                if min(abs(time - pre_time), abs(post_time - time)) > 2e8:  # no matched image
                    continue
            
            if img_ind >= len(time_r120_arr):
                pandar_end = True
                break

            if abs(time - pre_time) < abs(post_time - time):
                img_msg = image_r120_arr[img_ind - 1]
                image_time = str(pre_time) + "_" + str(time)
                saveImage(topics[7], img_msg, image_time, save_camear_r120)
            else:
                print("7777777777777")  # find matched image

                img_msg = image_r120_arr[img_ind]
                image_time = str(post_time) + "_" + str(time)
                saveImage(topics[7], img_msg, image_time, save_camear_r120)

        print("save image ok!")
