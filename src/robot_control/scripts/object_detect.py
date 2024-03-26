#!/usr/bin/env python3

import rospy
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

shape = (376, 672, 3)
frame = np.zeros(shape)
shape = (376, 672)
depth = np.zeros(shape)

model = YOLO('yolov8n-seg.pt')
frame_success = False
depth_success = False


def image_read(msg: Image):
    global frame
    global frame_success

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = orig
    frame_success = True


def depth_read(msg: Image):
    global depth
    global depth_success

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth = orig
    depth_success = True



if __name__ == '__main__':
    rospy.init_node('object_detect_node')
    rospy.loginfo("object identify node start")

    rate = rospy.Rate(30)

    sub_image = rospy.Subscriber("/zed_node/image", Image, callback=image_read)
    sub_depth = rospy.Subscriber("/zed_node/depth", Image, callback=depth_read)
    

    while not rospy.is_shutdown():

        if ( frame_success ):
            # # convert (376, 672, 3) to (376, 640, 3)
            # remove first and last 16 columns
            frame = frame[:, +16:-16, :]
            # cv2.imshow("upgrade frame", frame)
    
            # # YOLO model load ---------------------------------------------------------------
            results = model(frame)

            # person mask data
            person_mask = []

            try:
                for r in results:
                    masks = r.masks.cuda()
                    masks_xy = masks.xy
                    boxes = r.boxes.cuda()
                    object_XY = boxes.xyxy

                    person_count_val = -1

                    for count, dot_array in enumerate(masks_xy):
                        detect_mask = []

                        start_point_x = int(object_XY[count][0])
                        start_point_y = int(object_XY[count][1])
                        end_point_x = int(object_XY[count][2])
                        end_point_y = int(object_XY[count][3])

                        start_point = ( start_point_x, start_point_y)
                        end_point = ( end_point_x, end_point_y)

                        print("name", int(r.boxes.cls[count]), r.names[int(r.boxes.cls[count])], person_count_val +1)

                        if ( int(r.boxes.cls[count]) == 0):
                            person_count_val = person_count_val +1
                            # print(masks.data.size())
                            # print(masks.data[count])

                            # # person mask bitmap data -----------------------------------------------------
                            detect_mask.append(int(r.boxes.cls[count]))
                            detect_mask.append(person_count_val +1)
                            detect_mask.append(results[0].masks.data[count].cpu().numpy())
                            detect_mask.append(start_point)
                            detect_mask.append(end_point)
                            person_mask.append(detect_mask)
                            
                print("detection count", len(person_mask))
                print("------------------------------------------------------------")
            except:
                print("model detection error")

            
            # # depth data processing ------------------------------------------------------------
            try:
                for detect_mask in person_mask:
                    frame = cv2.rectangle(frame, detect_mask[3], detect_mask[4], (255,0,0), 1)
                    frame = cv2.putText( frame, str(r.names[detect_mask[0]]), detect_mask[3], cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 1, cv2.LINE_AA)
                    
                    # # mask boundary coords (image_mask X depth_matrix) ---------------------------
                    # get detect person and remove first 8 rows to 376x640
                    object_frame = detect_mask[2][ +4:-4, :]
                    person_depth_array = np.array(object_frame) * np.array(depth[:,+16:-16])
                    print("person_depth_array", person_depth_array.shape)

                    #     # # minimum point of mask -----------------------------------------------
                    #     min_array= np.empty((0, 2), dtype=float)
                    #     person_x_min=0
                    #     person_y_min=0
                    #     for row in person_depth_array:
                    #         # print(type(row))
                    #         row[ row == 0 ] = np.nan
                    #         # print("max", np.nanmax(row) , "min", np.nanmin(row))
                    #         if np.any(~np.isnan(row)):
                    #             person_x_min = np.nanargmin(np.abs(row))
                    #             min_array = np.vstack((min_array, np.array([np.nanmin(np.abs(row)) , person_x_min])))
                    #         else:
                    #             min_array = np.vstack((min_array, np.array([np.nan , np.nan])))
                        
                    #     print(min_array.shape)
                    #     # # minimum value select
                    #     minimum_value = np.nanmin(min_array[:,0])
                    #     person_y_min = np.nanargmin(min_array[:,0])
                    #     person_x_min = min_array[person_y_min,1]
                    #     print('minimun', minimum_value, "at:", person_x_min, person_y_min)

                    #     image_ndarray = cv2.circle( image_ndarray , (int(person_x_min*2),int(person_y_min*2)), 1, (255,0,0), 5)
                    #     image_ndarray = cv2.putText( image_ndarray, str(int(minimum_value)), [int(person_x_min*2),int(person_y_min*2)], cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 2, cv2.LINE_AA)
                    
                    #     # # boundary point values -----------------------------------------------------
                    #     # # box1 = [start_point_x,start_point_y], [start_point_x + 10, end_point_y]
                    #     # # box2 = [end_point_x - 10,start_point_y] , [end_point_x, end_point_y]
                    #     S_point_min = np.nanmin(np.abs(person_depth_array[ int(start_point_y/2):int(end_point_y/2), int(start_point_x/2):int(start_point_x/2+100) ]))
                    #     E_point_min = np.nanmin(np.abs(person_depth_array[ int(start_point_y/2):int(end_point_y/2), int(end_point_x/2-100):int(end_point_x/2) ]))
                    #     print("start point mini :", S_point_min)
                    #     print("end point min :", E_point_min)

                    #     image_ndarray = cv2.putText( image_ndarray, str(S_point_min), ( start_point_x - 100, end_point_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 2, cv2.LINE_AA)
                    #     image_ndarray = cv2.putText( image_ndarray, str(E_point_min), ( end_point_x, end_point_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 2, cv2.LINE_AA)
                    
                    frame_name = "person_depth" + str(detect_mask[1])
                    cv2.imshow(frame_name, person_depth_array/5000)
                
                cv2.imshow("frame data", frame)

                print("------------------------------------------------------------")
            except:
                print("depth processing error")


        if cv2.waitKey(1) & 0XFF == ord("q"):
            break

        frame_success = False

        rate.sleep()
    

    rospy.spin()