#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import cv2
from ultralytics import YOLO
import numpy as np




model1=YOLO('best.pt')

cap = cv2.VideoCapture(0)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

v_line_x_pos=int(width/4)
# v_line_x_pos=int(width/2)

# Generate 25 equally spaced floating-point values between 0 and 800
floating_values = np.linspace(0, height, 25)

# Convert the floating-point values to integers
y_values = floating_values.astype(int)

frame_counter = 0


if __name__ == '__main__' :
    rospy.init_node('lane_detect_node')
    rospy.loginfo("Lane detection start")

    pub_displacement = rospy.Publisher("/lane_detect/line_displace", Int32MultiArray, queue_size=500)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg = Int32MultiArray()
        # msg.data = [0,0,0,0,1,1,1,2,2,3]
        # pub_displacement.publish(msg)


        if cap.isOpened():
            # Read a frame from the video
            success, frame = cap.read()
            # height,width=frame.shape[:2]
            frame=frame[:height,:int(width/2)]
            # Draw the vertical line on the image
            

            if success:
                frame_counter += 1
                # Skip frames if not the 9th frame
                if frame_counter < 6:
                    continue

                # Reset the frame counter
                frame_counter = 0

   
                results = model1.predict(frame, conf=0.1,max_det=1)
                
                try:
                    # Run YOLOv8 inference on the frame
                    annotated_frame = results[0].plot()

                    cv2.line(annotated_frame, (v_line_x_pos, 0), (v_line_x_pos, height), (255,0,0), 2)

                    if(results[0].masks is not None):
                        # Convert mask to single channel image
                        mask_raw = results[0].masks[0].cpu().data.numpy().transpose(1, 2, 0)
                        
                        # Convert single channel grayscale to 3 channel image
                        mask_3channel = cv2.merge((mask_raw,mask_raw,mask_raw))

                        # Get the size of the original image (height, width, channels)
                        h2, w2, c2 = results[0].orig_img.shape
                        
                        # Resize the mask to the same size as the image (can probably be removed if image is the same size as the model)
                        mask = cv2.resize(mask_3channel, (w2, h2))

                        # Convert BGR to HSV
                        hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)

                        # Define range of brightness in HSV
                        lower_black = np.array([0,0,0])
                        upper_black = np.array([0,0,1])


                        # Create a mask. Threshold the HSV image to get everything black
                        mask = cv2.inRange(mask, lower_black, upper_black)

                        # Invert the mask to get everything but black
                        mask = cv2.bitwise_not(mask)
                        
                        # Applying the Canny Edge filter
                        edge = cv2.Canny(mask, 50, 150)

                        indices = np.where(edge != [0])

                        # Combine the lists into a list of tuples
                        combined_list = list(zip(indices[1],indices[0]))
                        combined_list = list(set(combined_list))

                        # Initialize empty lists for coordinates on the right side and left side
                        right_side = []
                        left_side = []

                        # Iterate through the list of coordinates
                        for coord in combined_list:
                            x, y = coord
                            if y in y_values:
                                if x <= v_line_x_pos:
                                    # Append to the left side list if x <= 100
                                    left_side.append((x, y))
                                else:
                                    # Append to the right side list if x > 100
                                    right_side.append((x, y))

                        # print("len left", len(left_side), "len right", len(right_side))
                        # Initialize an empty list for midpoints
                        midpoints = []

                        # Iterate through all pairs of coordinates in left_side and right_side
                        for left_coord in left_side:
                            for right_coord in right_side:
                                # Check if the y-coordinates match
                                if left_coord[1] == right_coord[1]:
                                    # Calculate the average x-value for the same y-coordinate
                                    avg_x = int((left_coord[0] + right_coord[0]) / 2)
                                    # Append the midpoint coordinates to the midpoints list
                                    midpoints.append((avg_x, left_coord[1]))  # or right_coord[1], as they are the same


                        # # Print the list of midpoints
                        # print("Midpoints:", midpoints)
                        for a in midpoints:
                            x, y = a
                            if y!=465:
                            # OpenCV uses BGR color format, so (0, 0, 255) represents red color
                                cv2.circle(annotated_frame, (x, y), 2, (255, 255, 255), -1)  # -1 specifies to fill the circle

                        displacement_dict = {y: x - v_line_x_pos for x, y in midpoints}
                        displacement_dict = dict(sorted(displacement_dict.items(), key=lambda item: item[0], reverse=True))
                        displacement_list = list(displacement_dict.values())
                        # print("list", displacement_list)
                        
                        n= len(displacement_list)
                        msg.data=displacement_list
                        print("dis list" , displacement_list)

                        # # Define the radius of the dots
                        # radius = 5

                        # # Draw a dot at each coordinate
                        # for coord in combined_list:
                        #     x, y = coord
                        #     # OpenCV uses BGR color format, so (0, 0, 255) represents red color
                        #     cv2.circle(edge, (x, y), radius, (255, 255, 255), -1)  # -1 specifies to fill the circle


                        # Show the masked part of the image
                        # cv2.imshow("mask", edge)

                    cv2.imshow("YOLOv8 Inference2",annotated_frame)

                    # Break the loop if 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                except:
                    # cv2.imshow("YOLOv8 Inference2",frame)
                    print("except part is running")
                    cv2.imshow("YOLOv8 Inference2",frame)


                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        pub_displacement.publish(msg)

        rate.sleep()

    rospy.spin()