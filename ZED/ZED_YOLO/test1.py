import cv2
from ultralytics import YOLO
import numpy as np
import time


# model load
model = YOLO('yolov8n.pt')

# Open camera
cap = cv2.VideoCapture(0)

if cap.isOpened() == 0:
	exit(-1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2540)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


# fps counter
prev_frame_T = 0
new_frame_T = 0

while cap.isOpened():
	# Read frame
	success, frame = cap.read()

	frame_split = np.split(frame, 2, axis=1)
	frame_edit = frame_split[0]


	if success:
		results = model.predict(frame_split[0], conf=0.5)

		for r in results:
			# print("box :", r.boxes.xyxy)
			boxes = r.boxes.cuda()
			object_XY = boxes.xyxy
			
			for i in range(len(object_XY)):
				start_point = ( int(object_XY[i][0]),int(object_XY[i][1]))
				end_point = (int(object_XY[i][2]),int(object_XY[i][3]))

				print( r.names[int(r.boxes.cls[i])], ":", start_point, end_point)

				frame_edit = cv2.rectangle(frame_edit, start_point, end_point, (255,0,0), 1)


		# annotated_frame = results[0].plot()
		# cv2.putText(annotated_frame, str(fps), (7,70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100,255,0), 3, cv2.LINE_AA)


		new_frame_T = time.time()
		fps = 1/(new_frame_T - prev_frame_T)
		prev_frame_T = new_frame_T
		cv2.putText(frame_edit, str(fps), (7,70), cv2.FONT_HERSHEY_SIMPLEX, 1, (100,255,0), 3, cv2.LINE_AA)

		cv2.imshow("frame", frame_edit)

		if cv2.waitKey(1) & 0XFF == ord("q"):
			break

	else :
		break



cap.release()
cv2.destroyAllWindows()