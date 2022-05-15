import dlib
import cv2
import os
import numpy as np
import time

hogFaceDetector = dlib.get_frontal_face_detector()

video_capture = cv2.VideoCapture(0)

# Initialize some variables
faces = []
process_this_frame = 0

print('starting now')
total_time = 0
frame_cnt = 0
detect_times = 0
while True:
    # Grab a single frame of video
    start_time = time.time()
    ret, frame = video_capture.read()

    # Resize frame of video to 1/4 size for faster face recognition processing

    # small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    # rgb_small_frame = small_frame[:, :, ::-1]

    # Only process every other frame of video to save time
    if process_this_frame==2:
        process_this_frame = 0
        # Find all the faces and face encodings in the current frame of video
        # faces_cnn = cnn_face_detector(small_frame, 0)
        faces = hogFaceDetector(frame, 0)
        if len(faces)>0 and total_time>5: detect_times+=1
        frame_cnt += 1
    process_this_frame += 1
    total_time += (time.time() - start_time)
        # Display the results
    for face in faces:
        # draw box over face
        cv2.rectangle(frame, (face.left(), face.top()), (face.right(), face.bottom()), (0, 0, 255), 2)



    # Display the resulting image
    cv2.imshow('Video', frame)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if total_time>65:
        break
print("frame rate", frame_cnt/(total_time-5))
print("detect times", detect_times)
# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
