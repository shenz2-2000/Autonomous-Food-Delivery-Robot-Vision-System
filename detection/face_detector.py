import face_recognition
import cv2
import os
import numpy as np

# This is a demo of running face recognition on live video from your webcam. It's a little more complicated than the
# other example, but it includes some basic performance tweaks to make things run a lot faster:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# PLEASE NOTE: This example requires OpenCV (the `cv2` library) to be installed only to read from your webcam.
# OpenCV is *not* required to use the face_recognition library. It's only required if you want to run this
# specific demo. If you have trouble installing it, try any of the other demos that don't require it instead.
print('starting now')
# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)
print('streaming now')

# Initialize some variables
face_locations = []
face_encodings = []
process_this_frame = 0

while True:
    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Resize frame of video to 1/4 size for faster face recognition processing
    # small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    # rgb_small_frame = small_frame[:, :, ::-1]
    rgb_small_frame = frame[:, :, ::-1]

    # Only process every other frame of video to save time
    if process_this_frame == 4:
        process_this_frame = 0
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)

    process_this_frame += 1


    # Display the results
    # for (top, right, bottom, left) in face_locations:
    #     # Scale back up face locations since the frame we detected in was scaled to 1/4 size
    #     cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
    if len(face_locations)>0:
        print('detected')

    # Display the resulting image
    # cv2.imshow('Video', frame)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
