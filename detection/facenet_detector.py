import jetson.inference
import jetson.utils
import time
from heapq import nsmallest

VALID_HEIGHT = 10
VALID_WIDTH = 10

net = jetson.inference.detectNet("facenet", threshold=0.15)
camera = jetson.utils.videoSource("/dev/video0",['--input-codec=mjpeg'])      # '/dev/video0' for V4L2
# camera = jetson.utils.videoSource("/dev/video0",['--input-width=800','--input-height=600', '--input-codec=mjpeg'])      # '/dev/video0' for V4L2
# display = jetson.utils.videoOutput() # 'my_video.mp4' for file

# while display.IsStreaming():
ass = []

start = time.time()
while True:
	if time.time() - start > 60:
		break
	img = camera.Capture()
	detections = net.Detect(img)
	for detection in detections:
		# ass.append(detection.Area)
		print(detection)
# print("min 5 width at this range", nsmallest(5, ws))
# print("min 5 height at this range", nsmallest(5, hs))
