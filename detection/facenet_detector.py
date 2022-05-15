import jetson.inference
import jetson.utils

net = jetson.inference.detectNet("facenet", threshold=0.05)
camera = jetson.utils.videoSource("/dev/video0",['--input-width=800','--input-height=600', '--input-codec=mjpeg'])      # '/dev/video0' for V4L2
display = jetson.utils.videoOutput() # 'my_video.mp4' for file

while display.IsStreaming():
	img = camera.Capture()
	
	detections = net.Detect(img)
	display.Render(img)
	display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

