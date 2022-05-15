import time

time.sleep(3)
print("starting")
while True:
    a = input()
    if (a[0]=="d" and a[1]=="i"):
        target = a.split("_")
        distance, angle = target[1], target[3]
        print(distance, angle)
    # time.sleep(0.1)
