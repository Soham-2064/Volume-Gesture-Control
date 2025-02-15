import cv2
import time
import numpy as np
import Hand_Tracking_Module as htm
import math
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume

# Defining the height and width of the camera
wcam, hcam = 640, 480

cap = cv2.VideoCapture(0)
cap.set(3, wcam)
cap.set(4, hcam)

ptime = 0
ctime = 0

# Initialize hand detector
detector = htm.handDetector(detectioncon=0.8)

# Audio utilities setup
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = interface.QueryInterface(IAudioEndpointVolume)
volrange = volume.GetVolumeRange()
minvol = volrange[0]
maxvol = volrange[1]

vol = 0
volBar = 400
volPer = 0

while True:
    success, img = cap.read()
    img = detector.findHands(img)
    lmlist = detector.findposition(img, draw=False)

    if len(lmlist) != 0:
        # Getting the coordinates of the thumb (4) and index finger (8) tips
        x1, y1 = lmlist[4][1], lmlist[4][2]
        x2, y2 = lmlist[12][1], lmlist[12][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        # Draw circles and line
        cv2.circle(img, (x1, y1), 10, (255, 0, 255), cv2.FILLED)
        cv2.circle(img, (x2, y2), 10, (255, 0, 255), cv2.FILLED)
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.circle(img, (cx, cy), 10, (255, 0, 255), cv2.FILLED)

        # Calculate the length of the line between thumb and index finger
        length = math.hypot(x2 - x1, y2 - y1)

        # Map length to volume range
        vol = np.interp(length, [50, 300], [minvol, maxvol])
        volBar = np.interp(length, [50, 300], [400, 150])
        volPer = np.interp(length, [50, 300], [0, 100])
        volume.SetMasterVolumeLevel(vol, None)

        if length < 50:
            cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED)

    # Volume bar and percentage text
    cv2.rectangle(img, (50, 150), (85, 400), (0, 255, 0), 3)
    cv2.rectangle(img, (50, int(volBar)), (85, 400), (0, 255, 0), cv2.FILLED)
    cv2.putText(img, f'{int(volPer)}%', (40, 450), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 3)

    # Calculate FPS
    ctime = time.time()
    fps = 1 / (ctime - ptime)
    ptime = ctime

    cv2.putText(img, f'FPS: {int(fps)}', (40, 70), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 3)

    # Display image
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


