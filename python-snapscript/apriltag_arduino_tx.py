import cv2
import numpy as np
from apriltag import apriltag
import serial
import struct
import time

# This example will detect an apriltag and send a "tx" metric to a microcontroller attached to the USB-A port of a Limelight 3

tx = 128
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
detector = apriltag("tag36h11")

def calculate_tx_byte(corners, image_width):
    """Calculate the tx value as a byte (0-255) where 128 is the center."""
    tag_center_x = np.mean(corners[:, 0])
    image_center_x = image_width / 2
    tx = (tag_center_x - image_center_x) / image_center_x
    # Convert -1 to 1 range to 0 to 255 range
    tx_byte = int((tx + 1) * 127.5)
    return max(0, min(255, tx_byte))  # Clamp the value between 0 and 255

def runPipeline(image, llrobot):
    global tx
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(img_gray)
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]   

    image_width = image.shape[1]
    image_height = image.shape[0]
    tx=128
    for detection in detections:
        corners = detection['lb-rb-rt-lt']
        tx = calculate_tx_byte(corners, image_width)
        for i in range(4):
            start_point = (int(corners[i][0]), int(corners[i][1]))
            end_point = (int(corners[(i+1) % 4][0]), int(corners[(i+1) % 4][1]))
            cv2.line(image, start_point, end_point, (0, 255, 0), 20)

    cv2.putText(image,
                f'TX: {tx}',
                (10, image_height-20), cv2.FONT_HERSHEY_SIMPLEX,
                1.5, (0, 255, 255),
                3, cv2.LINE_AA)

    try:
        ser.write(bytes([tx]))  # Send as a single byte
    except serial.SerialException as e:
        t=1

    return largestContour, image, llpython
