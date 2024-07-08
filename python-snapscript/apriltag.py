import cv2
import numpy as np
from apriltag import apriltag
detector = apriltag("tag36h11")
def drawDecorations(image):
    cv2.putText(image,
                'Limelight python script!',
                (0, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                .5, (0, 255, 0), 1, cv2.LINE_AA)
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(img_gray)
    largestContour = np.array([[]])
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    # Draw bounding boxes for each detection
    for detection in detections:
        corners = detection['lb-rb-rt-lt']
        for i in range(4):
            start_point = (int(corners[i][0]), int(corners[i][1]))
            end_point = (int(corners[(i+1) % 4][0]), int(corners[(i+1) % 4][1]))
            cv2.line(image, start_point, end_point, (0, 255, 0), 2)
    drawDecorations(image)
    # Return a contour, an image to stream, and optionally an array of up to 8 values for the "llpython" networktables array
    return largestContour, image, llpython