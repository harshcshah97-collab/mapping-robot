import depthai as dai
import cv2
import time

print("--- OAK-D Vision Check ---")

with dai.Pipeline() as pipeline:
    # 1. Create a standard camera node
    camRgb = pipeline.create(dai.node.Camera).build()
    
    # 2. Request a standard 720p image formatted for OpenCV (BGR Interleaved)
    camOutput = camRgb.requestOutput((1280, 720), type=dai.ImgFrame.Type.BGR888i)
    
    # 3. Create a queue to pull the frame to the Pi
    qRgb = camOutput.createOutputQueue(maxSize=1, blocking=True)

    print("Waking up camera...")
    pipeline.start()
    
    print("Adjusting auto-exposure (waiting 2 seconds)...")
    time.sleep(2) 

    print("Taking picture...")
    # Grab the frame from the queue
    inRgb = qRgb.get()
    
    # Convert it to a standard OpenCV image matrix
    frame = inRgb.getCvFrame()

    # Save the image to the SD card
    filename = "vision_test.jpg"
    cv2.imwrite(filename, frame)
    print(f"Success! Image saved as '{filename}' in your current directory.")