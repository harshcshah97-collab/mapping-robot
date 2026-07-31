import depthai as dai
import time

# A map for some common MobileNet-SSD labels to make the output readable
LABEL_MAP = {
    1: "aeroplane", 2: "bicycle", 3: "bird", 4: "boat", 5: "bottle",
    6: "bus", 7: "car", 8: "cat", 9: "chair", 10: "cow",
    11: "diningtable", 12: "dog", 13: "horse", 14: "motorbike", 15: "person",
    16: "pottedplant", 17: "sheep", 18: "sofa", 19: "train", 20: "tvmonitor"
}

print("--- OAK-D AI Sanity Check (V3 Native) ---")

with dai.Pipeline() as pipeline:
    
    print("Building camera and fetching AI model natively...")
    # 1. Create the universal Camera node
    cameraNode = pipeline.create(dai.node.Camera).build()
    
    # 2. Use V3's native Model Zoo! 
    # This automatically downloads the blob, sets the BGR format, links the camera, and configures the SSD decoder.
    model_description = dai.NNModelDescription("luxonis/mobilenet-ssd:300x300")
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, model_description)
    
    # Lower the confidence to 20% so we can see what the AI is guessing
    detectionNetwork.setConfidenceThreshold(0.2)

    # 3. Create the queue directly from the network output
    qDet = detectionNetwork.out.createOutputQueue(maxSize=4, blocking=False)

    print("Connecting to OAK-D camera...")
    try:
        pipeline.start()
        print("OAK-D connected successfully. Starting pipeline...")
        print("--- Live Detections ---")
        print("Point the camera at objects. Press Ctrl+C to exit.")
        
        while pipeline.isRunning():
            # Check for new detections
            inDet = qDet.tryGet()

            if inDet is not None:
                if len(inDet.detections) > 0:
                    print("---") 
                    for detection in inDet.detections:
                        # Map the numerical label to the string name
                        label = LABEL_MAP.get(detection.label, f"Label {detection.label}")
                        confidence = detection.confidence
                        print(f"Found: {label:<15} | Confidence: {confidence:.2f}")
            
            time.sleep(0.1)

    except RuntimeError as e:
        print(f"FATAL: Could not connect to the OAK-D camera.\nError: '{e}'")
        print("Ensure the camera is plugged in and not in use by another process.")
        exit(1)
    except KeyboardInterrupt:
        print("\n--- Test finished. ---")
