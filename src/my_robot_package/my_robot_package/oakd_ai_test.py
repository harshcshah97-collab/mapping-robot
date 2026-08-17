import depthai as dai
import time

# A map for some common MobileNet-SSD labels to make the output readable
LABEL_MAP = {
    1: "aeroplane", 2: "bicycle", 3: "bird", 4: "boat", 5: "bottle",
    6: "bus", 7: "car", 8: "cat", 9: "chair", 10: "cow",
    11: "diningtable", 12: "dog", 13: "horse", 14: "motorbike", 15: "person",
    16: "pottedplant", 17: "sheep", 18: "sofa", 19: "train", 20: "tvmonitor"
}


def main():
    """Run live on-device object detections as a DepthAI v3 diagnostic."""
    print("--- OAK-D AI Sanity Check (V3 Native) ---")
    try:
        with dai.Pipeline() as pipeline:
            print("Building camera and fetching AI model natively...")
            camera = pipeline.create(dai.node.Camera).build()
            model = dai.NNModelDescription(
                "luxonis/mobilenet-ssd:300x300"
            )
            network = pipeline.create(dai.node.DetectionNetwork).build(
                camera, model
            )
            network.setConfidenceThreshold(0.2)
            detections = network.out.createOutputQueue(
                maxSize=4, blocking=False
            )

            print("Connecting to OAK-D camera...")
            pipeline.start()
            print("Point the camera at objects. Press Ctrl+C to exit.")
            while pipeline.isRunning():
                packet = detections.tryGet()
                if packet is not None and packet.detections:
                    print("---")
                    for detection in packet.detections:
                        label = LABEL_MAP.get(
                            detection.label, f"Label {detection.label}"
                        )
                        print(
                            f"Found: {label:<15} | "
                            f"Confidence: {detection.confidence:.2f}"
                        )
                time.sleep(0.1)
    except RuntimeError as error:
        print(f"FATAL: Could not connect to the OAK-D camera: {error}")
        print("Ensure the camera is plugged in and not used by another process.")
        return 1
    except KeyboardInterrupt:
        print("\n--- Test finished. ---")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
