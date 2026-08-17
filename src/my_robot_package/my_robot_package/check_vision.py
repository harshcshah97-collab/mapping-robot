import cv2
import depthai as dai
import time


def main():
    """Capture one RGB frame as a simple OAK-D hardware diagnostic."""
    print("--- OAK-D Vision Check ---")
    try:
        with dai.Pipeline() as pipeline:
            camera = pipeline.create(dai.node.Camera).build()
            output = camera.requestOutput(
                (1280, 720), type=dai.ImgFrame.Type.BGR888i
            )
            queue = output.createOutputQueue(maxSize=1, blocking=True)

            print("Waking up camera...")
            pipeline.start()
            time.sleep(2)
            frame = queue.get().getCvFrame()
    except RuntimeError as error:
        print(f"Could not use the OAK-D camera: {error}")
        return 1

    filename = "vision_test.jpg"
    if not cv2.imwrite(filename, frame):
        print(f"Could not save '{filename}'.")
        return 1
    print(f"Success! Image saved as '{filename}' in your current directory.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
