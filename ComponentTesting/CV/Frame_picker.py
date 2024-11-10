import cv2
import argparse

def save_frame(frame, frame_count):
    cv2.imwrite(f"Frame_{frame_count}.png", frame)
    print(f"Frame {frame_count} saved as Frame_{frame_count}.png")

def main():
    # video_path = 'Sample_Video/NormalGRAYConditions1.mp4'
    # video_path = 'Sample_Video/Normal conditions1.mp4'
    parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
    parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
    args = parser.parse_args()
    # cap = cv2.VideoCapture(video_path)
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    frame_count = 0
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # print(frame.shape)
    frame = frame[:, :, 1]
    while True:
        
        if not ret:
            break

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('n'):
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            frame = frame[:, :, 1]
            frame = cv2.resize(frame, (640, 480))
            frame_count += 1
        elif key == ord('s'):
            save_frame(frame, frame_count)
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()