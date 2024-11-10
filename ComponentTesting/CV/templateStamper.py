import cv2
frame_num = 0
def save_frame(frame, temp_count, frame_num):
    temp_count = frame_num
    # cv2.imwrite(f"Sample_Video\sample1\Frame_{frame_num}_Templates\Template_{temp_count}.png", frame)
    # print(f"Template saved as Template_{temp_count}.png")
    cv2.imwrite(f"LiveFrames\Template_{temp_count}.png", frame)
    print(f"Template saved as Template_{temp_count}.png")

def main():
    # image_path = f"Sample_Video\sample1\Frame_{frame_num}.png"
    image_path = f"liveFrames\Frame_{frame_num}.png"
    frame = cv2.imread(image_path)
    frame = cv2.resize(frame, (640, 480))
    frame_copy = frame.copy()

    height = 26
    width = 26
    xcursor = 0
    ycursor = 0
    temp_count = 0
    while True:
        frame = frame_copy.copy()
        cv2.rectangle(frame, (xcursor, ycursor), (xcursor+width, ycursor+height), (0, 0, 255), 1)
        cv2.imshow("Frame", frame)

        key = cv2.waitKey(10) & 0xFF
        if key == ord('j'):
            xcursor -= 1
            xcursor = max(0, xcursor)
            xcursor = min(xcursor, frame.shape[1]-width)
        elif key == ord('k'):
            ycursor += 1
            ycursor = max(0, ycursor)
            ycursor = min(ycursor, frame.shape[0]-height)
        elif key == ord('l'):
            xcursor += 1
            xcursor = max(0, xcursor)
            xcursor = min(xcursor, frame.shape[1]-width)
        elif key == ord('i'):
            ycursor -= 1
            ycursor = max(0, ycursor)
            ycursor = min(ycursor, frame.shape[0]-height)
        elif key == ord('['):
            height -= 1
            width -= 1
        elif key == ord(']'):
            height += 1
            width += 1
        elif key == ord('s'):
            save_frame(frame_copy[ycursor:ycursor+height,xcursor:xcursor+width], temp_count, frame_num)
            temp_count += 1
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()