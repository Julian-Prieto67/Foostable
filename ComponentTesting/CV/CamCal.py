import cv2 as cv
import numpy as np
import cv2 as cv
import argparse
def load_video_as_images(video_path):
    # Initialize a list to store frames
    frames = []

    # Open the video file
    cap = cv.VideoCapture(video_path)

    # Check if video opened successfully
    if not cap.isOpened():
        print(f"Error opening video file {video_path}")
        return frames

    while True:
        # Read a frame
        ret, frame = cap.read()

        # Break the loop if no frame is returned
        if not ret:
            break

        # Append the frame to the list
        frames.append(frame)

    # Release the video capture object
    cap.release()

    return frames


# video_path = 'calibration2.mp4'
# frames = load_video_as_images(video_path)
parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()

## [cap]
cap = cv.VideoCapture(args.camera, cv.CAP_DSHOW)
## [cap]
frames = []
NumofPics = 5
count = 0
while count < NumofPics:
    ret, frame = cap.read()
    if cv.waitKey(1) & 0xFF == ord('s'):
        print(f"Frame {count} added")
        frames.append(frame)
        count += 1
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    cv.imshow('frame', frame)
cap.release()
# Print the number of frames loaded
print(f"Total number of frames loaded: {len(frames)}")

# Prepare object points (3D points in the real world space)

square_size = 25  # 25mm

# Define the number of inner corners per a chessboard row and column
pattern_size = (10, 7) #vertices

objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all the images
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane

# Capture images or load them
totalframes = len(frames)
images = []
images = frames
# images.append(frames[10])
# images.append(frames[10+int(totalframes/2)])
# images.append(frames[totalframes-10])

for img in images:
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (10,7), None)
    
    if ret:
        # print("Chessboard detected.")
        objpoints.append(objp)
        imgpoints.append(corners)
        cv.drawChessboardCorners(img, (10,7), corners, ret)
        # cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

# Calibrate the camera
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# File path to save the data
file_path = 'calibration_data.xml'
fs = cv.FileStorage(file_path, cv.FILE_STORAGE_WRITE)
fs.write("camera_matrix", camera_matrix)
fs.write("dist_coeffs", dist_coeffs)

# Write rvecs and tvecs individually
fs.startWriteStruct("rvecs", cv.FILE_NODE_SEQ)
for rvec in rvecs:
    fs.write("", rvec)
fs.endWriteStruct()

fs.startWriteStruct("tvecs", cv.FILE_NODE_SEQ)
for tvec in tvecs:
    fs.write("", tvec)
fs.endWriteStruct()

fs.release()

print("Calibration data saved successfully.")

print("Calibration data saved successfully.")