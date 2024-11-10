import cv2

def convert_to_gray(input_file, output_file):
    # Open the video file
    video = cv2.VideoCapture(input_file)
    if not video.isOpened():
        print("Error: Could not open the video file.")
        return

    # Get the video's properties
    width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = video.get(cv2.CAP_PROP_FPS)

    # Create a VideoWriter object to save the grayscale video
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height), isColor=False)

    while True:
        # Read a frame from the video
        ret, frame = video.read()

        # If the frame was not read successfully, break the loop
        if not ret:
            break

        # Convert the frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Write the grayscale frame to the output video
        writer.write(gray_frame)

    # Release the video and writer objects
    video.release()
    writer.release()

# Specify the input and output file paths
input_file = 'Sample_Video/Normal conditions1.mp4'
output_file = 'Sample_Video/NormalGRAYConditions1.mp4'

# Call the function to convert the video to grayscale
convert_to_gray(input_file, output_file)