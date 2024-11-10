import cv2
import numpy as np

# Load the still image
image = cv2.imread('Sample_Video\Frames\Frame_0.png')

# Make a copy of the image to reset the rectangle position each time
image_copy = image.copy()

# Rectangle's starting position and size
x, y = 50, 50  # Starting coordinates
width, height = 100, 50  # Width and height of the rectangle
dx, dy = 5, 3  # Change in position for each frame (speed of movement)

# Get image dimensions
image_height, image_width = image.shape[:2]

# Create a window
cv2.namedWindow('Moving Rectangle', cv2.WINDOW_AUTOSIZE)

while True:
    # Copy the original image to clear the previous rectangle
    frame = image_copy.copy()

    # Draw the rectangle (BGR color and thickness)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 3)

    # Show the frame with the moving rectangle
    cv2.imshow('Moving Rectangle', frame)

    # Update the rectangle's position
    x += dx
    y += dy

    # Check for boundary collision and reverse direction if needed
    if x + width >= image_width or x <= 0:
        dx = -dx  # Reverse horizontal direction
    if y + height >= image_height or y <= 0:
        dy = -dy  # Reverse vertical direction

    # Break the loop when 'q' is pressed
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# Release the window
cv2.destroyAllWindows()
