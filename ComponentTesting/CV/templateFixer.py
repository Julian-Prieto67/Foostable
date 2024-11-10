import cv2

# Load the grayscale image
image = cv2.imread('Sample_Video\Frame_-1_Templates\Template_0.png', cv2.IMREAD_GRAYSCALE)

# Get the dimensions of the image
height, width = image.shape

# Calculate the coordinates of the object in the middle
object_width = int(width / 4)
object_height = int(height / 4)
object_x = int(width / 2 - object_width / 2)
object_y = int(height / 2 - object_height / 2)

# Crop the image to isolate the object
isolated_object = image[object_y:object_y+object_height, object_x:object_x+object_width]

# Display the isolated object
cv2.imshow('Isolated Object', isolated_object)
cv2.waitKey(0)
cv2.destroyAllWindows()