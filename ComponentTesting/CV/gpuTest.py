import cv2
import numpy as np

# Check if CUDA is available
print("CUDA Available: ", cv2.cuda.getCudaEnabledDeviceCount())

# Load an image and upload it to GPU
image = cv2.imread('Sample_Video\Frame_-1_Templates\Template_0.png', cv2.IMREAD_GRAYSCALE)
gpu_image = cv2.cuda_GpuMat()
gpu_image.upload(image)

# Perform GaussianBlur on the GPU
gpu_blurred = cv2.cuda.createGaussianFilter(gpu_image.type(), -1, (15, 15), 0)
blurred_image = gpu_blurred.apply(gpu_image)

# Download result back to CPU and display
result = blurred_image.download()
cv2.imshow('Blurred Image', result)
cv2.waitKey(0)
cv2.destroyAllWindows()
