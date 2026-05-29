import cv2

# Load the exact 4x4 dictionary your detection node uses
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Generate Marker ID 0 with a resolution of 400x400 pixels
marker_img = cv2.aruco.generateImageMarker(aruco_dict, 0, 400)

# Save it as aruco.png in your current directory
cv2.imwrite("aruco.png", marker_img)
print("aruco.png generated successfully!")