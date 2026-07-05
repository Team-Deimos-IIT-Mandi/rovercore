import cv2
import cv2.aruco as aruco
import sys

def detect_marker_in_image(image_path):
    # Load the image
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image from {image_path}")
        sys.exit(1)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Try different dictionaries if we don't know the exact one
    aruco_dicts = [
        ("DICT_4X4_50", aruco.DICT_4X4_50),
        ("DICT_5X5_50", aruco.DICT_5X5_50),
        ("DICT_6X6_50", aruco.DICT_6X6_50),
        ("DICT_7X7_50", aruco.DICT_7X7_50),
        ("DICT_ARUCO_ORIGINAL", aruco.DICT_ARUCO_ORIGINAL)
    ]
    
    found = False
    for dict_name, dict_id in aruco_dicts:
        aruco_dict = aruco.getPredefinedDictionary(dict_id)
        aruco_params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, aruco_params)
        
        corners, ids, rejected = detector.detectMarkers(gray)
        
        if ids is not None and len(ids) > 0:
            print(f"Success! Detected marker using dictionary {dict_name}")
            for i in range(len(ids)):
                print(f"Marker ID: {ids[i][0]}")
                print(f"Corners:\n{corners[i]}")
            found = True
            
            # Optionally draw markers and save the output image
            output_image = image.copy()
            aruco.drawDetectedMarkers(output_image, corners, ids)
            output_path = image_path.replace(".png", "_detected.png")
            cv2.imwrite(output_path, output_image)
            print(f"Saved detection visualization to {output_path}")
            break
            
    if not found:
        print("No ArUco markers detected in the image.")

if __name__ == '__main__':
    image_path = '/home/deepak/arc/rovercore/rover_description/models/aruco_marker.png'
    detect_marker_in_image(image_path)
