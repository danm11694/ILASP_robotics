import cv2
import os

# Directory containing images
image_dir = '/home/andrea/workspace/src/utils/util_pkg/templates/Phantom100_50_0'

# File to store pixel coordinates
coordinates_file = '/home/andrea/workspace/src/utils/util_pkg/templates/initial_points_100_50_0.txt'

pix2mm = 0.295

# Function to handle mouse click events
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        with open(coordinates_file, 'a') as f:
            # f.write(f"Image: {image_file}, Coordinates: ({x}, {y})\n")
            f.write(f"{x*pix2mm}, {y*pix2mm}\n")
        print(f"Clicked at pixel coordinates: ({x}, {y})")

# Get a list of image file names in the directory
image_files = [f for f in os.listdir(image_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]

for image_file in image_files:
    # Load the image
    image_path = os.path.join(image_dir, image_file)
    image = cv2.imread(image_path)
    
    if image is None:
        print(f"Unable to load image: {image_file}")
        continue

    # Create a window and set the mouse callback function
    cv2.namedWindow(image_file)
    cv2.setMouseCallback(image_file, mouse_callback)

    while True:
        # Display the image
        cv2.imshow(image_file, image)
        
        # Wait for a key press and check if it's the 'q' key to exit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Close the window when done with the image
    cv2.destroyWindow(image_file)

# Close all OpenCV windows
cv2.destroyAllWindows()
