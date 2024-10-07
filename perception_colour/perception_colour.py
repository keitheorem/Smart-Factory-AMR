import cv2
import numpy as np

# Function to filter red objects, find the largest red area, and get its center with an oriented bounding box and angle
def filter_largest_red_object_with_center_and_angle(frame):
    # Convert the frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range of red color in HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    lower_blue = np.array([100, 150, 100])
    upper_blue = np.array([140, 255, 255])

    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask3 = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Combine both masks
    mask = mask1 + mask2 + mask3
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Filter by area to avoid small noise
        if cv2.contourArea(largest_contour) > 100:
            # Get the minimum area rectangle (oriented bounding box)
            rect = cv2.minAreaRect(largest_contour) 
            
            # Get box points (coordinates of the four corners of the oriented bounding box)
            box = cv2.boxPoints(rect)
            box = np.int0(box)  # Convert to integer
            
            # Draw the oriented bounding box
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
            
            # Calculate the area of the bounding box
            width, height = rect[1]
            area = width * height
            print(f"Bounding Rectangle Area: {area} pixels")
            
            if area > 4000: 
                print("Cube Detected")
                
            # Get the center of the bounding box
            cx, cy = np.int0(rect[0])  # Rect center is the center of the bounding box
            
            # Draw a circle at the center of the largest red object
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)  # Blue circle
            
            # Get the angle of rotation of the bounding box
            angle = rect[2]
            if width < height:
                angle = angle + 90  # Adjust angle if the height is greater than width
            
            # Display the angle
            print(f"Rotation Angle: {angle} degrees")
            
            # Display the angle on the frame
            cv2.putText(frame, f"Angle: {angle:.2f} deg", (cx - 50, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    return frame

# Initialize webcam
cap = cv2.VideoCapture(0)

while True:
    # Read frame from webcam
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break
    
    # Filter and highlight the largest red object with an oriented bounding box, its center, and angle
    largest_red_with_center_angle_frame = filter_largest_red_object_with_center_and_angle(frame)
    
    # Display the frame with the largest red object and its center highlighted
    cv2.imshow('Largest Red Object with Center and Angle', largest_red_with_center_angle_frame)
    
    # Break loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()
