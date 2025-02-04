import cv2
import numpy as np
import socket

# IP Camera URL
URL = "http://192.168.11.2"
STREAM_URL = f"{URL}:81/stream"

# ESP32-CAM details
ESP32_IP = "192.168.11.2"
ESP32_PORT = 12345

# Establish a UDP socket connection
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Use FFMPEG backend to read MJPEG stream
cap = cv2.VideoCapture(STREAM_URL, cv2.CAP_FFMPEG)

# Initialize the background subtractor
# bg_subtractor = cv2.createBackgroundSubtractorMOG2()

def adjust_brightness_contrast(image, brightness=30, contrast=50):
    beta = brightness - 50  # Shift to make trackbar range center at 0
    alpha = 1 + (contrast - 50) / 50.0  # Adjust contrast scale
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

# Callback function for trackbars (does nothing but required)
def nothing(x):
    pass

# Create a fixed window for trackbars with increased width
cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Controls", 400, 600)  # Fixed width and height

# Create Brightness and Contrast trackbars
cv2.createTrackbar("Brightness", "Controls", 50, 100, nothing)  # Range: 0 to 100
cv2.createTrackbar("Contrast", "Controls", 50, 100, nothing)    # Range: 0 to 100

# Create HSV range trackbars
cv2.createTrackbar("H_min", "Controls", 85, 179, nothing)
cv2.createTrackbar("H_max", "Controls", 140, 179, nothing)
cv2.createTrackbar("S_min", "Controls", 120, 255, nothing)
cv2.createTrackbar("S_max", "Controls", 225, 255, nothing)
cv2.createTrackbar("V_min", "Controls", 70, 255, nothing)
cv2.createTrackbar("V_max", "Controls", 225, 255, nothing)

if not cap.isOpened():
    print("Error: Could not open video stream.")
else:
    print("Stream opened successfully!")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Get current values from trackbars
        brightness = cv2.getTrackbarPos("Brightness", "Controls")
        contrast = cv2.getTrackbarPos("Contrast", "Controls")

        # Get HSV range values
        h_min = cv2.getTrackbarPos("H_min", "Controls")
        h_max = cv2.getTrackbarPos("H_max", "Controls")
        s_min = cv2.getTrackbarPos("S_min", "Controls")
        s_max = cv2.getTrackbarPos("S_max", "Controls")
        v_min = cv2.getTrackbarPos("V_min", "Controls")
        v_max = cv2.getTrackbarPos("V_max", "Controls")

        # Adjust brightness and contrast
        frame = adjust_brightness_contrast(frame, brightness=brightness, contrast=contrast)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Apply background subtraction
        #fg_mask = bg_subtractor.apply(frame)

        # Define HSV range dynamically from trackbars
        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        # Mask for color detection
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Combine the foreground mask and color mask
        #final_mask = cv2.bitwise_and(mask, fg_mask)

        # Show the final mask for debugging
        #cv2.imshow("Final Mask", final_mask)
        cv2.imshow("Final Mask", mask)

        #contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            perimeter = cv2.arcLength(largest_contour, True)

            circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0

            if circularity > 0.3 and area > 50:  # Lowered thresholds
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)

                if radius > 15:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

                    width = frame.shape[1]
                    center_frame = width // 2
                    dynamic_margin = width // 17  # ตรงกลางแคบลง สามารถทำได้โดยลดค่า dynamic_margin
                    if radius > 50:
                        dynamic_margin = width // 10

                    if x < center_frame - dynamic_margin: #left
                        position = "l"
                    elif x > center_frame + dynamic_margin: #right
                        position = "r"
                    else:
                        position = "c"

                    cv2.putText(frame, f"Position: {position}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    sock.sendto(position.encode(), (ESP32_IP, ESP32_PORT))
                    print(f"Sent: {position}, x: {int(x)}, margin: {dynamic_margin}")

        cv2.imshow("Camera Stream", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # Exit on 'ESC' key
            break

cap.release()
cv2.destroyAllWindows()
sock.close()
