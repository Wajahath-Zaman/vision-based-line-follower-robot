import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep, time
import threading # For potential future enhancements, currently not used for main loop

# -------------------- Motor Class (Adapted for 4 motors, based on your working logic) --------------------
class Motor():
    def __init__(self, EnaA, In1A, In2A, EnaB, In1B, In2B, EnaC, In1C, In2C, EnaD, In1D, In2D):
        # Motor A: Rear Left
        self.EnaA, self.In1A, self.In2A = EnaA, In1A, In2A
        # Motor B: Rear Right
        self.EnaB, self.In1B, self.In2B = EnaB, In1B, In2B
        # Motor C: Front Left
        self.EnaC, self.In1C, self.In2C = EnaC, In1C, In2C
        # Motor D: Front Right
        self.EnaD, self.In1D, self.In2D = EnaD, In1D, In2D

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup all pins as OUTPUT
        for pin in [EnaA, In1A, In2A, EnaB, In1B, In2B, EnaC, In1C, In2C, EnaD, In1D, In2D]:
            GPIO.setup(pin, GPIO.OUT)

        # Initialize PWM for all Enable pins (100 Hz frequency)
        self.pwmA = GPIO.PWM(EnaA, 100)
        self.pwmB = GPIO.PWM(EnaB, 100)
        self.pwmC = GPIO.PWM(EnaC, 100)
        self.pwmD = GPIO.PWM(EnaD, 100)
        
        # Start all PWMs with 0% duty cycle (stopped)
        for pwm in [self.pwmA, self.pwmB, self.pwmC, self.pwmD]:
            pwm.start(0)

    def move(self, speed_percent=0, turn_percent=0, t=0):
        """
        Controls the robot's movement.
        speed_percent: Overall forward/backward speed (-100 to 100).
                       Note: Your PID outputs [-1, 1], so you'll scale that to turn_percent.
        turn_percent: Steering amount (-100 for max left, 100 for max right).
        t: Duration to move (in seconds). If 0, moves indefinitely until new command.
        """
        # Calculate individual motor speeds based on overall speed and turn
        # A positive turn_percent makes the robot turn right (right motor faster, left motor slower)
        # A negative turn_percent makes the robot turn left (left motor faster, right motor slower)
        leftSpeed = speed_percent - turn_percent
        rightSpeed = speed_percent + turn_percent

        # Clamp speeds to valid range (-100 to 100)
        leftSpeed = max(min(leftSpeed, 100), -100)
        rightSpeed = max(min(rightSpeed, 100), -100)

        # Apply speed and direction to each motor
        self._set_motor(self.pwmA, self.In1A, self.In2A, leftSpeed) # Rear Left Motor
        self._set_motor(self.pwmC, self.In1C, self.In2C, leftSpeed) # Front Left Motor
        self._set_motor(self.pwmB, self.In1B, self.In2B, rightSpeed) # Rear Right Motor
        self._set_motor(self.pwmD, self.In1D, self.In2D, rightSpeed) # Front Right Motor

        if t > 0:
            sleep(t)

    def _set_motor(self, pwm, in1, in2, speed):
        """
        Helper function to set individual motor speed and direction.
        This uses the direction logic you provided as working:
        HIGH on in1, LOW on in2 for positive speed (forward).
        """
        GPIO.output(in1, GPIO.HIGH if speed > 0 else GPIO.LOW)
        GPIO.output(in2, GPIO.LOW if speed > 0 else GPIO.HIGH)
        pwm.ChangeDutyCycle(abs(speed))

    def stop(self):
        """Stops all motors immediately."""
        for pwm in [self.pwmA, self.pwmB, self.pwmC, self.pwmD]:
            pwm.ChangeDutyCycle(0) # Set duty cycle to 0
        # Ensure all input pins are LOW to fully stop/brake motors
        for pin in [self.In1A, self.In2A, self.In1B, self.In2B, self.In1C, self.In2C, self.In1D, self.In2D]:
            GPIO.output(pin, GPIO.LOW)

    # Simplified direct control methods (adjusted to use speed_percent range of move)
    def move_forward(self, speed_percent=20): # Adjusted for percentage
        self.move(speed_percent, 0, 0)

    def move_backward(self, speed_percent=20): # Adjusted for percentage
        self.move(-speed_percent, 0, 0)

    def turn_left(self, turn_percent=50): # Adjusted for percentage
        self.move(0, -turn_percent, 0)

    def turn_right(self, turn_percent=50): # Adjusted for percentage
        self.move(0, turn_percent, 0)

# -------------------- PID Controller Class --------------------
class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1, 1), integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limit = integral_limit

        self.previous_error = 0
        self.integral = 0
        self.last_time = None

    def update(self, error, current_time_ms=None):
        """
        Updates the PID controller with the current error.
        current_time_ms should be in milliseconds for consistent dt calculation.
        """
        if current_time_ms is None:
            current_time_ms = time() * 1000 # Convert seconds to milliseconds

        if self.last_time is None:
            self.last_time = current_time_ms
            self.previous_error = error
            self.integral = 0
            return 0.0

        dt = (current_time_ms - self.last_time) / 1000.0 # Convert dt to seconds

        if dt == 0: # Avoid division by zero
            return self.previous_error # Or return 0.0, depending on desired behavior

        P = self.kp * error

        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        I = self.ki * self.integral

        derivative = (error - self.previous_error) / dt
        D = self.kd * derivative

        output = P + I + D
        output = max(min(output, self.output_limits[1]), self.output_limits[0])

        self.previous_error = error
        self.last_time = current_time_ms

        return output

    def reset(self):
        """Resets the PID controller state."""
        self.previous_error = 0
        self.integral = 0
        self.last_time = None

# -------------------- Vision & Lane Detection Functions --------------------

def thresholding(img):
    """
    Applies adaptive thresholding and morphological operations to detect a black line.
    (Generally more robust than fixed HSV thresholds for black lines in varying lighting)
    """
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 0)
    
    # Adaptive thresholding: tune blockSize (odd, e.g., 11, 13, 15) and C (constant subtracted)
    imgThresh = cv2.adaptiveThreshold(imgBlur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 10)
    
    kernel = np.ones((3, 3), np.uint8)
    imgThresh = cv2.morphologyEx(imgThresh, cv2.MORPH_CLOSE, kernel)
    imgThresh = cv2.morphologyEx(imgThresh, cv2.MORPH_OPEN, kernel)
    
    return imgThresh

def warpImg(img, points, w, h, inv=False):
    """Applies a perspective transform (warp) to an image."""
    pts1 = np.float32(points)
    pts2 = np.float32([[0,0],[w,0],[0,h],[w,h]])
    matrix = cv2.getPerspectiveTransform(pts2, pts1) if inv else cv2.getPerspectiveTransform(pts1, pts2)
    return cv2.warpPerspective(img, matrix, (w, h))

def initializeTrackbars(initialTracbarVals, wT=480, hT=240):
    """Initializes trackbars for adjusting perspective transform points."""
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", initialTracbarVals[0], wT // 2, lambda x: None)
    cv2.createTrackbar("Height Top", "Trackbars", initialTracbarVals[1], hT, lambda x: None)
    cv2.createTrackbar("Width Bottom", "Trackbars", initialTracbarVals[2], wT // 2, lambda x: None)
    cv2.createTrackbar("Height Bottom", "Trackbars", initialTracbarVals[3], hT, lambda x: None)

def valTrackbars(wT=480, hT=240):
    """Reads values from the perspective transform trackbars."""
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    
    # Ensure points form a valid trapezoid (widthTop < widthBottom, heights are within bounds)
    # The current implementation assumes a trapezoid where top points are closer horizontally.
    # It might be good to add more robust validation/clamping here.
    
    return np.float32([
        (widthTop, heightTop),
        (wT - widthTop, heightTop),
        (widthBottom, heightBottom),
        (wT - widthBottom, heightBottom)
    ])

def drawPoints(img, points):
    """Draws the perspective transform points on the image."""
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 10, (0,0,255), cv2.FILLED)
    return img

def getHistogram(img, minPer=0.1, display=False, region=1):
    """
    Calculates the horizontal histogram of the image to find the line's center.
    Optionally displays the histogram.
    """
    histValues = np.sum(img[img.shape[0]//region:, :] if region > 1 else img, axis=0)
    
    # Apply a simple moving average filter to smooth the histogram
    kernel_size = 9
    if len(histValues) < kernel_size: # Handle case for very small images/histograms
        kernel_size = len(histValues) if len(histValues) > 0 else 1
    if kernel_size % 2 == 0: # Ensure kernel size is odd
        kernel_size += 1

    kernel = np.ones(kernel_size) / kernel_size
    histValues = np.convolve(histValues, kernel, mode='same')
    
    maxValue = np.max(histValues)
    minValue = minPer * maxValue
    
    # Find indices where histogram values are above a minimum threshold
    indexArray = np.where(histValues >= minValue)[0]
    
    if indexArray.size > 0:
        basePoint = int(np.average(indexArray)) # Average of all points above threshold
    else:
        basePoint = img.shape[1] // 2 # Default to center if no line detected

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for x, intensity in enumerate(histValues):
            # Scale histogram values for display
            height = int(intensity * img.shape[0] / maxValue) if maxValue > 0 else 0
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - height), (255, 0, 255), 1)
        cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist

    return basePoint

def stackImages(scale, imgArray):
    """
    Stacks multiple images into a single display window, resizing them based on scale.
    Handles grayscale to BGR conversion and None images.
    """
    rows = len(imgArray)
    cols = len(imgArray[0])
    
    # Ensure the first image is valid to determine base dimensions
    if imgArray[0][0] is None:
        print("Warning: First image in stackImages is None. Cannot establish reference size.")
        return np.zeros((240, 480, 3), dtype=np.uint8) # Return a blank image to prevent crash

    # Convert to BGR if grayscale for consistency before scaling reference
    if len(imgArray[0][0].shape) == 2:
        imgArray[0][0] = cv2.cvtColor(imgArray[0][0], cv2.COLOR_GRAY2BGR)
    
    # Calculate target dimensions for each cell after scaling
    scaled_width = int(imgArray[0][0].shape[1] * scale)
    scaled_height = int(imgArray[0][0].shape[0] * scale)
    
    # Create an empty array to store resized images
    resized_images = [[None for _ in range(cols)] for _ in range(rows)]

    for x in range(rows):
        for y in range(cols):
            img = imgArray[x][y]
            
            if img is None:
                # If an image is None, create a black image of the target scaled size
                resized_images[x][y] = np.zeros((scaled_height, scaled_width, 3), np.uint8)
                continue

            # Convert to BGR if grayscale for consistent stacking
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            # Resize every image to the calculated scaled dimensions
            resized_images[x][y] = cv2.resize(img, (scaled_width, scaled_height))
    
    # Now stack the consistently sized images horizontally then vertically
    hor_stacks = [np.hstack(resized_images[x]) for x in range(rows)]
    return np.vstack(hor_stacks)

# -------------------- Enhanced Control System --------------------
class RobotController:
    def __init__(self, motor):
        self.motor = motor
        self.line_following_active = False
        self.manual_control_active = False
        self.running = True # Flag to control main loop execution
        self.current_manual_action = None
        
        self.base_speed = 25 # Base speed for line following (percentage for Motor.move)
        self.turn_sensitivity = 70 # Scales PID output (-1 to 1) to turn_percent (-100 to 100)
                                    # e.g., if PID output is 0.5, turn_percent will be 0.5 * 70 = 35

        self.no_line_counter = 0
        self.max_no_line_count = 50 # Max frames to search before stopping

        # PID constants (tuned as per your initial values in the previous version)
        # These are crucial and might need fine-tuning for your specific robot/track
        self.pid_controller = PIDController(kp=0.0035, ki=0.0001, kd=0.0005, 
                                            output_limits=(-1, 1), integral_limit=50) # integral_limit to prevent windup
        
    def start_line_following(self):
        if not self.line_following_active:
            self.line_following_active = True
            self.manual_control_active = False # Disable manual when line following starts
            self.current_manual_action = None
            self.no_line_counter = 0
            self.pid_controller.reset() # Reset PID state for a fresh start
            print("Line following started.")
            
    def stop_robot(self):
        self.line_following_active = False
        self.manual_control_active = False
        self.current_manual_action = None
        self.motor.stop() # Stops all motors
        print("Robot stopped.")
        
    def manual_forward(self):
        if not self.line_following_active: # Only allow manual if not line following
            self.manual_control_active = True
            self.current_manual_action = "FORWARD"
            self.motor.move_forward(speed_percent=100) # Use percentage speed
            # print("Manual: Moving forward")
            
    def manual_backward(self):
        if not self.line_following_active:
            self.manual_control_active = True
            self.current_manual_action = "BACKWARD"
            self.motor.move_backward(speed_percent=100)
            # print("Manual: Moving backward")
            
    def manual_left(self):
        if not self.line_following_active:
            self.manual_control_active = True
            self.current_manual_action = "LEFT"
            self.motor.turn_left(turn_percent=30) # Adjusted turn for manual control
            # print("Manual: Turning left")
            
    def manual_right(self):
        if not self.line_following_active:
            self.manual_control_active = True
            self.current_manual_action = "RIGHT"
            self.motor.turn_right(turn_percent=30) # Adjusted turn for manual control
            # print("Manual: Turning right")
            
    def stop_manual_action(self):
        """Called when a manual key is released (key == 255 indicates no key pressed)"""
        if self.manual_control_active and not self.line_following_active:
            self.motor.stop()
            self.manual_control_active = False
            self.current_manual_action = None
            # print("Manual action stopped.")

# -------------------- Lane Detection & Motor Control Logic --------------------
def getLaneCurve(img, controller, display=2):
    """
    Processes the image to detect the lane and applies motor control based on robot state.
    """
    imgCopy = img.copy()
    imgResult = img.copy()

    # Get trackbar values for perspective transform
    hT, wT = img.shape[:2] # Use original image dimensions for trackbars reference
    points = valTrackbars(wT=wT, hT=hT)

    # Apply thresholding
    imgThres = thresholding(img) 

    # Apply perspective warp
    imgWarp = warpImg(imgThres, points, wT, hT)
    imgWarpPoints = drawPoints(imgCopy, points) # Draw points on original image copy

    # Get histogram and line center
    middlePoint, imgHist = getHistogram(imgWarp, display=True, minPer=0.1, region=3)
    
    imageCenter = wT // 2
    error = middlePoint - imageCenter # Error is deviation from center

    turnAmount = 0.0 # PID output
    scaled_turn = 0.0 # PID output scaled for motor control
    speed_percent = 0.0 # Motor speed in percentage

    # Check if a significant line is detected (sum of white pixels in warped image)
    # This threshold (0.04 * wT * hT) might need tuning
    lineDetected = np.sum(imgWarp) > (wT * hT * 0.04 * 255) # Multiply by 255 if sum is over pixel values

    direction_text = "STOPPED"
    direction_color = (0, 0, 255) # Red for stopped/unknown

    # --- Motor Control Logic (ONLY if line following is active) ---
    if controller.line_following_active:
        if not lineDetected:
            controller.no_line_counter += 1
            if controller.no_line_counter > controller.max_no_line_count:
                controller.motor.stop()
                controller.pid_controller.reset()
                direction_text = "NO LINE - STOPPED"
                direction_color = (0, 0, 255) # Red
                speed_percent = 0
                scaled_turn = 0
                turnAmount = 0
            else:
                direction_text = "SEARCHING"
                direction_color = (0, 255, 255) # Yellow
                # Gentle forward movement while searching
                controller.motor.move(speed_percent=5, turn_percent=0) # Low search speed
                speed_percent = 5
                scaled_turn = 0
                turnAmount = 0
        else: # Line detected
            controller.no_line_counter = 0 # Reset counter
            
            current_time_ms = time() * 1000
            turnAmount = controller.pid_controller.update(error, current_time_ms)
            
            # Scale the PID output to a turn percentage suitable for motor.move
            scaled_turn = turnAmount * controller.turn_sensitivity

            # Adjust speed based on how sharp the turn is
            # Slower on sharper turns, faster on straightaways
            speed_percent = controller.base_speed - abs(scaled_turn) * (controller.base_speed / 100) 
            speed_percent = max(speed_percent, 5) # Ensure a minimum forward speed

            controller.motor.move(speed_percent, scaled_turn, 0)

            if abs(error) < 15: # Threshold for considering "straight"
                direction_text = "FORWARD"
                direction_color = (0, 255, 0) # Green
            elif error > 0:
                direction_text = "RIGHT"
                direction_color = (255, 0, 0) # Blue
            else: # error < 0
                direction_text = "LEFT"
                direction_color = (0, 0, 255) # Red (Can be changed to different color if desired)

    else: # If line following is NOT active (e.g., in manual mode or stopped)
        controller.motor.stop() # Ensure motors are stopped if not actively controlled
        controller.no_line_counter = 0 # Reset line counter
        direction_text = controller.current_manual_action if controller.manual_control_active else "STOPPED"
        direction_color = (0, 255, 255) if controller.manual_control_active else (0, 0, 255)
        speed_percent = 0
        scaled_turn = 0
        turnAmount = 0


    # --- Display Logic (ALWAYS run this for visual feedback) ---
    # Re-create the inverse warp to overlay the detected lane back onto the original image
    imgInvWarp = warpImg(imgWarp, points, wT, hT, inv=True)
    imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR) # Convert to BGR to add color

    # Create a green overlay for the detected lane
    imgLaneColor = np.zeros_like(imgResult)
    imgLaneColor[:] = 0, 255, 0 # Green color
    imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor) # Apply green color only where line is detected

    # Blend the original image with the colored lane overlay
    imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)

    # Add text overlays to the result image
    cv2.putText(imgResult, f"DIR: {direction_text}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, direction_color, 3)
    cv2.putText(imgResult, f"ERROR: {int(error)}", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(imgResult, f"CENTER: {middlePoint}", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(imgResult, f"NO LINE: {controller.no_line_counter}", (30, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(imgResult, f"PID Output: {turnAmount:.2f}", (30, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    cv2.putText(imgResult, f"Scaled Turn: {scaled_turn:.2f}", (30, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(imgResult, f"Speed: {speed_percent:.2f}%", (30, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Draw reference lines on the result image
    cv2.line(imgResult, (imageCenter, 0), (imageCenter, hT), (255, 255, 0), 2) # Yellow center line
    cv2.line(imgResult, (middlePoint, 0), (middlePoint, hT), (0, 255, 255), 2) # Cyan line detection center

    # Stack images for combined display (Full Debug View)
    imgStacked = stackImages(0.7, ([img, imgWarpPoints, imgWarp], [imgHist, imgInvWarp, imgResult]))
    cv2.imshow('ImageStack', imgStacked)
    # The 'Result' window for display==1 is removed to simplify, as ImageStack provides more info.

    return error

# -------------------- Control Interface Functions --------------------
def create_control_window():
    """Creates the 'Robot Control' window."""
    control_img = np.zeros((500, 700, 3), dtype=np.uint8)
    cv2.namedWindow('Robot Control', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Robot Control', control_img) # Display initial black window
    
def draw_control_interface(controller):
    """Draws and updates the content of the 'Robot Control' window."""
    control_img = np.zeros((500, 700, 3), dtype=np.uint8) # Clear image each time

    # Determine robot status text and color
    status_text = "LINE FOLLOWING ACTIVE" if controller.line_following_active else "MANUAL CONTROL ACTIVE"
    if controller.current_manual_action:
        status_text += f" - {controller.current_manual_action}"
    elif not controller.line_following_active and not controller.manual_control_active:
        status_text = "STOPPED - IDLE"
    
    # Display main status
    cv2.putText(control_img, f"STATUS: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    # Header
    cv2.putText(control_img, "ENHANCED ROBOT CONTROL SYSTEM", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(control_img, "=" * 50, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    
    # Primary Controls section
    cv2.putText(control_img, "PRIMARY CONTROLS:", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(control_img, "S - START Line Following", (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, "X - STOP Robot", (10, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, "Q - QUIT Program", (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    # Manual Controls section
    cv2.putText(control_img, "MANUAL CONTROLS (when Line Following is OFF):", (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(control_img, "F - FORWARD", (10, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, "B - BACKWARD", (10, 305), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, "L - LEFT", (10, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, "R - RIGHT", (10, 355), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, "Release key to stop manual action", (10, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

    # Line Following Parameters section
    cv2.putText(control_img, "LINE FOLLOWING PARAMETERS:", (10, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(control_img, f"Base Speed: {controller.base_speed}%", (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    cv2.putText(control_img, f"Turn Sensitivity: {controller.turn_sensitivity:.2f}", (10, 475), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    # PID Constants display
    cv2.putText(control_img, "PID CONSTANTS:", (400, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(control_img, f"Kp: {controller.pid_controller.kp:.4f}", (400, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 150), 1)
    cv2.putText(control_img, f"Ki: {controller.pid_controller.ki:.4f}", (400, 475), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 150), 1)
    cv2.putText(control_img, f"Kd: {controller.pid_controller.kd:.4f}", (400, 500), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 150), 1)

    # Status indicators (rectangles with color change)
    line_color_indicator = (0, 255, 0) if controller.line_following_active else (50, 50, 50)
    manual_color_indicator = (0, 255, 0) if controller.manual_control_active and not controller.line_following_active else (50, 50, 50)
    
    cv2.rectangle(control_img, (500, 130), (680, 160), line_color_indicator, 2)
    cv2.putText(control_img, "LINE FOLLOW", (505, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, line_color_indicator, 1)
    
    cv2.rectangle(control_img, (500, 170), (680, 200), manual_color_indicator, 2)
    cv2.putText(control_img, "MANUAL MODE", (505, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, manual_color_indicator, 1)
    
    cv2.imshow('Robot Control', control_img)

# -------------------- Main Loop --------------------
def main():
    # --- IMPORTANT: Adjust these GPIO pin numbers according to your specific robot's wiring! ---
    # These pin numbers map to EnaA, In1A, In2A, EnaB, In1B, In2B, EnaC, In1C, In2C, EnaD, In1D, In2D
    # Where A,C are Left motors and B,D are Right motors
    # EnaX are PWM pins for speed control
    # In1X, In2X are direction control pins
    motor = Motor(EnaA=17, In1A=26, In2A=19,  # Rear Left Motor
                  EnaB=27, In1B=21, In2B=20,  # Rear Right Motor
                  EnaC=10, In1C=18, In2C=22,  # Front Left Motor
                  EnaD=9,  In1D=24, In2D=23)  # Front Right Motor

    controller = RobotController(motor)
    
    cap = cv2.VideoCapture(0) # 0 for default camera
    
    # Set camera resolution and FPS (adjust for performance if needed)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("Error: Could not open camera. Please check camera connection and permissions.")
        GPIO.cleanup()
        return

    # Initialize Trackbars for perspective transform
    # Initial values: [Width Top, Height Top, Width Bottom, Height Bottom]
    # These need to be tuned for your camera's view and line setup.
    # W=640, H=480 for the camera frame, but perspective transform works on 480x240 image
    # (rescaled in the loop before getLaneCurve)
    initializeTrackbars([100, 80, 20, 200], wT=480, hT=240) 
    
    # Create the control interface window
    create_control_window()

    print("\n" + "=" * 60)
    print("ENHANCED ROBOT CONTROL SYSTEM STARTED")
    print("=" * 60)
    print("Keyboard Controls:")
    print("  'S' - Start Line Following Mode")
    print("  'X' - Stop Robot (exits line following and manual control)")
    print("  'F' - Manual Forward (only when line following is OFF)")
    print("  'B' - Manual Backward (only when line following is OFF)")
    print("  'L' - Manual Turn Left (only when line following is OFF)")
    print("  'R' - Manual Turn Right (only when line following is OFF)")
    print("  Release manual key to stop manual action.")
    print("  'Q' - QUIT Program (exits everything and cleans up GPIO)")
    print("=" * 60 + "\n")
    
    try:
        while controller.running:
            ret, img = cap.read()
            if not ret:
                print("Error: Failed to grab frame. Attempting to re-open camera...")
                cap.release()
                cap = cv2.VideoCapture(0)
                sleep(1) # Give camera time to restart
                if not cap.isOpened():
                    print("Error: Could not re-open camera. Exiting.")
                    break # Exit if camera fails to re-open
                continue

            # Resize image for processing efficiency (match trackbar dimensions)
            img = cv2.resize(img, (480, 240)) 
                                
            # Process the image, get lane curve, and apply motor control if line following is active
            # This function also handles updating the 'ImageStack' window
            getLaneCurve(img, controller, display=2) # display=2 for full stacked view

            # Update and display the custom 'Robot Control' interface
            draw_control_interface(controller)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord('s') or key == ord('S'):
                controller.start_line_following()
            elif key == ord('x') or key == ord('X'):
                controller.stop_robot()
                sleep(0.1) # Short delay to ensure stop command is processed
            # Manual control keys only activate if line following is NOT active
            elif key == ord('f') or key == ord('F'):
                controller.manual_forward()
            elif key == ord('b') or key == ord('B'):
                controller.manual_backward()
            elif key == ord('l') or key == ord('L'):
                controller.manual_left()
            elif key == ord('r') or key == ord('R'):
                controller.manual_right()
            elif key == ord('q') or key == ord('Q'):
                controller.running = False # Set flag to exit loop
                break # Exit the while loop
            elif key == 255: # No key pressed, stop any active manual movement
                controller.stop_manual_action()
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user (Ctrl+C).")
        
    finally:
        controller.running = False # Ensure loop terminates
        controller.stop_robot() # Stop motors explicitly
        cap.release() # Release camera resource
        cv2.destroyAllWindows() # Close all OpenCV windows
        GPIO.cleanup() # Clean up GPIO pins
        print("Program ended - GPIO cleaned up, resources released.")

if __name__ == "__main__":
    main()