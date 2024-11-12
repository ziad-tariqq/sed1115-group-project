from machine import Pin, ADC, PWM
import time

# Initialize feedback pins and PWM control for shoulder and elbow joints
shoulder_feedback = ADC(Pin(28))  # Example feedback pin for shoulder
elbow_feedback = ADC(Pin(27))     # Example feedback pin for elbow

shoulder_pwm = PWM(Pin(15))  # Example PWM pin for shoulder control
elbow_pwm = PWM(Pin(14))     # Example PWM pin for elbow control

shoulder_pwm.freq(50)  # Servo frequency (e.g., 50 Hz for standard servos)
elbow_pwm.freq(50)

# Define a function to gradually move a joint and record its boundaries
def calibrate_joint(pwm, feedback):
    min_position = 65535  # Start with max ADC value
    max_position = 0      # Start with min ADC value
    pwm.duty_u16(0)       # Start at the initial position
    time.sleep(1)

    # Move the joint slowly across its range
    for position in range(0, 65535, 500):  # Adjust the increment as needed
        pwm.duty_u16(position)
        time.sleep(0.05)  # Allow time for the servo to move
        feedback_value = feedback.read_u16()  # Read feedback

        # Track the min and max feedback values
        if feedback_value < min_position:
            min_position = feedback_value
        if feedback_value > max_position:
            max_position = feedback_value

    pwm.duty_u16(0)  # Return to initial position
    return min_position, max_position

# Run the calibration for shoulder and elbow
shoulder_limits = calibrate_joint(shoulder_pwm, shoulder_feedback)
elbow_limits = calibrate_joint(elbow_pwm, elbow_feedback)

# Output the calibration results
print("Shoulder limits:", shoulder_limits)
print("Elbow limits:", elbow_limits)
