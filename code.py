from machine import ADC, Pin, PWM
import time
import random  # Needed for Cloryel's mock data generation

# Ziad's Contribution: Initialize ADC for potentiometers
x_pot = ADC(Pin(26))  # X potentiometer on GPIO 26
y_pot = ADC(Pin(27))  # Y potentiometer on GPIO 27

# Ziad's Contribution: Initialize pen control switch
pen_switch = Pin(15, Pin.IN, Pin.PULL_DOWN)  # Pen control switch on GPIO 15

# Frank's Contribution: Initialize PWM for shoulder, elbow, and pen servos
shoulder_pwm = PWM(Pin(14))  # Shoulder servo on GPIO 14
elbow_pwm = PWM(Pin(13))     # Elbow servo on GPIO 13
pen_pwm = PWM(Pin(12))       # Pen servo on GPIO 12

# Frank's Contribution: Set PWM frequency for servos
shoulder_pwm.freq(50)  # 50 Hz for standard servos
elbow_pwm.freq(50)
pen_pwm.freq(50)

# Frank's Contribution: Safety limits for servos
SHOULDER_MIN = 0
SHOULDER_MAX = 180
ELBOW_MIN = 0
ELBOW_MAX = 180
PEN_MIN = 0
PEN_MAX = 90

# Frank's Contribution: Function to set servo angle
def set_servo_angle(servo, angle):
    duty = int((angle / 180) * 65535)
    servo.duty_u16(duty)

# Frank's Contribution: Function to safely set servo angle
def safe_set_servo_angle(servo, angle, min_angle, max_angle):
    if min_angle <= angle <= max_angle:
        set_servo_angle(servo, angle)
    else:
        print("Angle out of range!")

# Frank's Contribution: Function to calibrate a joint and determine its movement boundaries
def calibrate_joint(pwm, feedback_adc):
    min_feedback = 65535  # Start with max ADC value
    max_feedback = 0      # Start with min ADC value
    pwm.duty_u16(0)       # Start at the initial position
    time.sleep(1)

    # Move the joint slowly across its range
    for position in range(0, 65535, 500):  # Adjust the increment as needed
        pwm.duty_u16(position)
        time.sleep(0.05)  # Allow time for the servo to move
        feedback_value = feedback_adc.read_u16()  # Read feedback

        # Track the min and max feedback values
        if feedback_value < min_feedback:
            min_feedback = feedback_value
        if feedback_value > max_feedback:
            max_feedback = feedback_value

    pwm.duty_u16(0)  # Return to initial position
    return min_feedback, max_feedback

# Ziad's Contribution: Function to read potentiometer values
def read_potentiometers():
    x_value = x_pot.read_u16()  # Returns a value between 0 and 65535
    y_value = y_pot.read_u16()
    return x_value, y_value

# Ziad's Contribution: Function to debounce the pen control switch
def debounce_switch(pin):
    stable_time = 20  # milliseconds
    last_state = pin.value()
    last_debounce_time = time.ticks_ms()

    while True:
        current_state = pin.value()
        if current_state != last_state:
            last_debounce_time = time.ticks_ms()
        if (time.ticks_diff(time.ticks_ms(), last_debounce_time) > stable_time):
            if current_state != pin.value():
                return current_state
        last_state = current_state

# Ziad's Contribution: Function to get processed input data
def get_input_data():
    x_value, y_value = read_potentiometers()  # Get potentiometer readings
    pen_state = debounce_switch(pen_switch)   # Get debounced switch state
    return {'x': x_value, 'y': y_value, 'pen': pen_state}  # Output as dictionary

# Cloryel's Contribution: Function to map input values to servo angles
def map_input_to_servo_angles(x_value, y_value):
    # Assuming x_value and y_value are between 0 and 65535
    # Map these values to servo angles (0 to 180 degrees)
    shoulder_angle = (y_value / 65535) * 180
    elbow_angle = (x_value / 65535) * 180
    return shoulder_angle, elbow_angle

# Cloryel's Contribution: Function to create mock data generator
def generate_mock_data():
    x_value = random.randint(0, 65535)
    y_value = random.randint(0, 65535)
    pen_state = random.choice([0, 1])
    return {'x': x_value, 'y': y_value, 'pen': pen_state}

# Jaures' Contribution: Testing framework and main function
def main():
    # Calibrate shoulder and elbow joints
    print("Calibrating shoulder joint...")
    shoulder_feedback = ADC(Pin(28))  # Example feedback pin for shoulder
    shoulder_limits = calibrate_joint(shoulder_pwm, shoulder_feedback)
    print("Shoulder limits:", shoulder_limits)

    print("Calibrating elbow joint...")
    elbow_feedback = ADC(Pin(27))     # Example feedback pin for elbow
    elbow_limits = calibrate_joint(elbow_pwm, elbow_feedback)
    print("Elbow limits:", elbow_limits)

    # Main loop to handle inputs
    while True:
        # Generate mock data (Cloryel's part)
        input_data = generate_mock_data()

        # Map input data to servo angles (Cloryel's part)
        shoulder_angle, elbow_angle = map_input_to_servo_angles(input_data['x'], input_data['y'])
        print("Mapped Angles - Shoulder:", shoulder_angle, "Elbow:", elbow_angle)

        # Frank's Contribution: Set servo angles with safety checks
        safe_set_servo_angle(shoulder_pwm, shoulder_angle, SHOULDER_MIN, SHOULDER_MAX)
        safe_set_servo_angle(elbow_pwm, elbow_angle, ELBOW_MIN, ELBOW_MAX)
        safe_set_servo_angle(pen_pwm, input_data['pen'] * 90, PEN_MIN, PEN_MAX)  # Example: pen up/down

        # Print input data for debugging
        print("Input Data:", input_data)
        time.sleep(0.5)  # Adjust the delay as needed

# Run the main function
if __name__ == "__main__":
    main()
