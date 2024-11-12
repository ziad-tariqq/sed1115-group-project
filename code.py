from machine import ADC, Pin, PWM
import time

# Initialize ADC for potentiometers
x_pot = ADC(Pin(26))  # X potentiometer on GPIO 26
y_pot = ADC(Pin(27))  # Y potentiometer on GPIO 27

# Initialize pen control switch
pen_switch = Pin(15, Pin.IN, Pin.PULL_DOWN)  # Pen control switch on GPIO 15

# Initialize PWM for shoulder and elbow servos
shoulder_pwm = PWM(Pin(14))  # Shoulder servo on GPIO 14
elbow_pwm = PWM(Pin(13))     # Elbow servo on GPIO 13

# Set PWM frequency for servos
shoulder_pwm.freq(50)  # 50 Hz for standard servos
elbow_pwm.freq(50)

# Function to calibrate a joint and determine its movement boundaries
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

# Function to read potentiometer values
def read_potentiometers():
    x_value = x_pot.read_u16()  # Returns a value between 0 and 65535
    y_value = y_pot.read_u16()
    return x_value, y_value

# Function to debounce the pen control switch
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

# Function to get processed input data
def get_input_data():
    x_value, y_value = read_potentiometers()  # Get potentiometer readings
    pen_state = debounce_switch(pen_switch)   # Get debounced switch state
    return {'x': x_value, 'y': y_value, 'pen': pen_state}  # Output as dictionary

# Main function
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
        input_data = get_input_data()
        print("Input Data:", input_data)
        # Add code here to process input_data and control servos accordingly
        time.sleep(0.1)  # Adjust the delay as needed

# Run the main function
if __name__ == "__main__":
    main()
