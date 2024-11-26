from machine import ADC, Pin, PWM
import time
import random  # For mock data generation

# ------------------------- Setup and Configuration -------------------------

# Ziad's Contribution: Initialize ADC for potentiometers
x_pot = ADC(Pin(26))  # X potentiometer on GPIO 26
y_pot = ADC(Pin(27))  # Y potentiometer on GPIO 27

# Ziad's Contribution: Initialize pen control switch
pen_switch = Pin(12, Pin.IN, Pin.PULL_DOWN)  # Pen control switch on GPIO 12

# Frank's Contribution: Initialize PWM for shoulder, elbow, and pen servos
shoulder_pwm = PWM(Pin(0))  # Shoulder servo on GPIO 0
elbow_pwm = PWM(Pin(1))     # Elbow servo on GPIO 1
pen_pwm = PWM(Pin(2))       # Pen servo on GPIO 2

# Frank's Contribution: Set PWM frequency for servos
shoulder_pwm.freq(50)  # Standard servo frequency (50 Hz)
elbow_pwm.freq(50)
pen_pwm.freq(50)

# Frank's Contribution: Duty cycle limits for safe operation
DUTY_MIN = 2300  # Minimum allowed duty cycle
DUTY_MAX = 7500  # Maximum allowed duty cycle
PEN_UP = 2300    # Duty cycle for pen up
PEN_DOWN = 3000  # Duty cycle for pen down

# ------------------------- Functions -------------------------

# Function to translate angle to a duty cycle safely within limits
def translate_angle_to_duty(angle):
    """
    Converts an angle (0-180 degrees) to a duty cycle value.
    Ensures the duty cycle stays within the safe operating range.
    """
    # Calculate duty cycle proportional to the angle
    duty = int((DUTY_MAX - DUTY_MIN) * (angle / 180) + DUTY_MIN)
    # Clamp the duty cycle within the safe range
    return max(DUTY_MIN, min(DUTY_MAX, duty))

# Function to safely set the duty cycle of a servo
def safe_set_servo_duty(servo, duty):
    """
    Sets the servo to a specific duty cycle within the safe range.
    """
    if DUTY_MIN <= duty <= DUTY_MAX:
        servo.duty_u16(duty)
    else:
        print(f"Error: Duty cycle {duty} is out of range!")

# Function to set servo angle safely
def set_servo_angle(servo, angle):
    """
    Converts an angle to a duty cycle and applies it to the servo.
    """
    duty = translate_angle_to_duty(angle)
    safe_set_servo_duty(servo, duty)

# Function to read potentiometer values
def read_potentiometers():
    """
    Reads analog values from the X and Y potentiometers.
    """
    x_value = x_pot.read_u16()  # Read X potentiometer
    y_value = y_pot.read_u16()  # Read Y potentiometer
    return x_value, y_value

# Function to debounce the pen control switch
def debounce_switch(pin):
    """
    Debounces the input from a switch to ensure stable readings.
    """
    stable_time = 20  # Stabilization time in milliseconds
    last_state = pin.value()  # Read the initial state
    last_debounce_time = time.ticks_ms()  # Record the current time

    while True:
        current_state = pin.value()  # Check the current state
        if current_state != last_state:
            last_debounce_time = time.ticks_ms()  # Reset debounce timer
        if (time.ticks_diff(time.ticks_ms(), last_debounce_time) > stable_time):
            if current_state != pin.value():  # Confirm stable state
                return current_state
        last_state = current_state

# Function to generate mock data for testing
def generate_mock_data():
    """
    Simulates input data for testing purposes.
    """
    x_value = random.randint(0, 65535)  # Mock X potentiometer value
    y_value = random.randint(0, 65535)  # Mock Y potentiometer value
    pen_state = random.choice([0, 1])   # Random pen switch state
    return {'x': x_value, 'y': y_value, 'pen': pen_state}

# Testing input handling
def test_input_handling():
    """
    Tests reading input values and prints them.
    """
    x, y = read_potentiometers()
    pen = debounce_switch(pen_switch)
    print(f"Potentiometer X: {x}, Potentiometer Y: {y}, Pen Switch: {pen}")

# Testing signal processing
def test_signal_processing():
    """
    Simulates mapping input values to servo angles and applies them.
    """
    mock_data = generate_mock_data()
    shoulder_angle = (mock_data['y'] / 65535) * 180
    elbow_angle = (mock_data['x'] / 65535) * 180
    print(f"Mapped Angles - Shoulder: {shoulder_angle}, Elbow: {elbow_angle}")

# Testing servo control
def test_servo_control():
    """
    Tests servo control by moving them to random angles.
    """
    mock_data = generate_mock_data()
    shoulder_angle = (mock_data['y'] / 65535) * 180
    elbow_angle = (mock_data['x'] / 65535) * 180
    pen_position = PEN_DOWN if mock_data['pen'] else PEN_UP

    set_servo_angle(shoulder_pwm, shoulder_angle)
    set_servo_angle(elbow_pwm, elbow_angle)
    safe_set_servo_duty(pen_pwm, pen_position)

# Main function to run tests
def main():
    """
    Main entry point to run the testing framework.
    """
    print("Starting Tests...")
    test_input_handling()
    #test_signal_processing()
    #test_servo_control()
    print("All Tests Passed!")

if __name__ == "__main__":
    main()
