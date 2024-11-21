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

#  Limits: Absolute duty cycle limits
DUTY_MIN = 2300
DUTY_MAX = 7500
PEN_UP = 2300
PEN_DOWN = 3000

# Function to translate angle to duty cycle (safely within limits)
def translate(angle):
    """
    Converts an angle (0-180 degrees) to a duty cycle within safe limits.
    """
    duty = int((DUTY_MAX - DUTY_MIN) * (angle / 180) + DUTY_MIN)
    return max(DUTY_MIN, min(DUTY_MAX, duty))

# Frank's Contribution: Function to safely set servo duty cycle
def safe_set_servo_duty(servo, duty):
    """
    Set the servo to a specific duty cycle within safe limits.
    """
    if DUTY_MIN <= duty <= DUTY_MAX:
        servo.duty_u16(duty)
    else:
        print(f"Error: Duty cycle {duty} out of range ({DUTY_MIN}-{DUTY_MAX})")

# Frank's Contribution: Function to set servo angle
def set_servo_angle(servo, angle):
    """
    Sets the servo to a specific angle, ensuring safety limits are respected.
    """
    duty = translate(angle)
    safe_set_servo_duty(servo, duty)

# Ziad's Contribution: Function to read potentiometer values
def read_potentiometers():
    """
    Reads the current values from the X and Y potentiometers.
    """
    x_value = x_pot.read_u16()
    y_value = y_pot.read_u16()
    return x_value, y_value

# Ziad's Contribution: Function to debounce the pen control switch
def debounce_switch(pin):
    """
    Debounce the input from a switch to ensure stable readings.
    """
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
    """
    Retrieve and process input data from potentiometers and the pen switch.
    """
    x_value, y_value = read_potentiometers()
    pen_state = debounce_switch(pen_switch)
    return {'x': x_value, 'y': y_value, 'pen': pen_state}

# Cloryel's Contribution: Function to create mock data generator
def generate_mock_data():
    """
    Generate mock input data for testing purposes.
    """
    x_value = random.randint(0, 65535)
    y_value = random.randint(0, 65535)
    pen_state = random.choice([0, 1])
    return {'x': x_value, 'y': y_value, 'pen': pen_state}

# Main function
def main():
    """
    Main function to control the system.
    """
    # Set pen to initial up position
    safe_set_servo_duty(pen_pwm, PEN_UP)
    print("Pen set to UP position.")

    # Main loop to handle inputs
    while True:
        # Generate mock data (Cloryel's part)
        input_data = generate_mock_data()
        print("Input Data:", input_data)

        # Map potentiometer values to angles and apply to servos
        shoulder_angle = (input_data['y'] / 65535) * 180
        elbow_angle = (input_data['x'] / 65535) * 180
        pen_position = PEN_DOWN if input_data['pen'] == 1 else PEN_UP

        set_servo_angle(shoulder_pwm, shoulder_angle)
        set_servo_angle(elbow_pwm, elbow_angle)
        safe_set_servo_duty(pen_pwm, pen_position)

        time.sleep(0.5)

if __name__ == "__main__":
    main()
