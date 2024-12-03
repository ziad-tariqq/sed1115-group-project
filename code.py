from machine import ADC, Pin, PWM
import math
import time

# ------------------------- Setup and Configuration -------------------------

# Arm length constants
L1, L2 = 155, 155

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

# Inverse Kinematics Function
def inverse_kinematics(x_target, y_target):
    """Compute shoulder (θ1) and elbow (θ2) angles for a given (x_target, y_target) position."""
    r = math.sqrt(x_target**2 + y_target**2)  # Distance from the target

    # Check if the target is reachable
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None, None

    # Calculate the elbow angle (theta2)
    theta2 = math.acos((r**2 - L1**2 - L2**2) / (2 * L1 * L2))

    # Calculate the shoulder angle (theta1)
    theta1 = math.atan2(y_target, x_target) - math.acos((L2**2 + r**2 - L1**2) / (2 * L2 * r))

    return math.degrees(theta1), math.degrees(theta2)  # Convert to degrees for servo control

# Function to translate angle to a duty cycle safely within limits
def translate(angle: float) -> int:
    pulse_width = 500 + (2500 - 500) * angle / 180  # Pulse width equation
    duty_cycle = pulse_width / 20000  # 20000 microseconds / 20ms
    duty_u16_value = int(duty_cycle * 65535)  # Scale to 16-bit range
    duty_u16_value = max(DUTY_MIN, min(DUTY_MAX, duty_u16_value))  # Clamp to safe range
    return duty_u16_value

# Function to safely set the duty cycle of a servo
def safe_set_servo_duty(servo, duty):
    """Sets the servo to a specific duty cycle within the safe range."""
    if DUTY_MIN <= duty <= DUTY_MAX:
        servo.duty_u16(duty)
    else:
        print(f"Error: Duty cycle {duty} is out of range!")

# Function to set servo angle safely
def set_servo_angle(servo, angle):
    """Converts an angle to a duty cycle and applies it to the servo."""
    duty = translate(angle)
    safe_set_servo_duty(servo, duty)

# Function to read potentiometer values
def read_potentiometers():
    """Reads analog values from the X and Y potentiometers."""
    x_value = x_pot.read_u16()  # Read X potentiometer
    y_value = y_pot.read_u16()  # Read Y potentiometer
    return x_value, y_value

# Toggle mechanism for the pen
pen_state = False  # Initial state of the pen (False = Up, True = Down)
last_button_state = 0  # Last recorded state of the button

def handle_pen_toggle():
    """Handles the pen's toggle functionality."""
    global pen_state, last_button_state

    current_button_state = pen_switch.value()
    if current_button_state == 1 and last_button_state == 0:  # Button press detected
        pen_state = not pen_state  # Toggle the pen state
        print(f"Pen State Toggled: {'DOWN' if pen_state else 'UP'}")
        pen_position = PEN_DOWN if pen_state else PEN_UP
        safe_set_servo_duty(pen_pwm, pen_position)

    last_button_state = current_button_state  # Update the last button state

# Testing servo control with inverse kinematics
def test_servo_control():
    """Uses potentiometer inputs as (x, y) coordinates and applies inverse kinematics."""
    # Read potentiometer values
    x_value, y_value = read_potentiometers()

    # Map potentiometer values to target coordinates (-L1-L2 to L1+L2)
    x_target = (x_value / 65535) * (2 * (L1 + L2)) - (L1 + L2)
    y_target = (y_value / 65535) * (2 * (L1 + L2)) - (L1 + L2)

    # Compute inverse kinematics
    theta1, theta2 = inverse_kinematics(x_target, y_target)
    if theta1 is None or theta2 is None:
        print("Target out of reach.")
        return

    # Convert angles to duty cycle and send to servos
    print(f"Setting Shoulder Angle: {theta1:.2f}, Elbow Angle: {theta2:.2f}")
    set_servo_angle(shoulder_pwm, theta1)
    set_servo_angle(elbow_pwm, theta2)

    # Handle pen toggle
    handle_pen_toggle()

# Main function to run the inverse kinematics control
def main():
    """Main entry point to run the robotic arm control."""
    print("Starting Arm Control...")
    while True:
        test_servo_control()
        time.sleep(0.1)

if __name__ == "__main__":
    main()
