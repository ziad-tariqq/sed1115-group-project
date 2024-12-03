from machine import ADC, Pin, PWM
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import random  # For mock data generation

# ------------------------- Setup and Configuration -------------------------

# Ziad's Contribution: Initialize ADC for potentiometers
x_pot = ADC(Pin(26))  # X potentiometer on GPIO 26
y_pot = ADC(Pin(27))  # Y potentiometer on GPIO 27

# Ziad's Contribution: Initialize pen control switch
#pen_switch = Pin(12, Pin.IN, Pin.PULL_DOWN)  # Pen control switch on GPIO 12
pen_switch = Pin(12, Pin.IN)  # Pen control switch on GPIO 12

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

# Arm length constants
L1, L2 = 155, 155

# Forward Kinematics Function
def forward_kinematics(theta1, theta2):
    """Calculate the position of the end effector (x, y) using forward kinematics."""
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y

# Fonction de Cinématique Inverse
def inverse_kinematics(x_target, y_target):
    r = np.sqrt(x_target**2 + y_target**2) # Distance from the target
    
    if r > (L1+L2) or r < abs(L1 - L2): # Checking if the target is reachable
        print("Sorry I can't go that far")
        return None, None
    
    theta2 = np.arccos((r**2 - L1**2 - L2**2) / (2 * L1 * L2)) #Calculate the elbow angle
    theta1 = np.arctan2(y_target, x_target) - np.arccos((L2**2 + r**2 - L1**2) / (2 * L2 * r)) ##Calculate the sholder angle

    return theta1, theta2

#Example of target
x_target = 100
y_target = 200

theta1, theta2 = inverse_kinematics(x_target, y_target)

if theta1 is not None and theta2 is not None:
    print(f"Angles calculés :")
    print(f"Angle de l'épaule (θ1) : {np.degrees(theta1):.2f}°")
    print(f"Angle du coude (θ2) : {np.degrees(theta2):.2f}°")

    # Affichage du bras robotique dans un graphique
    fig, ax = plt.subplots()
    ax.set_xlim(-L1 - L2 - 1, L1 + L2 + 1)
    ax.set_ylim(-L1 - L2 - 1, L1 + L2 + 1)
    
    # Calcul des positions des articulations
    x1 = L1 * np.cos(theta1)  # Position du coude
    y1 = L1 * np.sin(theta1)
    x2 = x1 + L2 * np.cos(theta1 + theta2)  # Position du poignet (effecteur final)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    
    # Tracer le bras et les articulations
    ax.plot([0, x1, x2], [0, y1, y2], 'b-o', label="Bras robotique")
    ax.scatter([x_target], [y_target], color="red", label="Position cible")
    ax.set_title("SED 1115 : Inverse Kinematic_Brachiograph Final")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()
    ax.grid(True)
    
    plt.show()
else:
    print("Impossible d'atteindre la position cible.")

# Frank's Contribution: Function to set servo angle
def set_servo_angle(servo, angle):
    duty = int((angle / 180) * 65535)
    servo.duty_u16(duty)

# Function to translate angle to a duty cycle safely within limits
def translate(angle: float) -> int:
    pulse_width = 500 + (2500-500) * angle / 180 #Pulse width equation
    duty_cycle = pulse_width / 20000 # 20000 microseconds / 20ms
    duty_u16_value = int(duty_cycle * 65535) #multiply so it is in the pwn cl
    duty_u16_value = max (2300, min(7500, duty_u16_value)) #Clamps down value
    return duty_u16_value

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
    duty = translate(angle)
    safe_set_servo_duty(servo, duty)

# Function to read potentiometer values
def read_potentiometers():
  #Reads analog values from the X and Y potentiometers.
    x_value = x_pot.read_u16()  # Read X potentiometer
    y_value = y_pot.read_u16()  # Read Y potentiometer
    print(x_value,y_value)
    return x_value, y_value

# Function to debounce the pen control switch
def debounce_switch(pin):

    #Debounces the input from a switch to ensure stable readings.

    stable_time = 20  # Stabilization time in milliseconds
    last_state = pin.value()  # Read the initial state
    last_debounce_time = time.ticks_ms()  # Record the current time
    
    return last_state

    '''
    while True:
        current_state = pin.value()  # Check the current state
        if current_state != last_state:
            last_debounce_time = time.ticks_ms()  # Reset debounce timer
        if (time.ticks_diff(time.ticks_ms(), last_debounce_time) > stable_time):
            if current_state != pin.value():  # Confirm stable state
                return current_state
        last_state = current_state
    '''

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

def test_servo_control():
   #Reads potentiometer inputs (X and Y) and maps them directly to PWM outputs. Applies the angles to the shoulder and elbow servos.
    # Read potentiometer values
    x_value, y_value = read_potentiometers()
    
    # Map potentiometer values to angles (0 to 180 degrees)
    shoulder_angle = (y_value / 65535) * 180  # Y controls shoulder
    elbow_angle = (x_value / 65535) * 180    # X controls elbow

    # Convert angles to duty cycle and send to servos
    print(f"Setting Shoulder Servo Angle: {shoulder_angle}")
    set_servo_angle(shoulder_pwm, shoulder_angle)
    print(x_value,y_value)

    print(f"Setting Elbow Servo Angle: {elbow_angle}")
    set_servo_angle(elbow_pwm, elbow_angle)

    # Example: Check the pen state and adjust its position
    pen_state = debounce_switch(pen_switch)
    pen_position = PEN_DOWN if pen_state else PEN_UP
    print(f"Setting Pen Position: {'DOWN' if pen_state else 'UP'}")
    safe_set_servo_duty(pen_pwm, pen_position)


# Main function to run tests
def main():
    """
    Main entry point to run the testing framework.
    """
    print("Starting Tests...")
    while True:
        test_input_handling()
        print()
        test_signal_processing()
        test_servo_control()
    print("All Tests Passed!")

if __name__ == "__main__":
    main()
