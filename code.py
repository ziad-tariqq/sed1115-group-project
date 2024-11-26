# Ziad's Contribution: Handles Input Handling and Processing
from machine import ADC, Pin, PWM
import time
import random  # Needed for Cloryel's mock data generation

# Ziad's Contribution: Initialize ADC for potentiometers
x_pot = ADC(Pin(26))  # X potentiometer on GPIO 26
y_pot = ADC(Pin(27))  # Y potentiometer on GPIO 27

# Ziad's Contribution: Initialize pen control switch
pen_switch = Pin(12, Pin.IN, Pin.PULL_DOWN)  # Pen control switch on GPIO 15

# Frank's Contribution: PWM Configuration and Servo Safety Checks
# Initialize PWM for shoulder, elbow, and pen servos
shoulder_pwm = PWM(Pin(0))  # Shoulder servo on GPIO 0
elbow_pwm = PWM(Pin(1))     # Elbow servo on GPIO 1
pen_pwm = PWM(Pin(2))       # Pen servo on GPIO 2

# Frank's Contribution: Set PWM frequency for servos
shoulder_pwm.freq(50)  # 50 Hz for standard servos
elbow_pwm.freq(50)
pen_pwm.freq(50)

# Frank's Contribution: Limits for duty cycles
DUTY_MIN = 2300
DUTY_MAX = 7500
PEN_UP = 2300
PEN_DOWN = 3000

# Frank's Contribution: Function to translate angle to duty cycle (safely within limits)
def translate(angle: float) -> int:
# Your code here
    pulse_width = 500 + (2500-500) * angle / 180 #Pulse width equation
    duty_cycle = pulse_width / 20000 # 20000 microseconds / 20ms
    duty_u16_value = int(duty_cycle * 65535) #multiply so it is in the pwn cl
    duty_u16_value = max (2300, min(7500, duty_u16_value)) #Clamps down value
    return duty_u16_value

# Frank's Contribution: Function to safely set servo duty cycle
def safe_set_servo_duty(servo, duty):
    #Set the servo to a specific duty cycle within safe limits.
    if DUTY_MIN <= duty <= DUTY_MAX:
        servo.duty_u16(duty)
    else:
        print(f"Error: Duty cycle {duty} out of range ({DUTY_MIN}-{DUTY_MAX})")

# Frank's Contribution: Function to set servo angle
def set_servo_angle(servo, angle):
    #Sets the servo to a specific angle, ensuring safety limits are respected.
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

# Cloryel's Contribution: Mock Data Generation for Testing
def generate_mock_data():
    """
    Generate mock input data for testing purposes.
    """
    x_value = random.randint(0, 65535)
    y_value = random.randint(0, 65535)
    pen_state = random.choice([0, 1])
    return {'x': x_value, 'y': y_value, 'pen': pen_state}

# Jaures' Contribution: Mock Modules and Testing Framework
class MockInputHandlingModule:
    def __init__(self):
        self.potentiometer_value = 512
        self.pen_switch_state = False

    def get_potentiometer(self):
        return self.potentiometer_value

    def get_pen_switch(self):
        return self.pen_switch_state

class MockSignalProcessingModule:
    def map_input_to_angle(self, potentiometer_value):
        return (potentiometer_value / 1023) * 180

class MockServoControlModule:
    def __init__(self):
        self.servo_positions = {'shoulder': 0, 'elbow': 0, 'pen': 0}

    def move_servo(self, servo_name, angle):
        if 0 <= angle <= 180:
            self.servo_positions[servo_name] = angle
        else:
            raise ValueError(f"Invalid angle: {angle}. Must be between 0 and 180.")

    def get_servo_position(self, servo_name):
        return self.servo_positions.get(servo_name, None)

class BrachiographTestingFramework:
    def __init__(self):
        self.input_module = MockInputHandlingModule()
        self.signal_module = MockSignalProcessingModule()
        self.servo_module = MockServoControlModule()

    def test_input_handling(self):
        potentiometer_value = self.input_module.get_potentiometer()
        pen_switch_state = self.input_module.get_pen_switch()
        print(f"Potentiometer Value: {potentiometer_value}, Pen Switch State: {pen_switch_state}")
        assert 0 <= potentiometer_value <= 1023, "Potentiometer value out of range"
        assert isinstance(pen_switch_state, bool), "Pen switch state is not boolean"

    def test_signal_processing(self):
        potentiometer_value = self.input_module.get_potentiometer()
        angle = self.signal_module.map_input_to_angle(potentiometer_value)
        print(f"Mapped Angle: {angle}")
        assert 0 <= angle <= 180, f"Mapped angle {angle} is out of range"

    def test_servo_control(self):
        angle = self.signal_module.map_input_to_angle(self.input_module.get_potentiometer())
        self.servo_module.move_servo('shoulder', angle)
        shoulder_position = self.servo_module.get_servo_position('shoulder')
        print(f"Shoulder Position: {shoulder_position}")
        assert 0 <= shoulder_position <= 180, "Servo movement out of range"

    def run_tests(self):
        print("Running Input Handling Test...")
        self.test_input_handling()

        print("\nRunning Signal Processing Test...")
        self.test_signal_processing()

        print("\nRunning Servo Control Test...")
        self.test_servo_control()

        print("\nAll Tests Passed!")

# Main function for Testing
if __name__ == "__main__":
    test_framework = BrachiographTestingFramework()
    test_framework.run_tests()
