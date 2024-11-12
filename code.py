from machine import ADC, Pin
import time

# Initialize ADC for potentiometers
x_pot = ADC(Pin(26))  # X potentiometer on GPIO 26
y_pot = ADC(Pin(27))  # Y potentiometer on GPIO 27
pen_switch = Pin(15, Pin.IN, Pin.PULL_DOWN)  # Pen control switch on GPIO 15

# Function to read potentiometer values
def read_potentiometers():
    return x_pot.read_u16(), y_pot.read_u16()  # Return raw ADC values

# Function to debounce the pen control switch
def debounce_switch(pin):
    stable_time = 20  # Debounce delay in ms
    last_state = pin.value()
    last_debounce_time = time.ticks_ms()

    while True:
        current_state = pin.value()
        if current_state != last_state:
            last_debounce_time = time.ticks_ms()
        if time.ticks_diff(time.ticks_ms(), last_debounce_time) > stable_time:
            if current_state != pin.value():
                return current_state
        last_state = current_state

# Function to get processed input data
def get_input_data():
    x_value, y_value = read_potentiometers()  # Get potentiometer readings
    pen_state = debounce_switch(pen_switch)  # Get debounced switch state
    return {'x': x_value, 'y': y_value, 'pen': pen_state}  # Output as dictionary
