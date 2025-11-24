import math

class Motor:
    def __init__(self):
        self.speed = 0
        self.prev_error = 0
        self.integral = 0
        self.target = 0
        self.control = 0
        self.duty_cycle = 0
    
    def pid_controller(self, target, current_speed, dt, kp, ki, kd):
        error = target - current_speed
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        control = kp * error + ki * self.integral + kd * derivative
        self.prev_error = error
        self.target = target
        self.control = control
        self.speed = current_speed
        return control
    
    def run(self):
        # TODO: Rotate at self.duty_cycle
        pass

# TODO: Add dynamic yaw correction. Ensure global movement vector is maintained.
def move(direction, speed, rotation, motors, dt, diameter, kp, ki, kd): # degrees, mm/s
    direction -= 45
    a_mult = math.sin(math.radians(direction))
    b_mult = math.cos(math.radians(direction))
    c_mult = -math.sin(math.radians(direction))
    d_mult = -math.cos(math.radians(direction))

    # Values in mm/s
    a_value = int(a_mult * speed)
    b_value = int(b_mult * speed)
    c_value = int(c_mult * speed)
    d_value = int(d_mult * speed)

    # Values in rpm
    motors[0].target = a_value / (diameter * math.pi) * 60
    motors[1].target = b_value / (diameter * math.pi) * 60
    motors[2].target = c_value / (diameter * math.pi) * 60
    motors[3].target = d_value / (diameter * math.pi) * 60

    # Assuming motors is a list/tuple of 4 Motor objects [a, b, c, d]
    for motor in motors:
        control = motor.pid_controller(motor.target, motor.speed, dt, kp, ki, kd)
        motor.duty_cycle += control
        motor.duty_cycle = max(0, min(100, motor.duty_cycle))
        motor.run()
