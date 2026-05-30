from pybricks.hubs import InventorHub
from pybricks.parameters import Button, Axis, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import Car
from pybricks.tools import wait, StopWatch
from pupremote import PUPRemoteHub
from pybricks.tools import wait, run_task

############ DECLARACION DE EQUIPO NO CONECTADO AL ESP32 ######
inventor_hub = InventorHub(top_side=-Axis.Z, front_side=Axis.X)
pr = PUPRemoteHub(Port.D)

#der = UltrasonicSensor(Port.C)

motor_traccion = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_direccion = Motor(Port.B, Direction.CLOCKWISE)
car = Car(motor_direccion, motor_traccion, 40)
inventor_hub.imu.reset_heading(0)
print(inventor_hub.battery.voltage())
pr.add_channel("cam","h")
pr.add_channel('dist','hhh')
def giro_con_imu(steering, angulo_obj, vel):
    """
    Full PID control for precise rotation to target angle
    """
    # === PID TUNING PARAMETERS ===
    Kp = 2      # Proportional gain
    Ki = 0          # Integral gain
    Kd = 0.8        # Derivative gain
    
    MIN_SPEED = 100       # Minimum speed when close to target
    MAX_SPEED = abs(vel)  # Maximum speed
    TOLERANCE = 2         # Stop when within ±2 degrees
    MAX_INTEGRAL = 100    # Anti-windup limit for integral term
    DEADBAND = 1     # Dead zone to prevent oscillation
    # ============================
    
    # PID state variables
    integral = 0
    last_error = 0
    stopwatch = StopWatch()
    last_time = 0
    
    car.steer(steering)
    
    while True:
        # Get current time and calculate dt
        current_time = stopwatch.time()
        dt = (current_time - last_time) / 1000.0  # Convert to seconds
        if dt < 0.001:  # Prevent division by zero
            dt = 0.01
        last_time = current_time
        
        # Calculate error
        current_heading = inventor_hub.imu.heading()
        error = angulo_obj - current_heading
        
        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # Check if within tolerance
        if abs(error) < TOLERANCE:
            break
        
        # Calculate PID terms
        if abs(error) < DEADBAND:
            # Within deadband - reset integral and set speed to 0
            current_speed = 0
            integral = 0
        else:
            # Proportional term (proportional to error)
            p_term = Kp * error
            
            # Integral term (with anti-windup)
            integral += error * dt
            integral = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, integral))
            i_term = Ki * integral
            
            # Derivative term (rate of change of error)
            derivative = (error - last_error) / dt
            d_term = Kd * derivative
            
            # Combined PID output (this is our correction factor, not final speed)
            pid_output = p_term + i_term + d_term
            
            # Convert PID output to speed
            # The PID output tells us how aggressively to turn
            # We scale this to our speed range (MIN_SPEED to MAX_SPEED)
            
            # Normalize pid_output as a factor of the error
            # Larger errors should give speeds closer to MAX_SPEED
            speed_factor = abs(pid_output) / (Kp * 180)  # 180 is max possible error
            speed_factor = max(0, min(1, speed_factor))  # Clamp to [0, 1]
            
            # Calculate speed based on factor
            current_speed = int(MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor)
            
            # Apply direction based on error sign
            if error < 0:
                current_speed = current_speed
            
            # Apply velocity direction preference
            if vel < 0:
                current_speed = -abs(current_speed)
        
        last_error = error
        
        # Apply speed
        car.drive_speed(current_speed)
        wait(10)
    
    # Stop the robot
    car.drive_power(0)
    car.steer(0)
    wait(50)
def drive_with_heading_lock(target_heading, drive_speed, sensor, value, confirmations=5, stop=1):
    # PID Constants
    Kp = 2.2
    Ki = 0.1
    Kd = 1.5
    MAX_STEER = 50
    DEADBAND = 0.2
    MAX_INTEGRAL = 50
    
    # Define your two target colors here
    TARGET_COLOR_1 = Color.WHITE  # Change to your first color
    TARGET_COLOR_2 = Color.BLACK  # Change to your second color
    
    # PID state
    integral = 0
    last_error = 0
    start_time = StopWatch()
    last_time = 0
    
    # Confirmation counter for sensor readings
    confirma = 0
    
    def check_stop_condition():
        """Check if stop condition is met based on sensor type"""
        try:
            if sensor == 1:
                # Left ultrasonic sensor
                distance = dist_front()
                return distance <= value
            
            elif sensor == 2:
                # Front ultrasonic sensor (ESP32 channel or direct sensor)
                # If using ESP32: distance = pr.channel('dists')[0]  # or appropriate index
                # For now using direct sensor (add if you have one):
                # distance = sensor_front.distance()
                # return distance <= value
                return False  # Replace with actual sensor check
            
            elif sensor == 3:
                # Right ultrasonic sensor
                distance = sensor_der.distance()
                return distance <= value
            
            elif sensor == 4:
                # Color sensor
                detected_color = color_sensor.color()
                if value == 1:
                    return detected_color == TARGET_COLOR_1
                elif value == 2:
                    return detected_color == TARGET_COLOR_2
                return False
            
            elif sensor == 5:
                # Time-based (no confirmations needed)
                return start_time.time() >= value
            
        except:
            return False
        
        return False
    
    while True:
        # Check stop condition with confirmation counter
        if check_stop_condition():
            confirma += 1
            if confirma >= confirmations or sensor == 5 or sensor == 4:  # Time doesn't need confirmations
                break
        else:
            # Reset counter if condition is not met
            confirma = 0
        
        current_time = start_time.time()
        dt = (current_time - last_time) / 1000.0
        if dt < 0.001:
            dt = 0.01
        last_time = current_time
        
        current_heading = inventor_hub.imu.heading()
        error = target_heading - current_heading
        
        # Normalize error to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        
        # PID control for steering
        if abs(error) < DEADBAND:
            steer_correction = 0
            integral = 0
        else:
            p_term = Kp * error
            integral += error * dt
            integral = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, integral))
            i_term = Ki * integral
            derivative = (error - last_error) / dt
            d_term = Kd * derivative
            steer_correction = p_term + i_term + d_term
            steer_correction = int(max(-MAX_STEER, min(MAX_STEER, steer_correction)))
        
        last_error = error
        
        car.drive_speed(drive_speed)
        car.steer(steer_correction)
        wait(10)
    
    # Stop smoothly
    if stop == 1:
        car.drive_power(0)
        car.steer(0)
        wait(50)
    elif stop == 2:
        car.steer(0)
        wait(50)
def dist_izq():
    der, fron, izq = pr.call('dist')
    return izq
def dist_front():
    der, front, izq = pr.call('dist')
    return front
def dist_der():
    der, front, izq = pr.call('dist')           
    return der
def sigue_pared1(dist,value,speed):
    p=0
    esquinas=0
    while esquinas < 12:
        x=-12 + p
        y=12 + p
        z=0 + p
        actual = dist_der()
        error = dist - actual
        if actual < 1000:
            if error > value:
                drive_with_heading_lock(x,speed,5,250,stop=2)
            elif error < -value:
                drive_with_heading_lock(y,speed,5,250,stop=2)
            else:
                drive_with_heading_lock(z,speed,5,250,stop=2)
        elif actual > 1000:
            lol = dist_front()
            if lol > 900:
                wait(500)
                print(lol)
            confirma = 0
            p+=90
            giro_con_imu(60,p,1000)
            esquinas += 1
            wait(1000)
            print(esquinas)
            car.drive_speed(speed)
            wait(700)
            while not confirma >= 5:
                derr = dist_der()
                while not derr < 1000:
                    derr = dist_der()
                    wait(10)
                confirma += 1
    drive_with_heading_lock(p,500,1,1400,5,1)

def sigue_pared2(dist,value,speed):
    p=0
    esquinas=0
    lol = dist_front()
    while esquinas < 12:
        x=16 + p
        y=-16 + p
        z=0 + p
        actual = dist_izq()
        error = dist - actual
        if actual < 1000:
            if error > value:
                drive_with_heading_lock(x,speed,5,250,stop=2)
            elif error < -value:
                drive_with_heading_lock(y,speed,5,250,stop=2)
            else:
                drive_with_heading_lock(z,speed,5,250,stop=2)
        elif actual > 1000:
            lol = dist_front()
            if lol > 900:
                wait(500) 
                print(lol)
            confirma = 0
            p+=-90
            print("MAD")
            giro_con_imu(-60,p,1000)
            print("paso")
            esquinas += 1
            wait(1000)
            print(esquinas)
            car.drive_speed(speed)
            wait(700)
            while not confirma >= 5:
                izqq = dist_izq()
                while not izqq < 1000:
                    izqq = dist_izq()
                    wait(10)
                confirma += 1
    drive_with_heading_lock(p,500,1,1400,5,1)
def main():
    wait(500)
    car.drive_speed(500)
    car.steer(-4)
    while not (dist_der() > 1000 or dist_izq() > 1000):
        wait(10)

    if dist_der() > 1000:
        print("girando clock")
        sigue_pared1(150, 50, 700)
    elif dist_izq() > 1000:
        print("girando counter")
        sigue_pared2(150, 50, 700)
    else:
        car.drive_speed(500)

main()
wait(500)
print(inventor_hub.imu.heading())
wait(500)

