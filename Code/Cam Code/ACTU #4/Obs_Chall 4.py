from pybricks.hubs import InventorHub
from pybricks.parameters import Button, Axis, Color, Direction, Port, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import Car
from pybricks.tools import wait, StopWatch
from pupremote import PUPRemoteHub
from pybricks.tools import wait, run_task

############ DECLARACION DE EQUIPO NO CONECTADO AL ESP32 ######
inventor_hub = InventorHub(top_side=-Axis.Z, front_side=Axis.X)
pr = PUPRemoteHub(Port.F)

#der = UltrasonicSensor(Port.C)
pfin = 0
motor_traccion = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_direccion = Motor(Port.B, Direction.CLOCKWISE)
car = Car(motor_direccion, motor_traccion, 40)
inventor_hub.imu.reset_heading(0)
print(inventor_hub.battery.voltage())
pr.add_channel("cam","h")
pr.add_channel('dist','hhh')
daniel = 0
probot = 0
sentido = 0

def main():
    izq = dist_izq()
    der = dist_der()
    vueltas = 0
    if der < izq:
        print('Girando counter')
        sentido = 1
        salida_counter()  
    elif izq < der:
        print('Girando clock')
        sentido = 2
        salida_clock()
    while vueltas < 11:
        if sentido == 1:
            navegacion_ccw()
            curvas_ccw()
            vueltas += 1
        elif sentido == 2:
            navegacion()
            curvas()
            vueltas += 1

def giro_con_imu(steering, angulo_obj, vel):
    """
    Full PID control for precise rotation to target angle
    """
    # === PID TUNING PARAMETERS ===
    Kp = 2      # Proportional gain
    Ki = 0          # Integral gain
    Kd = 0.8        # Derivative gain
    
    MIN_SPEED = 200     # Minimum speed when close to target
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
                # Front ultrasonic sensor
                distance = dist_front()
                return distance <= value
            
            elif sensor == 2:
                #Left ultrasonic sensor.
                distance = dist_izq()
                return distance <= value
            
            elif sensor == 3:
                # Right ultrasonic sensor
                distance = dist_der()
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

def move_for_distance(speed,distance,angle):
    wc = 17.5 #Wheel circumference = 17.5 centimeters 
    velocity = wc*(speed/360) #converting degrees per second to centimeters per second, the robot has a 1:1 gear ration. 
    time1 = ((distance/velocity)*0.8) * 100
    time2 = ((distance*0.2)/8.75) * 100
    drive_with_heading_lock(angle,speed,5,time1,stop=2)
    drive_with_heading_lock(angle,180,5,time2)
    car.drive_power(0)

def look_for_block(speed,angle,dist=1000):
    print("Estoy buscando bloques")
    while True:
        global daniel
        # Keep creeping forward until camera sees *something*
        front = dist_front()
        while front > dist:
            front = dist_front()
            drive_with_heading_lock(angle,300,5,100,stop=2)
            color = lectura()
            wait(250)
            color = lectura()
            if color > 0:
                car.drive_speed(0)
                break
            else:
                color = lectura()
                drive_with_heading_lock(angle,300,5,100,stop=2)
        print(lectura())
        break

def dist_izq():
    der, fron, izq = pr.call('dist')
    return izq

def dist_front():
    der, front, izq = pr.call('dist')
    return front

def dist_der():
    der, front, izq = pr.call('dist')           
    return der

def lectura():
    dato = pr.call("cam")
    return dato

def sigue_pared(dist, speed, sensor, direction, value=50, angle=50, p=0):

    if direction == 1:
        fact = -angle
    elif direction == 2:
        fact = angle
    
    x = fact + p
    y = fact + p
    z = 0 + p

    while True:
        if sensor == 3:
            actual = dist_der()
        elif sensor == 1:
            actual = dist_izq()
        elif sensor == 2:
            actual = dist_front()
        error = dist - actual

        if error > value:
            drive_with_heading_lock(x, speed, 5, 250, stop=2)
        elif error < -value:
            drive_with_heading_lock(y, speed, 5, 250, stop=2)
        else:
            drive_with_heading_lock(z, speed, 5, 250, stop=2)
            print("I finished sigue pared")
            print(dist_izq())
            break

def salida_counter():
    global pfin, daniel
    wait(500)
    giro_con_imu(-80, -20, 100)
    giro_con_imu(70,-40,-100)
    giro_con_imu(-70,-70,100)
    color = lectura()
    if color < 15:
        print('Salida_counterR')
        giro_con_imu(60,5,200)
        drive_with_heading_lock(daniel+10,500,1,680)
        giro_con_imu(-40,-90+daniel,300)
        probot = 0
        daniel -= 90.5
    elif color > 15:
        print('salida_counterV')
        giro_con_imu(-65,-110,200)
        drive_with_heading_lock(-120,200,1,300)
        giro_con_imu(60,0,300)
        curva_v_ccw()
        daniel -= 90.5

def salida_bloque():
    if color > 15:
        print('Salida_clockV')
        giro_con_imu(-30,0,300)
        curva_v()
    elif color < 15:
        print('salida_clockR')
        giro_con_imu(40,90,200)
        drive_with_heading_lock(90,200,1,300)
        giro_con_imu(-40,0,300)
        curva_r()
#Counterclockwise movements

def verde_ccw():
    # mirror of old rojo_ccw() shape (front sensor, 55 deg swing)
    global daniel
    giro_con_imu(-40, -55+daniel, 850)
    sigue_pared(300, 300, 2, 1, angle=50, p=daniel)
    print('Termine sigue pared')
    print(dist_izq())
    giro_con_imu(40, 0 + daniel, 500)

def rojo_ccw():
    # mirror of old verde_ccw() shape (izq sensor, 40 deg swing)
    global daniel
    giro_con_imu(40, 40+daniel,400)
    sigue_pared(350,400, 3, 2, angle=40, p=daniel)
    giro_con_imu(-40, daniel,400)

def rojo2_ccw():
    global daniel
    if not daniel in (-360, -720, -1080):
        dist = 300
    else: 
        dist = 500
    drive_with_heading_lock(daniel,400,1,dist)
    giro_con_imu(-50,-90+daniel,400)
    print('rojo2_ccw')
    print(dist_front())

def verde2_ccw():
    global daniel
    drive_with_heading_lock(2+daniel,200,1,910)
    giro_con_imu(-50,-88+daniel,400)
    print('verde_ccw')

def nada_ccw():
    global pfin, daniel
    izq = dist_izq() 
    der = dist_der()
    if izq < der:
        pfin = 2
    elif der < izq:
        pfin = 1
    drive_with_heading_lock(daniel,400,1,1350,stop=2)

def nada2_ccw():
    global daniel, probot
    drive_with_heading_lock(daniel+15,500,1,700)
    giro_con_imu(-45,daniel-90,350)
    print('cvn')
    probot = 3

def navegacion_ccw(): #Aqui asignamos el valor de color a una variable para que no se vaya actualizando con cada condicion que pase
    global pfin, daniel, probot
    color = 0
    color = lectura()
    print("color=",color)
    if not daniel in (-362, -724, -1086):
        if probot == 0: 
            print('probot=',probot)    
            if 0 < color < 15:
                print('vi algo')
                rojo_ccw()
                print('rojo_ccw')
                look_for_block(400,daniel-3)
                car.drive_speed(0)
                wait(300)
                color = lectura()
                if color == 0:
                    nada_ccw()
                    print('nada_ccw')
                    print(color) 
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        drive_with_heading_lock(0 + daniel,500,1,1000)
                        print(color)
                        pfin = 1
                    elif color > 15:
                        giro_con_imu(-50,-90+daniel,400)
                        drive_with_heading_lock(-90+daniel,300,1,350)
                        giro_con_imu(40,daniel-1,400)
                        pfin = 2
            elif color > 15:
                print('vi algo')
                verde_ccw()
                print('verde_ccw')
                look_for_block(400,daniel+1)
                wait(800)
                color = lectura()
                print(color)
                if color == 0:
                    nada_ccw()
                    print('nada_ccw')
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        giro_con_imu(40,90+daniel,400)
                        drive_with_heading_lock(90+daniel,400,1,350)
                        giro_con_imu(-40,0+daniel,400)
                        print('rojo_ccw')
                        pfin = 1
                    elif color > 15:
                        drive_with_heading_lock(daniel-1,400,1,1100)
                        print('verde_ccw')
                        pfin = 2
            elif color == 0:
                print('nada_ccw')
                look_for_block(400,daniel,dist=100)
                car.drive_speed(0)
                wait(200)
                color = lectura()
                if color < 15:
                    giro_con_imu(40, 40+daniel, 500)
                    sigue_pared(300, 500, 3, 2, angle=40, p=daniel)
                    giro_con_imu(-40, daniel, 500)
                    drive_with_heading_lock(daniel,400,1,1100)
                    pfin = 1
                elif color > 15:
                    giro_con_imu(-40, -35+daniel, 400)
                    drive_with_heading_lock(-35+daniel,300,2,320)
                    giro_con_imu(40, 0 + daniel, 500)
                    drive_with_heading_lock(daniel,400,1,1100)
                    pfin = 2
        elif probot == 1:
            print('probot=',probot)
            look_for_block(400,daniel-1)
            if color == 0:
                nada_ccw()
                print('nada_ccw')
                print(color) 
            elif color > 0:
                print('vi algo')
                if color < 15:
                    drive_with_heading_lock(2 + daniel,500,1,1000)
                    print(color)
                    pfin = 1
                elif color > 15:
                    verde_ccw()
                    print('verde_ccw')
                    print(lectura())
                    pfin = 2     
            probot = 2
        elif probot == 2:
            print('probot=',probot)
            look_for_block(400,daniel+5,dist=1000)
            wait(200)
            color = lectura()
            print("Frontal distance=",dist_front())
            if color == 0:
                nada_ccw()
                print('nada_ccw')
            elif color > 0:
                print('vi algo')
                if color < 15:
                        giro_con_imu(40,90+daniel,400)
                        drive_with_heading_lock(90+daniel,400,1,350)
                        giro_con_imu(-40,0+daniel,400)
                        drive_with_heading_lock(daniel-1,400,1,1000,stop=2)
                        wait(350)
                        print('rojo_ccw')
                        pfin = 1
                elif color > 15:
                        drive_with_heading_lock(daniel-1,400,1,1000)
                        pfin = 2
            probot= 0
        elif probot == 3:
            look_for_block(500,daniel)
            car.drive_power(0)
            wait(250)
            color = lectura()
            if 0 < color < 15:
                print('Vi rojo_ccw')
                giro_con_imu(50,45+daniel,300)
                drive_with_heading_lock(45+daniel,300,3,320)
                giro_con_imu(-50,daniel,300)
                drive_with_heading_lock(daniel-2,400,1,950,stop=2)
                pfin = 1
            elif color > 15:
                print('Vi verde_ccw')
                giro_con_imu(-40,-45+daniel,400)
                sigue_pared(350,300,2,1,angle=45,p=daniel)
                giro_con_imu(40,daniel,400)
                pfin = 2
    else:
        print('parking lane')
        if probot == 0:
            if 0 < color < 15:
                print('vi algo')
                giro_con_imu(40,40+daniel,400)
                drive_with_heading_lock(daniel+40,400,3,450)
                giro_con_imu(-40,daniel,400)
                print('rojo_ccw')
                look_for_block(400,daniel-1)
                car.drive_speed(0)
                wait(300)
                color = lectura()
                if color == 0:
                    nada_ccw()
                    print('nada_ccw')
                    print(color) 
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        drive_with_heading_lock(0 + daniel,500,1,1000)
                        print(color)
                        pfin = 1
                    elif color > 15:
                        giro_con_imu(-50,daniel-90,400)
                        drive_with_heading_lock(daniel-90,350,1,360)
                        giro_con_imu(50,daniel+1,400)
                        drive_with_heading_lock(daniel+1,350,1,1100)
                        pfin = 2
            elif color > 15:
                print('vi algo')
                verde_ccw()
                print('verde_ccw')
                look_for_block(400,daniel-0.5)
                car.drive_power(0)
                wait(300)
                color = lectura()
                print(color)
                if color == 0:
                    nada_ccw()
                    print('nada_ccw')
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        giro_con_imu(50,daniel+45,400)
                        move_for_distance(250,340,daniel+45)
                        giro_con_imu(-50,daniel,300)
                        print('rojo_ccw')
                        pfin = 1
                    elif color > 15:
                        drive_with_heading_lock(daniel,400,1,1100)
                        print('verde_ccw')
                        pfin = 2
            elif color == 0:
                print('nada_ccw')
                color = lectura()
                while True:
                    # Keep creeping forward until camera sees *something*
                    while color < 10:
                        drive_with_heading_lock(daniel,300,5,200,stop=2)
                        color = lectura()
                    car.drive_power(0)
                    wait(250)
                    color = lectura()
                    if 0 < color < 15:
                        print('Vi rojo_ccw')
                        giro_con_imu(50,daniel+45,400)
                        move_for_distance(250,290,daniel+45)
                        giro_con_imu(-50,daniel,300)
                        pfin = 1
                        break
                    elif color > 15:
                        print('Vi verde_ccw')
                        drive_with_heading_lock(daniel,400,1,1100)
                        pfin = 2
                        break
                    else:
                        # color == 0 again -- false alarm, lost it, go back to creeping
                        print('perdi el bloque, sigo buscando')
                        # small extra nudge forward so we don't loop-check the exact same spot
                        drive_with_heading_lock(daniel,300,5,200,stop=2)
                        color = lectura()       
        elif probot == 1:
            print('jaja=',probot)
            look_for_block(400,daniel-1)
            if color == 0:
                nada_ccw()
                print('nada_ccw')
                print(color) 
            elif color > 0:
                print('vi algo')
                if color < 15:
                    drive_with_heading_lock(2 + daniel,500,1,1000)
                    print(color)
                    pfin = 1
                elif color > 15:
                    verde_ccw()
                    print('verde_ccw')
                    pfin = 2     
            probot = 2
        elif probot == 2:
            print('probot=',probot)
            look_for_block(400,daniel)
            wait(300)
            color = lectura()
            if color == 0:
                nada_ccw()
                print('nada_ccw')
            elif color > 0:
                print('vi algo')
                if color < 15:
                        drive_with_heading_lock(2+daniel,500,1,1000)
                        print('rojo_ccw')
                        pfin = 1
                elif color > 15:
                        giro_con_imu(-40,-45+daniel,600)
                        move_for_distance(300,370,daniel-45)
                        giro_con_imu(40,0+daniel,500)
                        print('verde_ccw')
                        pfin = 2
            probot= 0
        elif probot == 3:
            color = lectura()
            while True:
                # Keep creeping forward until camera sees *something*
                while color < 10:
                    drive_with_heading_lock(daniel,300,5,200,stop=2)
                    color = lectura()
                car.drive_power(0)
                wait(250)
                color = lectura()
                if 0 < color < 15:
                    print('Vi rojo_ccw')
                    giro_con_imu(50,daniel+40,400)
                    giro_con_imu(-50,daniel,400)
                    drive_with_heading_lock(daniel,400,1,1100)
                    pfin = 1
                    break
                elif color > 15:
                    print('Vi verde_ccw')
                    drive_with_heading_lock(daniel-30,300,2,320)
                    drive_with_heading_lock(daniel,300,1,1100)
                    pfin = 2
                    break
                else:
                    # color == 0 again -- false alarm, lost it, go back to creeping
                    print('perdi el bloque, sigo buscando')
                    # small extra nudge forward so we don't loop-check the exact same spot
                    drive_with_heading_lock(daniel,300,5,200,stop=2)
                    color = lectura()

def curva_r_ccw():
    # now the FIXED, unconditional pivot (was curva_v_ccw's job in CW),
    # mirrored for CCW direction
    global daniel, probot
    if not daniel in (-362, -724, -1086):
        dist = 650
        angle = 0
        steer = -40
    else:
        dist = 660
        angle = 10
        steer = -60
    drive_with_heading_lock(daniel+angle,500,1,dist)
    giro_con_imu(steer,-90+daniel,300)
    probot = 0

def curva_v_ccw():
    # now the branching pivot (was curva_r_ccw's job in CW), including
    # the probot==3 fallback state that lives here since this is now
    # the branching function
    global probot
    wait(500)
    Color = lectura()
    wait(500)
    if 0 < Color < 12:
        probot = 1
        print(Color)
        rojo2_ccw()
    elif Color> 15:
        probot = 2
        print(Color)
        verde2_ccw()
    else:
        print(Color)
        nada2_ccw()  
        probot = 3

def curvas_ccw():
    global daniel, pfin
    print(pfin)
    if pfin == 1:
        print('cvr')
        curva_r_ccw()
        daniel -= 90.5
        print(daniel) 
    elif  pfin ==2:
        print('cvv')
        drive_with_heading_lock(daniel,250,1,975)
        curva_v_ccw()
        daniel -= 90.5
        print(daniel)
################################################################################################################################
###   ####  ########    ######   ####  ##  #####################################################################################
#  #######  #######  ##  ###  #######   ########################################################################################
#  #######  #######  ##  ###  #######    #######################################################################################
###   ####      ####    ######   ####  ##  #####################################################################################
################################################################################################################################
#Clockwise movements

def salida_clock():
    global pfin
    wait(500)
    giro_con_imu(65, 25, 200)
    giro_con_imu(-65,52,-200)
    giro_con_imu(-65,50,200)
    car.drive_speed(0)
    wait(300)
    color = lectura()
    if color > 15:
        print('Salida_clockV')
        car.steer(0)
        drive_with_heading_lock(55,200,5,1050)
        giro_con_imu(-30,0,300)
        pfin = 2
        curvas()
    elif color < 15:
        print('salida_clockR')
        giro_con_imu(40,85,200)
        drive_with_heading_lock(85,200,1,370)
        giro_con_imu(-40,0,300)
        drive_with_heading_lock(-1,300,1,1450)
        wait(100)
        color = lectura()
        if color < 15:
            drive_with_heading_lock(-1,400,1,1100)
            pfin = 1
        elif color > 15:
            verde()
            drive_with_heading_lock(1,400,1,1050)
            pfin = 2
        curvas()

def verde():
    global daniel
    giro_con_imu(-40,-45+daniel,500)
    sigue_pared(325,400,1,1,angle=40, p=daniel)
    giro_con_imu(40,daniel,500)

def rojo():
    global daniel
    giro_con_imu(40,55+daniel,850)
    print('ROJO')
    sigue_pared(300,300,2,2,angle=50,p=daniel)
    print('Termine sigue pared')
    print(dist_der())
    giro_con_imu(-40,0 + daniel,500)

def rojo2():
    global daniel
    drive_with_heading_lock(-2+daniel,500,1,900)
    giro_con_imu(50,87+daniel,400)
    print('rojo')

def verde2():
    global daniel
    if not daniel in (270, 630, 990):
        dist = 350
    else:
        dist = 510
    drive_with_heading_lock(-2+daniel,500,1,dist)
    giro_con_imu(50,90+daniel,400)
    print('verde')

def nada():
    global pfin, daniel
    izq = dist_izq() 
    der = dist_der()
    if der < izq:
        pfin = 1
    elif izq < der:
        pfin = 2
    drive_with_heading_lock(daniel,500,1,1350,stop=2)

def nada2():
    global daniel, probot
    drive_with_heading_lock(daniel-20,500,1,740)
    giro_con_imu(45,daniel+90,350)
    print('cvn')
    probot = 3

def navegacion(): #Aqui asignamos el valor de lectura a una variable para que no se vaya actualizando con cada condicion que pase
    global pfin, daniel, probot
    color = 0
    print(lectura())
    color = lectura()
    if not daniel in (360, 720, 1080):
        if probot == 0: 
            print('jaja=',probot)    
            if 0 < color < 15:
                print('vi algo')
                rojo()
                print('rojo')
                look_for_block(500,daniel+1)
                car.drive_speed(0)
                wait(300)
                color = lectura()
                print(color,'color')
                if color == 0:
                    nada()
                    print('nada')
                    print(color) 
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        drive_with_heading_lock(daniel,300,1,1100)
                        print('doble rojo',color)
                        pfin = 1
                    elif color > 15:
                        giro_con_imu(-50,-90+daniel,400)
                        drive_with_heading_lock(-90+daniel,400,1,350)
                        giro_con_imu(50,daniel,400)
                        print('verde')
                        pfin = 2
            elif color > 15:
                print('vi algo')
                verde()
                print('verde')
                look_for_block(400,daniel+2)
                car.drive_speed(0)
                wait(800)
                color = lectura()
                print(color)
                if color == 0:
                    nada()
                    print('nada')
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        giro_con_imu(50,90+daniel,400)
                        drive_with_heading_lock(daniel+90,400,1,350)
                        giro_con_imu(-50, daniel,400)
                        print('rojo')
                        pfin = 1
                    elif color > 15:
                        drive_with_heading_lock(2+daniel,500,1,1000)
                        print('verde')
                        pfin = 2
            elif color == 0:
                print('nada')
                color = lectura()
                while True:
                    # Keep creeping forward until camera sees *something*
                    while color < 10:
                        drive_with_heading_lock(daniel,300,5,200,stop=2)
                        color = lectura()
                    car.drive_power(0)
                    wait(250)
                    color = lectura()
                    if 0 < color < 15:
                        print('Vi rojo')
                        rojo()
                        pfin = 1
                        break
                    elif color > 15:
                        print('Vi verde')
                        verde()
                        pfin = 2
                        break
                    else:
                        # color == 0 again -- false alarm, lost it, go back to creeping
                        print('perdi el bloque, sigo buscando')
                        # small extra nudge forward so we don't loop-check the exact same spot
                        drive_with_heading_lock(daniel,300,5,200,stop=2)
                        color = lectura()
        elif probot == 1:
            print('jaja=',probot)
            look_for_block(400,daniel)
            wait(800)
            if color == 0:
                nada()
                print('nada')
                print(color) 
            elif color > 0:
                print('vi algo')
                if color < 15:
                    drive_with_heading_lock(-2 + daniel,500,1,1000)
                    print(color)
                    pfin = 1
                    look_for_block(500,daniel)
                elif color > 15:
                    verde()
                    print('verde')
                    print(lectura())
                    pfin = 2     
            probot = 2
        elif probot == 2:
            print('jaja=',probot)
            look_for_block(300,daniel+9)
            wait(200)
            color = lectura()
            if color == 0:
                nada()
                print('nada')
            elif color > 0:
                print('vi algo')
                if 0 <color < 15:
                        giro_con_imu(50,daniel+90,400)
                        drive_with_heading_lock(daniel+90, 400,1,340)
                        giro_con_imu(-50,daniel,400)
                        print('rojo')
                        pfin = 1
                elif color > 15:
                        drive_with_heading_lock(2+ daniel,500,1,1000)
                        print('verde')
                        pfin = 2
                elif color == 0:
                    drive_with_heading_lock(daniel+1,500,1,1050)
                    pfin = 2
            probot= 0
        elif probot == 3:
            color = lectura()
            while True:
                # Keep creeping forward until camera sees *something*
                while color < 10:
                    drive_with_heading_lock(daniel,300,5,200,stop=2)
                    color = lectura()
                car.drive_power(0)
                wait(250)
                color = lectura()
                if 0 < color < 15:
                    print('Vi rojo','jaja3')
                    global daniel
                    giro_con_imu(40,55+daniel,850)
                    drive_with_heading_lock(55+daniel,300,1,315)
                    print('Termine sigue pared')
                    giro_con_imu(-40,0 + daniel,400)
                    pfin = 1
                    break
                elif color > 15:
                    print('Vi verde','jaja3')
                    verde()
                    pfin = 2
                    break
                else:
                    # color == 0 again -- false alarm, lost it, go back to creeping
                    print('perdi el bloque, sigo buscando')
                    # small extra nudge forward so we don't loop-check the exact same spot
                    drive_with_heading_lock(daniel,300,5,200,stop=2)
                    color = lectura()
    else:
        print('parking lane')
        if probot == 0:
            if 0 < color < 15:
                print('vi algo')
                rojo()
                print('rojo')
                drive_with_heading_lock(0+daniel,500,1,1650,stop=2)
                car.drive_speed(0)
                wait(300)
                print(color,'rojo')
                color = lectura()
                if color == 0:
                    nada()
                    print('nada')
                    print(color) 
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        drive_with_heading_lock(0 + daniel,500,1,1000)
                        print(color)
                        pfin = 1
                    elif color > 15:
                        verde()
                        print('verde')
                        print(lectura())
                        pfin = 2
            elif color > 15:
                print('vi algo')
                global daniel
                giro_con_imu(-40,-35+daniel,500)
                sigue_pared(420,500,1,1,angle=40, p=daniel)
                giro_con_imu(40,daniel,500)
                print('verde')
                drive_with_heading_lock(2+daniel,500,1,1650)
                wait(800)
                color = lectura()
                print(color)
                if color == 0:
                    nada()
                    print('nada')
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        giro_con_imu(40,55+daniel,600)
                        sigue_pared(375,300,3,2,angle=50,p=daniel)
                        giro_con_imu(-40,0 + daniel,500)
                        print('rojo')
                        pfin = 1
                    elif color > 15:
                        drive_with_heading_lock(2+daniel,500,1,1000)
                        print('verde')
                        pfin = 2
            elif color == 0:
                print('nada')
                look_for_block(300,daniel,dist=200)
                car.drive_speed(0)
                color = lectura()
                if color < 15:
                    sigue_pared(300,350,3,2)
                    giro_con_imu(-50,daniel,400)
                    drive_with_heading_lock(0+daniel,500,1,1100)
                    pfin = 1
                elif color > 15:
                    giro_con_imu(-30,daniel-30,300)
                    drive_with_heading_lock(daniel-20,400,1,750)
                    giro_con_imu(45,daniel,400)
                    drive_with_heading_lock(daniel,400,1,1100)
                    pfin = 2
        elif probot == 1:
                print('jaja=',probot)
                look_for_block(500,daniel)
                wait(250)
                color = lectura()
                if color == 0:
                    nada()
                    print('nada')
                    print(color) 
                elif color > 0:
                    print('vi algo')
                    if color < 15:
                        drive_with_heading_lock(-2 + daniel,500,1,1000)
                        print(color)
                        pfin = 1
                    elif color > 15:
                        verde()
                        print('verde')
                        print(lectura())
                        pfin = 2     
                probot = 2
        elif probot == 2:
            print('jaja=',probot)
            drive_with_heading_lock(3+daniel,500,1,1550)
            giro_con_imu(50,20+daniel,200)
            wait(200)
            color = lectura()
            if color == 0:
                nada()
                print('nada')
            elif color > 0:
                print('vi algo')
                if color < 15:
                        giro_con_imu(40,55+daniel,600)
                        sigue_pared(375,300,3,2,angle=50,p=daniel)
                        print(dist_der())
                        giro_con_imu(-40,0 + daniel,500)
                        print('rojo')
                        pfin = 1
                elif color > 15:
                    drive_with_heading_lock(0,300,5,500)
                    drive_with_heading_lock(2+ daniel,500,1,1000)
                    print('verde')
                    pfin = 2
                    probot= 0
        elif probot == 3:
            look_for_block(300,daniel,dist=200)
            car.drive_speed(0)
            color = lectura()
            if color < 15:
                sigue_pared(300,350,3,2)
                drive_with_heading_lock(0+daniel,500,1,1100)
            elif color > 15:
                giro_con_imu(-30,daniel-30,300)
                drive_with_heading_lock(daniel-20,400,1,750)
                drive_with_heading_lock(daniel,400,1,1100)

def curva_r():
    global probot
    wait(500)
    Color = lectura()
    wait(500)
    if 0 < Color < 15:
        probot = 1
        print(Color)
        rojo2()
    elif Color> 15:
        probot = 2
        print(Color)
        verde2()
    else:
        print(Color)
        nada2()  
        probot = 3

def curva_v():
    global daniel, probot
    print(dist_front())
    drive_with_heading_lock(daniel,500,1,700)
    giro_con_imu(40,88+daniel,300)
    car.drive_speed(200)
    wait(250)
    car.drive_speed(0)
    probot = 0

def curvas():
    global daniel, pfin
    print(pfin)
    if pfin == 1:
        print('cvr')
        drive_with_heading_lock(daniel,300,1,975)
        car.drive_power(0)
        print('pare')
        curva_r()
        daniel += 90
        print(daniel)

    elif  pfin ==2:
        print('cvv')
        curva_v()
        daniel += 90
        print(daniel)
car.drive_power(0)
car.steer(0)
wait(1000)

main()

