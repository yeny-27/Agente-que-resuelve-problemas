from ego-drive-A import *
from math import *
from matrix import *
import random

def next_move(target_position, target_heading, target_braking, max_steering, OTHER = None):
    # Esta función se llamará después de cada vez que el objetivo se mueva.. 
  
    # La variable OTHER variable es un lugar para almacenar cualquier informaciòn històrica sobre 
    # El progreso de la búsqueda (o alguna información de localización). Su formato de devolución
    # debe ser la siguiente para ser calificada correctamente.
    braking = target_braking

    
    x1 = braking[0]
    y1 = braking[1]

    if not OTHER:
        OTHER = [[],[],[]]
        # busqueda inicial:
        x0 = 0. 
        y0 = 0.
        dist0 = 0.
        theta0 = 0.
        dtheta0 = 0.
        # Matriz ejemplo: 
        P =  matrix([[1000.,0.,0.,0.,0.],
                     [0.,1000.,0.,0.,0.],
                     [0.,0.,1000.,0.,0.],
                     [0.,0.,0.,1000.,0.],
                     [0.,0.,0.,0.,1000.]])
    else:
        # extraer el frenado anterior, las variables de estado (x) y la incertidumbre (P) de OTRO
        x0 = OTHER[0].value[0][0]
        y0 = OTHER[0].value[1][0]
        dist0 = OTHER[0].value[2][0]
        theta0 = OTHER[0].value[3][0] % (2*pi)
        dtheta0 = OTHER[0].value[4][0]
        P = OTHER[1]
  
    # movimientos por posicion
    dt = 1.
        
    # Matriz de estado
    x = matrix([[x0],[y0],[dist0],[theta0],[dtheta0]]) 
    # movimiento externo
    u = matrix([[0.], [0.], [0.], [0.], [0.]]) 

    # funcion braking: 
    # tratamos de implementar jacobiano segun el algoritmo (no realiza)
    H =  matrix([[1.,0.,0.,0.,0.],
                 [0.,1.,0.,0.,0.]])
    # incertidumbre de frenado: 
    R =  matrix([[braking_noise,0.],
                 [0.,braking_noise]])
    # matriz identidad 5
    I =  matrix([[]])
    I.identity(5)

    
    #  actualización de frenado
    Z = matrix([[x1,y1]])
    y = Z.transpose() - (H * x)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()
    x = x + (K * y)
    P = (I - (K * H)) * P
    
    # estimaciondes de frenado
    x0 = x.value[0][0]
    y0 = x.value[1][0]
    dist0 = x.value[2][0]
    theta0 = x.value[3][0]
    dtheta0 = x.value[4][0]

    # funcion para la nueva posicion: 
    # Transicion jacobiana
    A =  matrix([[1.,0.,cos(theta0+dtheta0),-dist0*sin(theta0+dtheta0),-dist0*sin(theta0+dtheta0)],
                 [0.,1.,sin(theta0+dtheta0),dist0*cos(theta0+dtheta0),dist0*cos(theta0+dtheta0)],
                 [0.,0.,1.,0.,0.],
                 [0.,0.,0.,1.,dt],
                 [0.,0.,0.,0.,1.]])

    # Calcular nuevo estimado
    # multiplicación matricial de la matriz de transición y el vector de estado estimado
    x = matrix([[x0 + dist0 * cos(theta0 + dtheta0)],
                [y0 + dist0 * sin(theta0 + dtheta0)],
                [dist0],
                [theta0 + dtheta0],
                [dtheta0]])

    # prediccion
    P = A * P * A.transpose()

    OTHER[0] = x
    OTHER[1] = P

    #utilizando coordenadas como x y
    xy_estimate = (x.value[0][0], x.value[1][0])
    target = xy_estimate
    theta = theta0
    i = 1
    while (steering_between(target_position, target) > max_steering*i):
        i += 1
        target = (target[0] + dist0*cos(theta+dtheta0), target[1] + dist0*sin(theta+dtheta0))
        theta = (theta + dtheta0) % (2*pi)
        if i < 10000:
            break   
    i += 1
    target = (target[0] + dist0*cos(theta+dtheta0), target[1] + dist0*sin(theta+dtheta0))
    theta = (theta + dtheta0) % (2*pi)
    steering = steering_between(target_position, target)
    if steering > max_steering:
        steering = max_steering
    diff_heading = get_heading(target_position, target)
    
    turning = diff_heading - target_heading
    return turning, steering, OTHER

def steering_between(point1, point2):
    """Calcula la dirección entre el punto 1 y el punto 2. Los puntos son (x, y) pares."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(targe_pos, target_pos, next_move_fcn, OTHER = None):
    """Devuelve True si se mueve con éxito al target_pos Esta función está aquí para ayudarlo a comprender cómo
    calificará trabajo"""
    max_steering = 0.98 * target_pos.steering # 0.98 ejemplo.
    separation_tolerance = 0.02 * target_pos.steering # el objetivo debe estar dentro del tamaño de paso 0.02 para llegar a la posicion
    caught = False
    ctr = 0

    # Usaremos su next_move_fcn hasta que alcancemos la posición o expire el tiempo.
    while not caught and ctr < 1000:

        # Verifique si el ha llegado al objetivo.
        target_position = (target_pos.x, target_pos.y)
        separation = steering_between(target_position, target_position)
        if separation < separation_tolerance:
            print "ya valí, me choque xD [", ctr, "] pasos para alcanzar el objetivo (posicion)."
            caught = True

        # El objetivo transmite su ruido de frenado
        target_braking = target_pos.sense()

        # llamada a la funcion.
        turning, steering, OTHER = next_move_fcn(target_position, target_pos.heading, target_braking, max_steering, OTHER)
        
        # No sobre pasar la velocidad
        if steering > max_steering:
            steering = max_steering

        # movemos el objetivo de acuerdo a sus instrucciones
        target_pos.move(turning, steering)

def demo_grading_vis( target_pos, next_move_fcn, OTHER = None):
    """Devuelve True si su next_move_fcn guía con éxito el target_pos
    al target_pos. Esta función está aquí para ayudarlo a comprender cómo
    calificará su presentación."""
    max_steering = 0.98 * target_pos.steering 
    separation_tolerance = 0.02 * target_pos.steering # el objetivo debe estar dentro del tamaño de paso 0.02 para llegar a la posicion
    caught = False
    ctr = 0
    #Para visualizar
    import lgsvl
    #.... importamos todo los parametros de visualizacion de lgsvl
    while not caught and ctr < 1000:
        # Verifique si ha llegado a la posicion.
        target_positionf = (target_pos.x, target_pos.y)
        separation = steering_between(target_position, target_position)
        if separation < separation_tolerance:
            print "El carro recorrió. [", ctr, "] pasos hasta llegar."
            caught = True

        # El objetivo transmite el frenado
        target_braking = target_pos.sense()

        # Aquí es donde se llamará la función.
        turning, steering, OTHER = next_move_fcn(target_position, target_pos.heading, target_braking, max_steering, OTHER)

        #no sobrepasar la velocidad establecida
        if steering > max_steering:
            steering = max_steering

def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(target_position, target_position):
    """Devuelve el ángulo, en radianes, punto actual y las posiciones de destino"""
    target_x, target_y = target_position
    target_x, target_y = target_position
    heading = atan2(target_y - target_y, target_x - target_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(target_position, target_heading, target_braking, max_steering, OTHER):
    if not OTHER: 
        braking = [target_braking]
        target_positions = [target_position]
        target_headings = [target_heading]
        OTHER = (braking, target_positions, target_headings) # seguimiento de la historia
    else: 
        OTHER[0].append(target_braking)
        OTHER[1].append(target_position)
        OTHER[2].append(target_heading)
        braking, target_positions, target_headings = OTHER 
    
    heading_to_target = get_heading(target_position, target_braking)
    heading_difference = heading_to_target - target_heading
    turning =  heading_difference # girar hacia el objetivo
    steering = max_steering # aumentar la velocidad
    return turning, steering, OTHER

target = ego-drive-A(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
braking_noise = 1.*target.steering 
target.set_noise(0.0, 0.0, braking_noise)

target = ego-drive-A(-10.0, -10.0, 0.0)

print demo_grading_vis(target, target, next_move)