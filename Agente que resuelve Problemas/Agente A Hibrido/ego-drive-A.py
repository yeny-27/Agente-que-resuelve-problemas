#!/usr/bin/env python3
# 
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import os
import lgsvl
import random
import time
import math

random.seed(0)
# El simulador recibe la interfaz de conexión 
sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
#carga el asset con el mapa deseado
if sim.current_scene == "SanFrancisco":
  sim.reset()
else:
  sim.load("SanFrancisco")

# nodo inicial
spawns = sim.get_spawn()

# estado del agente 
state = lgsvl.AgentState()
state.transform = spawns[0]
a = sim.add_agent("Lexus2016RXHybrid (Autoware)", lgsvl.AgentType.EGO, state)

forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])
r = spawns[0]
print("Default transform: {}".format(tr))

# Funcion que convierte las coordenadas del simulador a coordenadas reales gps
# GPS
gps = sim.map_to_gps(tr)
print("GPS coordinates: {}".format(gps))

# toma los puntos de longitud/lat o  norte/este y los devuelve en coordenadas reales 
# altitud y norte son ocionales
t1 = sim.map_from_gps(northing = gps.northing, easting = -gps.easting, altitude = gps.altitude, orientation = gps.orientation)
print("Transforma coordenas desde Norte/Este: {}".format(t1))

t2 = sim.map_from_gps(latitude = 10.127656, longitude = -4.492843)
print("TTransforma desde longitud/latitud sin contar altitud/orientacion: {}".format(t2))


######Preprocesador de ambiente
#acturadores
def __init__(self, x = 0.0, y = 0.0, throttle= 0.0, steering = 2*pi/10, braking= 1.0):
          self.x = x
          self.y = y
          def throttle = 0
          def steering = 0
          def braking = 0

def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """Esto nos permite cambiar los parámetros , que pueden ser muy
        útil cuando se usan filtros de choque."""
        self.throttle_noise    = float(new_t_noise)
        self.steering_noise    = float(new_d_noise)
        self.braking_noise = float(new_m_noise)

 def move(self, throttle, steering, braking = 0.001, max_turning_angle = pi):
        """Esta función gira el automovil y luego lo mueve hacia adelante."""
        # aplica parametros del sensor
        # and distance_noise es cero.
        throttle = random.gauss(throttle, self.throttle_noise)
        steering = random.gauss(steering, self.steering_noise)

        # especifica las restricciones físicas del vehiculo
        throttle = max(-max_throttle_angle, throttle)
        throttle = min( max_throttle_angle, throttle)
        steering = max(0.0, steering)

        # Ejecutar movimiento
        self.heading += throttle
        self.heading = angle_trunc(self.braking)
        self.x += distance * cos(self.braking)
        self.y += distance * sin(self.braking)

   
    def __repr__(self):
        """Esto no permite conocer la posicion"""
        return '[%.5f, %.5f]'  % (self.x, self.y)
"""
# Control del Vehiculo = se utilizan los detallados en EGO
# Sigue los parametros de los controles del agente
c = lgsvl.VehicleControl()
c.throttle = 1
c.steering = 0
c.braking = 0.5
"""
# True significa que el control se aplicará continuamente. Falso significa que el se manejara manual.
a.apply_control(c, True)

input("Presiona Enter para reiniciar simulacion")
sim.reset()

