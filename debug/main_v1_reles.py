"""
-------------------------------------------------------------------------------------------------------------------------------------------------------------
Script:     main.py
Author:     NEUROGENESIS - Adaptacion David Ponce
Date:       16/10/2022
Release:    v12
-------------------------------------------------------------------------------------------------------------------------------------------------------------
Function:   Este script integra las funcionalidades de todos los dispositivos sensores, actuadores y equipos de backup 
            que conforman el ROBOT ACUAMATTIC.
-------------------------------------------------------------------------------------------------------------------------------------------------------------
"""
# ------------------------------------------------------------- Librerias requeridas ----------------------------------------------------------------------- #
import time, machine
# Manejo de pines de entrada y salida - Control Analogo Digital
from machine import ADC, Pin, UART, I2C
# Control de driver de temperatura
import onewire, ds18x20
# Librerias para el manejo de modulo ESP
import network
from simple import MQTTClient
import urequests as requests
import json
# Libreria para el uso de hilos
import _thread
# Libreria propia para el manejo de los drivers del motor 
import PCA9685 as motor
# Libreria para la obtencion de datos de temperatura y humedad DHT 22
import dht


# -------------------------------------------------------------  Declaracion Pines Salida ------------------------------------------------------------------ #

rl_vent_nucleo  = Pin(1, mode=Pin.OUT)                              # Pin de salida para la activacion de los ventiladores del nucleo                             # Pin de salida para la activacion del relay 
rl_vent_exterior= Pin(17, mode=Pin.OUT)                             # Pin de salida para la activacion de los ventilador exterior
seleneoide_pin  = Pin(8, mode=Pin.OUT)                              # Pin de salida para la activacion de la valvula selenoide
led_pin         = Pin(25,mode=Pin.OUT)                              # Pin de salida de indicador led 

#Relays Normalmente Abierto
reley_NA_1     = Pin(3, mode=Pin.OUT)                              # Pin de salida de reley NA de dispositivo XXXX
reley_NA_2     = Pin(4, mode=Pin.OUT)                              # Pin de salida de reley NA de dispositivo XXXX
reley_NA_3     = Pin(5, mode=Pin.OUT)                              # Pin de salida de reley NA de dispositivo XXXX
reley_NA_4     = Pin(8, mode=Pin.OUT)                              # Pin de salida de reley NA de dispositivo XXXX
reley_NA_5     = Pin(9, mode=Pin.OUT)                              # Pin de salida de reley NA de dispositivo XXXX
reley_NA_6     = Pin(10, mode=Pin.OUT)                             # Pin de salida de reley NA de dispositivo XXXX
reley_NA_7     = Pin(11, mode=Pin.OUT)                             # Pin de salida de reley NA de dispositivo XXXX
reley_NA_8     = Pin(13, mode=Pin.OUT)                             # Pin de salida de reley NA de dispositivo XXXX

# Relays Normalmente Cerrado
reley_NC_1     = Pin(16, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
# reley_NC_2     = Pin(17, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
reley_NC_3     = Pin(18, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
reley_NC_4     = Pin(19, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
led = machine.Pin("LED", machine.Pin.OUT)


def apagar_reles():
    reley_NC_1.off()
    rl_vent_exterior.off()
    reley_NC_3.off()
    reley_NC_4.off()
    
    reley_NA_1.off()
    reley_NA_2.off()
    reley_NA_3.off()
    reley_NA_4.off()
    reley_NA_5.off()
    reley_NA_6.off()
    reley_NA_7.off()
    reley_NA_8.off()

# Funcion principal del core del Robot
def execution_core():
    """
    Funcion utilizada la ejecucion de todos los componentes: sensores, actuadores y 
    equipos de backup.
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    i = 0
    reles = [reley_NA_1,reley_NA_2, rl_vent_exterior, reley_NA_3, reley_NA_4, reley_NA_5, reley_NA_6, reley_NA_7, reley_NA_8, reley_NC_1, reley_NC_3, reley_NC_4] 
    while True:
        if(i<len(reles)):
            reles[i].on()
            
        elif(i == len(reles)):
            apagar_reles()
            
        i = i + 1
        time.sleep_ms(1000)
            
            
                  
execution_core()    