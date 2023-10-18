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

# -------------------------------------------------------------  Declaracion Pines Entrada ----------------------------------------------------------------- #

temp_acuario_pin= Pin(22, mode=Pin.IN)                              # Pin de entrada digital para la lectura del sensor de temperatura
temp_hum_nuc_pin= Pin(0, mode=Pin.IN)                               # Pin de entrada digital para la lectura de temperatura y humedad nucleo
temp_hum_rob_pin= Pin(20, mode=Pin.IN)                              # Pin de entrada digital para la lectura de temperatura y humedad robot
temp_hum_amb_pin= Pin(21, mode=Pin.IN)                              # Pin de entrada digital para la lectura de temperatura y humedad ambiente


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
    
# Funcion utilizada para la lectura del sensor de luz 
def sensor_temperatura(arreglo_roms, sensor_temp):
    """
    Funcion utilizada para extraer la los valores del sensor de temperatura ds18b20
        
    Parameters
    ----------
    arreglo_roms:                   arreglo de lectura de sensor de temperatura
    sensor_temp:                    variable instanciada por la libreria onewire
        
    Returns
    -------
    temp_sensor_temperatura:       valor del sensor de temperatura
    """
    temp_sensor_temperatura = ""
    sensor_temp.convert_temp()
    time.sleep_ms(200)
    if (len(arreglo_roms) > 0):
        for rom in arreglo_roms:
            print(f"El valor de la temperatura es: {sensor_temp.read_temp(rom)}")
            temp_sensor_temperatura = sensor_temp.read_temp(rom)
    time.sleep_ms(200)
    return temp_sensor_temperatura

# Ventiladores del core - encender
def encender_ventiladores_nucleo():
    """
    Funcion utilizada para el control de los ventiladores - encender  
        
    Parameters
    ----------
    N/Aparametro
        
    Returns
    -------
    N/A
    """
    
    print("Ventiladores Encendidos")
    rl_vent_nucleo.on()

# Ventiladores del core - apagar   
def apagar_ventiladores_nucleo():
    """
    Funcion utilizada para el control de los ventiladores - apagar 
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    print("Ventiladores Apagados")
    rl_vent_nucleo.off()
    
# Funcion para la lectura interna de la raspberry pi pico
def temperatura_pico(sensor_temp, conversion_factor):
    """
    Funcion utilizada para extraer la temepratura de la raspberry pi pico 

    Parameters
    ----------
    sesor_temp:         pin de lectura raspberry pi pico 
    conversio_factor:   constante para el calculo de la temperatura de la raspberry pi pico
        
    Returns
    -------
    temperatura:        valor de la temperatura de la raspberry pi pico
    """
    reading = sensor_temp.read_u16() * conversion_factor 
    temperatura = 27 - (reading - 0.706)/0.001721
    print(f"Temperatura: {temperatura}")
    
    # Control de temperatura 
    if(temperatura > 65):
        encender_ventiladores_nucleo()
        
    if(temperatura < 45):
        apagar_ventiladores_nucleo()
        
    time.sleep_ms(200)
    return temperatura

# Funcion para la obtencion de temperatura y humedad DHT22
def obtener_temp_hum(sensor_temp_hum):
    temperatura = ""
    humedad = ""
    time.sleep_ms(100)
    try:
        sensor_temp_hum.measure()
        temperatura = sensor_temp_hum.temperature()
        humedad = sensor_temp_hum.humidity()
        print(f"Temperatura: {temperatura}")
        print(f"Humedad: {humedad}")
    except:
        pass
    
    return temperatura, humedad

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
    
    # -------------------------------------------------------------  Configuracion Sensor de Temperatura -------------------------------------------------------------------------- #
    ds_sensor_temperatura = ds18x20.DS18X20(onewire.OneWire(temp_acuario_pin))
    roms = ds_sensor_temperatura.scan()
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    
    # -------------------------------------------------------------  Configuracion Sensor Temperatura y Humedad ------------------------------------------------------------------- #
    # Creacion del objeto para la lectura de temperatura y humedad del nucleo
    obj_temp_hum_nucleo = dht.DHT22(temp_hum_nuc_pin)
    
    # Creacion del objeto para la lectura de temperatura y humedad del robot
    obj_temp_hum_robot = dht.DHT22(temp_hum_rob_pin)
    
    # Creacion del objeto para la lectura de temperatura y humedad del ambiente
    obj_temp_hum_ambiente = dht.DHT22(temp_hum_amb_pin)
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    
    
    while True:
        
        # ---------------------------------------------------------------  Sensor de Temperatura ---------------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de temperatura, muestreo cada 2000ms
        valor_sensor_temperatura = sensor_temperatura(arreglo_roms=roms,sensor_temp=ds_sensor_temperatura)             
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
        
        
        # ---------------------------------------------------------------  Reles ---------------------------------------------------------------------------------- #
        if(i<len(reles)):
            reles[i].on()
            
        elif(i == len(reles)):
            apagar_reles()
            
        i = i + 1
        time.sleep_ms(1000)
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        # ---------------------------------------------------------------  Sensor Temperatura Humedad ----------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de temperatura y humedad del nucleo
        dht_temperatura_nucleo, dht_humedad_nucleo = obtener_temp_hum(obj_temp_hum_nucleo)  
        
        # Se obtienen los datos del sensor de temperatura y humedad del robot
        dht_temperatura_robot, dht_humedad_robot = obtener_temp_hum(obj_temp_hum_robot)    
        
        # Se obtienen los datos del sensor de temperatura y humedad del ambiente
        dht_temperatura_ambiente, dht_humedad_ambiente = obtener_temp_hum(obj_temp_hum_ambiente)   
        
            
                  
execution_core()    