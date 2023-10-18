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

tds_1_pin       = Pin(26, mode=Pin.IN)                              # Pin de entrada analoga para la lectura del sensor de TDS 1
tds_2_pin       = Pin(27, mode=Pin.IN)                              # Pin de entrada analoga para la lectura del sensor de TDS 2
tds_3_pin       = Pin(28, mode=Pin.IN)                              # Pin de entrada analoga para la lectura del sensor de TDS 3


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


# -------------------------------------------------------------  Configuracion Sensor luz ADS115 ----------------------------------------------------------- #
dev = I2C(1, freq=400000, scl=Pin(15), sda=Pin(14))                 # Configuracion I2C para el sensor de luz
devices = dev.scan()
address = 72 
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#


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

def read_configuration():
    """
    Funcion utilizada para leer la configuracion de los registros de la placa ADS1115 
    determinada condicion 
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    config:     variable de configuracion 
    """
    dev.writeto(address, bytearray([1]))
    # leer los primeros 2 bytes
    result = dev.readfrom(address,2)
    # Extraer los bytes mas significativos
    config = result[0]<<8 | result[1]
    return config

# Lectura de los bytes, lectura del sensor ADS1115
def read_value():
    """
    Funcion utilizada para leer los registros de la placa ADS1115 
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    result:     arreglo con los valores de los registros 
    """
    dev.writeto(address, bytearray([0]))
    # leer los primeros 2 bytes
    result = dev.readfrom(address,2)
    # Extraer los bytes mas significativos
    config = read_configuration()
    config &= ~(7<<12) & ~(7<<9)
    config |= (4<<12) | (1<<9) | (1<<15)
    
    config = [int(config>>i & 0xff) for i in (8,0)]
    dev.writeto(address, bytearray([1] + config))
    return result[0]<<8 | result[1]

def sensor_luz():
    """
    Funcion utilizada para evaluar el porcentaje de luz y tomar accion,
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    valor_sensor_luz:       valor del sensor de luz
    """
    # Lectura mediante entrada Analoga/Digital
    valor_sensor_luz = read_value()
    valor_sensor_luz = int(valor_sensor_luz/25965*100) 
    print(f"El valor de la fotoresistencia es: {valor_sensor_luz} %")
    # Delay entre lecturas
    time.sleep_ms(100) 
    # Condicion cuando exceda el limite establecido
    if valor_sensor_luz<=35:
        led_pin.off()
    else:
        led_pin.on()
    return valor_sensor_luz


def sensor_TDS(analog_value, temperature=25, set_vref=3.3, adc_range=65536):
    """
    Funcion utilizada para tomar el muestreo de los sensores TDS distribuidos en el acuario
    todas las entradas son analogas. 
        
    Parameters
    ----------
    analog_value:       valor de lectura del pin ADC del sensor TDS
    temperature:        constante para la lectura del sensor TDS
    adc_range:          constante de la lectura del sensor TDS 16 bits
        
    Returns
    -------
    tds_value:          valor del sensor TDS 
    """
    reading = analog_value.read_u16()
    compensation_coefficient=1.0+0.02*(temperature-25.0) 
    average_voltage = reading * set_vref/adc_range   
    compensation_voltage=average_voltage/compensation_coefficient
    tds_value=(133.42*compensation_voltage*compensation_voltage*compensation_voltage - 255.86*compensation_voltage*compensation_voltage + 857.39*compensation_voltage)*0.5
    print(f"Valor de TDS es: {tds_value}")
    # Condicion para que en el caso de que exista una anomalia 
    if(tds_value>=0 and tds_value<100):
        # Nivel 1
        pass
    elif(tds_value>=200 and tds_value<300):
        # Nivel 2
        pass
    elif(tds_value>=300 and tds_value<400):
        # Nivel 3
        pass
    elif(tds_value>=400 and tds_value<500):
        # Nivel 3
        pass
    elif(tds_value>=500):
        # Nivel 4
        pass
    
    time.sleep_ms(200)
    return tds_value


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
    
    # -------------------------------------------------------------  Configuracion Sensor Temperatura ----------------------------------------------------------------------------- #
    sensor_temp = ADC(4)
    conversion_factor = 3.3 / (65535)
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    
    # ------------------------------------------------------------------  Configuracion Sensor de TDS ----------------------------------------------------------------------------- #
    set_vref = 3.3
    temperature_tds = 25
    adc_range = 65536
    
    valor_analogo_tds_1 = ADC(tds_1_pin)
    valor_analogo_tds_2 = ADC(tds_2_pin)
    valor_analogo_tds_3 = ADC(tds_3_pin)
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
        
        # ---------------------------------------------------------------  Sensor Temperatura Interna ----------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de interno de la raspberry pi pico, muestreo cada 2000ms
        temp_pico = temperatura_pico(sensor_temp=sensor_temp, conversion_factor=conversion_factor)                                                                     
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
        
        
        # ---------------------------------------------------------------  Sensor de Luz ------------------------------------------------------------------------------------------ #
        # Se obtienen los datos del sensor de luz, muestreo cada 1000ms
        valor_sensor_luz = sensor_luz()                                                                        
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        
        # ---------------------------------------------------------------  Sensores TDS ------------------------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de nivel de agua, muestreo cada 200ms
        valor_TDS_1 = sensor_TDS(analog_value=valor_analogo_tds_1, temperature=temperature_tds, set_vref=set_vref, adc_range=adc_range)
        valor_TDS_2 = sensor_TDS(analog_value=valor_analogo_tds_2, temperature=temperature_tds, set_vref=set_vref, adc_range=adc_range) 
        valor_TDS_3 = sensor_TDS(analog_value=valor_analogo_tds_3, temperature=temperature_tds, set_vref=set_vref, adc_range=adc_range)   
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

                  
execution_core()    