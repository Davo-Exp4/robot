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
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# -------------------------------------------------------------  Declaracion Pines Entrada ----------------------------------------------------------------- #

tds_1_pin       = Pin(26, mode=Pin.IN)                              # Pin de entrada analoga para la lectura del sensor de TDS 1
tds_2_pin       = Pin(27, mode=Pin.IN)                              # Pin de entrada analoga para la lectura del sensor de TDS 2
tds_3_pin       = Pin(28, mode=Pin.IN)                              # Pin de entrada analoga para la lectura del sensor de TDS 3

temp_hum_nuc_pin= Pin(0, mode=Pin.IN)                               # Pin de entrada digital para la lectura de temperatura y humedad nucleo
temp_hum_rob_pin= Pin(20, mode=Pin.IN)                              # Pin de entrada digital para la lectura de temperatura y humedad robot
temp_hum_amb_pin= Pin(21, mode=Pin.IN)                              # Pin de entrada digital para la lectura de temperatura y humedad ambiente

temp_acuario_pin= Pin(22, mode=Pin.IN)                              # Pin de entrada digital para la lectura del sensor de temperatura
nivel_agua_pin  = Pin(12, mode=Pin.IN, pull=Pin.PULL_UP)            # Pin entrada para la lectura del sensor de nivel de agua se levenatan las resistencias de PULL UP
caudal_pin      = Pin(2, mode=Pin.IN, pull=Pin.PULL_UP)             # Pin de entrada para el sensor caudal, se levenatan las resistencias de PULL UP
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#
               
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


i = 0
# Relays Normalmente Cerrado
reley_NC_1     = Pin(16, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
# reley_NC_2     = Pin(17, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
reley_NC_3     = Pin(18, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
reley_NC_4     = Pin(19, mode=Pin.OUT)                              # Pin de salida de reley NC de dispositivo XXXX
led = machine.Pin("LED", machine.Pin.OUT)
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# -------------------------------------------------------------  Configuracion Sensor luz ADS115 ----------------------------------------------------------- #
dev = I2C(1, freq=400000, scl=Pin(15), sda=Pin(14))                 # Configuracion I2C para el sensor de luz
devices = dev.scan()
address = 72 
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# -------------------------------------------------------------  Configuracion Sensor Caudal --------------------------------------------------------------- #
global count
count = 0

global run
run = True
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# -------------------------------------------------------------  Configuracion Sensor ESP8266 -------------------------------------------------------------- #
# WiFi_SSID = 'Kame-House'                                            # Wifi_SSID
# WiFi_password = '1720015773lola'
WiFi_SSID = 'Red Wi-Fi de Agustin'                                            # Wifi_SSID
WiFi_password = '956271235'   
ultima_peticion = 0
intervalo_peticiones = 45
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# -------------------------------------------------------------  Configuracion Endpoint AWS ---------------------------------------------------------------- #
CLIENT_ID = "acuamattic-001"                                        # Id del dispositivo, debe ser diferente para otro
AWS_ENDPOINT = b'abs5cr0g9wj2z-ats.iot.eu-west-1.amazonaws.com'     # Endpoint de AWS IoT Core
PUBLISH_CHANNEL='parametros_acuario'                                # Canal MQTT para publicar
SUBSCRIBED_CHANNEL='funcion'                                            # Canal MQTT para suscribir
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# ----------------------------------------------------  Funciones manejo de sensores y actuadores ---------------------------------------------------------- #  
# Funcion usada para obtener los certificados con AWS IoT
def get_ssl_params():
    """
    Funcion utilizada para obtener los certificados generados por AWS IoT core 
    para su conexion
    
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    # Deben estar en formato DER
    keyfile = '/private.der'
    with open(keyfile, 'rb') as f:
        key = f.read()
    certfile = "/certificate.der"
    with open(certfile, 'rb') as f:
        cert = f.read()    
    ssl_params = {'key': key,'cert': cert, 'server_side': False}
    return ssl_params

# Funcion para publicar desde la nube de AWS 
def mqtt_callback(topic, msg):
    """
    Funcion utilizada para recibir mensajes publicados desde amazon AWS IoT Core
    
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    global run
    # Se prueba prendiendo un led
    print("received data:")
    print("topic: %s message: %s" %(topic, msg))
    if topic==b'funcion':
        # Para encender los reles NC
        if msg==b'rele_NC_1_on':
            reley_NC_1.on()
        elif msg==b'on':
            led.on()
        elif msg==b'off':
            led.off()
        elif msg==b'rele_NC_3_on':
            reley_NC_3.on()
            # Este es el 3
        elif msg==b'rele_NC_4_on':
            reley_NC_4.on()
            
        elif msg==b'rele_NC_2_on':
            rl_vent_exterior.on()
            
        # Para apagar los reles NC
        elif msg==b'rele_NC_1_off':
            reley_NC_1.off()
        elif msg==b'rele_NC_3_off':
            reley_NC_3.off()
        elif msg==b'rele_NC_4_off':
            reley_NC_4.off()
            
        elif msg==b'rele_NC_2_off':
            rl_vent_exterior.off()
            
        # Para encender los reles NA
        elif msg==b'rele_NA_1_on':
            reley_NA_1.on()
        elif msg==b'rele_NA_2_on':
            reley_NA_2.on()
        elif msg==b'rele_NA_3_on':
            reley_NA_3.on()
        elif msg==b'rele_NA_4_on':
            reley_NA_4.on()
        elif msg==b'rele_NA_5_on':
            reley_NA_5.on()
        elif msg==b'rele_NA_6_on':
            reley_NA_6.on()
        elif msg==b'rele_NA_7_on':
            reley_NA_7.on()
        elif msg==b'rele_NA_8_on':
            reley_NA_8.on()
            
        # Para apagar los reles NA
        elif msg==b'rele_NA_1_off':
            reley_NA_1.off()
        elif msg==b'rele_NA_2_off':
            reley_NA_2.off()
        elif msg==b'rele_NA_3_off':
            reley_NA_3.off()
        elif msg==b'rele_NA_4_off':
            reley_NA_4.off()
        elif msg==b'rele_NA_5_off':
            reley_NA_5.off()
        elif msg==b'rele_NA_6_off':
            reley_NA_6.off()
        elif msg==b'rele_NA_7_off':
            reley_NA_7.off()
        elif msg==b'rele_NA_8_off':
            reley_NA_8.off()
        
        
        
        elif msg==b'romper':
            run = False
        else:
            print("Esto no esta programado al momento %s" % msg)

def mqtt_callback_2(topic, msg):

    """ Callback function for received messages"""
    print("received data:")
    print("topic: %s message: %s" %(topic, msg))
    print(type(msg))
    my_json = msg.decode('utf8')
    print(type(my_json))
    print(my_json)
    # response = json.load(msg)
    m_in=json.loads(my_json)
    print(m_in)
    print(type(m_in))
    print(m_in['humidity'])
    print(m_in['especie'])
    
def mqtt_callback_3(topic, msg):
    
    """ Callback function for received messages"""
    my_json = msg.decode('utf8')
    
    # response = json.load(msg)
    m_in=json.loads(my_json)
    print(m_in['payload'])
    # print(m_in['especie'])
    payload_mensaje = m_in['payload']
    
    try:
        
        if topic==b'funcion':
            # Prueba led
            if payload_mensaje=='on':
                led.on()
            elif payload_mensaje=='off':
                led.off()
                
            elif payload_mensaje=='vent_nucleo_on':
                rl_vent_nucleo.on()
                
            elif payload_mensaje=='vent_nucleo_off':
                rl_vent_nucleo.off()
                
            # Para encender los reles NC
            elif payload_mensaje=='NC1on':
                reley_NC_1.on()
            
            elif payload_mensaje=='NC3on':
                reley_NC_3.on()
                # Este es el 3
            elif payload_mensaje=='NC4on':
                reley_NC_4.on()
                
            elif payload_mensaje=='NC2on':
                rl_vent_exterior.on()
                
            # Para apagar los reles NC
            elif payload_mensaje=='NC1off':
                reley_NC_1.off()
            elif payload_mensaje=='NC3off':
                reley_NC_3.off()
            elif payload_mensaje=='NC4off':
                reley_NC_4.off()
                
            elif payload_mensaje=='NC2off':
                rl_vent_exterior.off()
                
            # Para encender los reles NA
            elif payload_mensaje=='NA1on':
                reley_NA_1.on()
            elif payload_mensaje=='NA2on':
                reley_NA_2.on()
            elif payload_mensaje=='NA3on':
                reley_NA_3.on()
            elif payload_mensaje=='NA4on':
                reley_NA_4.on()
            elif payload_mensaje=='NA5on':
                reley_NA_5.on()
            elif payload_mensaje=='NA6on':
                reley_NA_6.on()
            elif payload_mensaje=='NA7on':
                reley_NA_7.on()
            elif payload_mensaje=='NA8on':
                reley_NA_8.on()
                
            # Para apagar los reles NA
            elif payload_mensaje=='NA1off':
                reley_NA_1.off()
            elif payload_mensaje=='NA2off':
                reley_NA_2.off()
            elif payload_mensaje=='NA3off':
                reley_NA_3.off()
            elif payload_mensaje=='NA4off':
                reley_NA_4.off()
            elif payload_mensaje=='NA5off':
                reley_NA_5.off()
            elif payload_mensaje=='NA6off':
                reley_NA_6.off()
            elif payload_mensaje=='NA7off':
                reley_NA_7.off()
            elif payload_mensaje=='NA8off':
                reley_NA_8.off()
            elif payload_mensaje=='romper':
                run = False
            else:
                print("Esto no esta programado al momento")
    except:
        print("An exception occurred")
  
# Funcion usada para la reconexion de la red
def reconectar():
    """
    Funcion utilizada para reconectar en caso de que exista falla en la red
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    print('Fallo de conexión. Reconectando...')
    time.sleep(10)
    machine.reset()
      
# Funcion para reconectar en caso de no establecer conexion
def check_wifi(wlan):
    """Wait for connection"""
    while not wlan.isconnected():
        time.sleep_ms(700)
        print(".")
        wlan.connect( WiFi_SSID, WiFi_password )
    if not wlan.isconnected():
        print("not connected")
    if wlan.isconnected():
        print("connected")
        
    
# Envio de informacion al servidor de los sensores
def send_data(mqtt, luz_ambiente,tempe_acuario, temp_pico, TDS1, TDS2, TDS3, caudal, n_agua, dht_temp_nuc, dht_hum_nuc, dht_temp_rob, dht_hum_rob, dht_temp_amb, dht_hum_amb):  
    """
    Funcion utilizada para el envio de informacion al servidor cloud de todas las variables 
    sensadas al momento
        
    Parameters
    ----------
    luz_ambiente:   valor sensor de luz ambiente
    tempe_acuario:  temperatura del sensor de acuario
    temp_pico:      temperatura del core de la raspeberry pi pico
    TDS1:           valor del sensor TDS 1
    TDS2:           valor del sensor TDS 2
    TDS3:           valor del sensor TDS 3
    caudal:         valor del sensor de caudal
    n_agua:         valor del sensor de nivel de agua
    dht_temp_nuc:   valor del sensor de temperatura del nucleo
    dht_hum_nuc:    valor del sensor de humedad del nucleo
    dht_temp_rob:   valor del sensor de temperatura del robot
    dht_hum_rob:    valor del sensor de humedad del robot
    dht_temp_amb:   valor del sensor de temperatura del ambiente
    dht_hum_amb:    valor del sensor de humedad del ambiente
        
    Returns
    -------
    N/A
    """  
    global ultima_peticion, intervalo_peticiones
    try:
        message = {
            'luz_ambiente': luz_ambiente,
            'temperatura_acuario': tempe_acuario,
            'temperatura_core': temp_pico,
            'TDS_1': TDS1,
            'TDS_2': TDS2,
            'TDS_3': TDS3,
            'caudal': caudal,
            # 'nivel_agua': n_agua,
            'dht_temperatura_nucleo': dht_temp_nuc,
            'dht_humedad_nucleo': dht_hum_nuc,
            'dht_temperatura_robot': dht_temp_rob,
            'dht_humedad_robot': dht_hum_rob,
            'dht_temperatura_ambiente': dht_temp_amb,
            'dht_humedad_ambiente': dht_hum_amb
        }
        mqtt.publish( topic = PUBLISH_CHANNEL, msg = json.dumps(message), qos = 0 )
            
        time.sleep_ms(100)
        # ultima_peticion = time.time()
    except OSError as e:
        reconectar()
# wifi de agustin
# 956271235

# Lectura de los bytes, lectura de la configuracion del sensor ADS1115
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

# Dispositivos de control de caudal  de valvulas
def encender_valvulas():
    
    """
    Funcion utilizada para activar los dispositivos actuadores cuando se cumpla una 
    determinada condicion 
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    # relay_pin.on()
    seleneoide_pin.on()

# Dispositivos de control de caudal  de valvulas   
def apagar_valvulas():
    
    """
    Funcion utilizada para apagar los dispositivos actuadores cuando se cumpla una 
    determinada condicion 
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    # relay_pin.off()
    seleneoide_pin.off()

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

# Ventiladores exterior - encender
def encender_ventiladores_exterior():
    """
    Funcion utilizada para el control de los ventiladores - encender  
        
    Parameters
    ----------
    N/A
        
    Returns
    -------
    N/A
    """
    
    print("Ventiladores Encendidos")
    rl_vent_exterior.on()

# Ventiladores del exterior - apagar   
def apagar_ventiladores_exterior():
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
    rl_vent_exterior.off()
    
# Funcion utilizada para la lectura del sensor de luz 
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

# Funcion utilizada para el control de nivel de agua
def sensor_nivel_agua():
    i = 0
    """
    Funcion utilizada para tomar accion cuando los niveles de agua se encuentren en el limite
        
    Parameters
    ----------
    pin_entrada_nivel:  objeto creado por la clase Pin, donde se especifica el numero de 
                        pin de la entrada del sensor.
        
    Returns
    -------
    pin_entrada_nivel:      Estado del pin que sensa el nivel de agua
    """
    reles = [reley_NA_1,reley_NA_2, rl_vent_exterior, reley_NA_3, reley_NA_4, reley_NA_5, reley_NA_6, reley_NA_7, reley_NA_8, reley_NC_1, reley_NC_3, reley_NC_4]
    pin_entrada_nivel  = Pin(12, mode=Pin.IN, pull=Pin.PULL_UP)
    while True:
        # Estado tanque - Nivel de agua alto, se para la valvula selenoide o reley
        if pin_entrada_nivel.value() == 0:
            encender_valvulas()
            
            # print("Tanque vacio...valvula abierta")
            time.sleep_ms(10)
            
        if pin_entrada_nivel.value() == 1:
            apagar_valvulas()
            
            # print("Tanque lleno...valvula cerrada")
            time.sleep_ms(10)
            
        print(reles[i])
        reles[i].on()
        i = i + 1
        time.sleep_ms(4000)

# Funcion utilizada para el control TDS
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

# Funcion de handler sensor de caudal
def obtener_frecuencia(channel):
    """
    Funcion para obtener la frecuencia del canal
        
    Parameters
    ----------
    channel:            canal del sensor de caudal 
        
    Returns
    -------
    N/A
    """
    global count, start_counter
    if start_counter == 1:
        count = count+1
        
# Funcion utilizada para el sensor de caudal
def sensor_caudal(factor_conversion):
    """
    Funcion utilizada para el sensor de caudal
        
    Parameters
    ----------
    factor_conversion:      variable para el calculo del sensor de caudal
        
    Returns
    -------
    flow:                   valor del sensor de caudal
    """
    global start_counter, count
    start_counter = 1
    time.sleep(1)
    start_counter = 0
    flow = (count / factor_conversion) # Pulse frequency (Hz) = 11Q, Q is flow rate in L/min.
    print("The flow is: %.3f Liter/min" % (flow))
    count = 0
    time.sleep_ms(1000)
    return flow

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

# Funcion para el accionar de los motores 
def accion_motores(motor_cl, id_motor='MA', potencia=100, direccion="forward", tiempo=10):
    """
    Funcion utilizada para el accionar de los motores tanto del sistema de osmosis 
    (bombas peristalticas) asi como tambien el de analisis de agua

    Parameters
    ----------
    motor_cl:           objeto instanciado de la clase motor
    id_motor:           identificador del motor
    potencia:           variable para controlar la velocidad del motor pwm
    direccion:          controla el sentido del motor
    tiempo:             indica la duracion por la cual va a estar encendido el motor
        
    Returns
    -------
    N/A:        
    """
    try:
        motor_cl.MotorRun(id_motor, direccion, potencia, tiempo)
        print(f"Motor {id_motor} en direccion: {direccion} con una potencia de {potencia} por {tiempo} segundos")
    except OSError as e:
        print("fError: {e}")
        
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
    global run
    i = 0
    # -------------------------------------------------------------  Configuracion Sensor de ESP8266 ------------------------------------------------------------------------------ #
    # Establecer conexion con la red WiFi
    red = network.WLAN(network.STA_IF)
    red.active(True)
    red.connect(WiFi_SSID, WiFi_password)
    
    while red.isconnected() == False:
        pass
    
    print('Conexión correcta')
    
    ssl_params=get_ssl_params()
    print(red.ifconfig())
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

    # -------------------------------------------------------------  Conexion con el broker MQTT ---------------------------------------------------------------------------------- #
    mqtt = MQTTClient( CLIENT_ID, AWS_ENDPOINT, port = 8883, keepalive = 10000, ssl = True, ssl_params = ssl_params )
    mqtt.set_callback(mqtt_callback_3)
    mqtt.connect()
    
    mqtt.subscribe(SUBSCRIBED_CHANNEL)                                  # suscripcion al canal
    # -------------------------------------------------------------  Configuracion Sensor de Temperatura -------------------------------------------------------------------------- #
    ds_sensor_temperatura = ds18x20.DS18X20(onewire.OneWire(temp_acuario_pin))
    roms = ds_sensor_temperatura.scan()
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    # ------------------------------------------------------------------  Configuracion Sensor de TDS ----------------------------------------------------------------------------- #
    set_vref = 3.3
    temperature_tds = 25
    adc_range = 65536
    
    valor_analogo_tds_1 = ADC(tds_1_pin)
    valor_analogo_tds_2 = ADC(tds_2_pin)
    valor_analogo_tds_3 = ADC(tds_3_pin)
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    # -------------------------------------------------------------  Configuracion Sensor Caudal ---------------------------------------------------------------------------------- #
    factor_conversion=11                                        #para convertir de frecuencia a caudal
    #attach the interrupt to the sensor
    caudal_pin.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=obtener_frecuencia)
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    # -------------------------------------------------------------  Configuracion Sensor Temperatura ----------------------------------------------------------------------------- #
    sensor_temp = ADC(4)
    conversion_factor = 3.3 / (65535)
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    # -------------------------------------------------------------  Configuracion Sensor Temperatura y Humedad ------------------------------------------------------------------- #
    # Creacion del objeto para la lectura de temperatura y humedad del nucleo
    obj_temp_hum_nucleo = dht.DHT22(temp_hum_nuc_pin)
    
    # Creacion del objeto para la lectura de temperatura y humedad del robot
    obj_temp_hum_robot = dht.DHT22(temp_hum_rob_pin)
    
    # Creacion del objeto para la lectura de temperatura y humedad del ambiente
    obj_temp_hum_ambiente = dht.DHT22(temp_hum_amb_pin)
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    
    # -------------------------------------------------------------  Configuracion Motores ----------------------------------------------------------------------------- #
    try:
        motores_peristalticos = motor.MotorDriver()
        motores_analisis = motor.MotorDriver(default=True)
        time.sleep_ms(500)
    except:
        pass
    # ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
    # reles = [reley_NA_1,reley_NA_2, rl_vent_exterior, reley_NA_3, reley_NA_4, reley_NA_5, reley_NA_6, reley_NA_7, reley_NA_8, reley_NC_1, reley_NC_3, reley_NC_4]
    
    while True:
        # ---------------------------------------------------------------  Sensor de Luz ------------------------------------------------------------------------------------------ #
        # Se obtienen los datos del sensor de luz, muestreo cada 1000ms
        valor_sensor_luz = sensor_luz()                                                                        
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        # ---------------------------------------------------------------  Sensor de Temperatura ---------------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de temperatura, muestreo cada 2000ms
        valor_sensor_temperatura = sensor_temperatura(arreglo_roms=roms,sensor_temp=ds_sensor_temperatura)             
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
        
        # ---------------------------------------------------------------  Sensor de Nivel de Agua -------------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de nivel de agua para activar/desactivar la bomba, muestreo cada 2000ms
        # valor_sensor_nivel = sensor_nivel_agua(pin_entrada_nivel=nivel_agua_pin)                                                                 
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        # ---------------------------------------------------------------  Sensor Caudal ------------------------------------------------------------------------------------------ #
        # Se obtienen los datos del sensor de caudal, muestreo cada 5000ms
        valor_sensor_caudal = sensor_caudal(factor_conversion=factor_conversion)                                                                     
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        # ---------------------------------------------------------------  Sensores TDS ------------------------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de nivel de agua, muestreo cada 200ms
        valor_TDS_1 = sensor_TDS(analog_value=valor_analogo_tds_1, temperature=temperature_tds, set_vref=set_vref, adc_range=adc_range)
        valor_TDS_2 = sensor_TDS(analog_value=valor_analogo_tds_2, temperature=temperature_tds, set_vref=set_vref, adc_range=adc_range) 
        valor_TDS_3 = sensor_TDS(analog_value=valor_analogo_tds_3, temperature=temperature_tds, set_vref=set_vref, adc_range=adc_range)   
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        # ---------------------------------------------------------------  Sensor Temperatura Interna ----------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de interno de la raspberry pi pico, muestreo cada 2000ms
        temp_pico = temperatura_pico(sensor_temp=sensor_temp, conversion_factor=conversion_factor)                                                                     
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
        
        # ---------------------------------------------------------------  Sensor Temperatura Humedad ----------------------------------------------------------------------------- #
        # Se obtienen los datos del sensor de temperatura y humedad del nucleo
        dht_temperatura_nucleo, dht_humedad_nucleo = obtener_temp_hum(obj_temp_hum_nucleo)  
        
        # Se obtienen los datos del sensor de temperatura y humedad del robot
        dht_temperatura_robot, dht_humedad_robot = obtener_temp_hum(obj_temp_hum_robot)    
        
        # Se obtienen los datos del sensor de temperatura y humedad del ambiente
        dht_temperatura_ambiente, dht_humedad_ambiente = obtener_temp_hum(obj_temp_hum_ambiente)   
        
        
        
        # if (((int(dht_humedad_nucleo) > 60) and (int(dht_humedad_nucleo) < 200)) or ((int(dht_temperatura_nucleo) > 30))):
        #     encender_ventiladores_nucleo() 
        # if (((int(dht_humedad_nucleo) > 0) and (int(dht_humedad_nucleo) <= 60)) or ((int(dht_temperatura_nucleo) <= 30))):
        #     apagar_ventiladores_nucleo()                                                                   
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #
        
        # ---------------------------------------------------------------  Envio de Informacion ----------------------------------------------------------------------------------- #
        
        valor_sensor_nivel = 0
        send_data(mqtt, valor_sensor_luz,valor_sensor_temperatura, temp_pico, valor_TDS_1, valor_TDS_2, valor_TDS_3, valor_sensor_caudal, valor_sensor_nivel, \
            dht_temperatura_nucleo, dht_humedad_nucleo, dht_temperatura_robot, dht_humedad_robot, dht_temperatura_ambiente, dht_humedad_ambiente)
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

        # ----------------------------------------------------------- Mensaje de configuracion MQTT ------------------------------------------------------------------------------- #
        mqtt.check_msg()
        time.sleep_ms(100)
        
        
            
            
        
        
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #

_thread.start_new_thread(sensor_nivel_agua, ())
# Ejecucion del codigo CORE 
execution_core()
