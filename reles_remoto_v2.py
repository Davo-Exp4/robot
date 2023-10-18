import time, machine
# Manejo de pines de entrada y salida - Control Analogo Digital
from machine import ADC, Pin, UART, I2C
import network
from simple import MQTTClient
import json

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

global run
run = True

# -------------------------------------------------------------  Configuracion Sensor ESP8266 -------------------------------------------------------------- #
WiFi_SSID = 'Kame-House'                                            # Wifi_SSID
WiFi_password = '1720015773lola'                                    # WiFi Password
ultima_peticion = 0
intervalo_peticiones = 45
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#

# -------------------------------------------------------------  Configuracion Endpoint AWS ---------------------------------------------------------------- #
CLIENT_ID = "acuamattic-001"                                        # Id del dispositivo, debe ser diferente para otro
AWS_ENDPOINT = b'abs5cr0g9wj2z-ats.iot.eu-west-1.amazonaws.com'     # Endpoint de AWS IoT Core
PUBLISH_CHANNEL='parametros_acuario'                                # Canal MQTT para publicar
SUBSCRIBED_CHANNEL='funcion'                                            # Canal MQTT para suscribir
# -----------------------------------------------------------------------------------------------------------------------------------------------------------#


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
    
    try:
        
        if topic==b'funcion':
            # Prueba led
            if msg==b'on':
                led.on()
            elif msg==b'off':
                led.off()
                
            # Para encender los reles NC
            elif msg==b'NC1on':
                reley_NC_1.on()
            
            elif msg==b'NC3on':
                reley_NC_3.on()
                # Este es el 3
            elif msg==b'NC4on':
                reley_NC_4.on()
                
            elif msg==b'NC2on':
                rl_vent_exterior.on()
                
            # Para apagar los reles NC
            elif msg==b'NC1off':
                reley_NC_1.off()
            elif msg==b'NC3off':
                reley_NC_3.off()
            elif msg==b'NC4off':
                reley_NC_4.off()
                
            elif msg==b'NC2off':
                rl_vent_exterior.off()
                
            # Para encender los reles NA
            elif msg==b'NA1on':
                reley_NA_1.on()
            elif msg==b'NA2on':
                reley_NA_2.on()
            elif msg==b'NA3on':
                reley_NA_3.on()
            elif msg==b'NA4on':
                reley_NA_4.on()
            elif msg==b'NA5on':
                reley_NA_5.on()
            elif msg==b'NA6on':
                reley_NA_6.on()
            elif msg==b'NA7on':
                reley_NA_7.on()
            elif msg==b'NA8on':
                reley_NA_8.on()
                
            # Para apagar los reles NA
            elif msg==b'NA1off':
                reley_NA_1.off()
            elif msg==b'NA2off':
                reley_NA_2.off()
            elif msg==b'NA3off':
                reley_NA_3.off()
            elif msg==b'NA4off':
                reley_NA_4.off()
            elif msg==b'NA5off':
                reley_NA_5.off()
            elif msg==b'NA6off':
                reley_NA_6.off()
            elif msg==b'NA7off':
                reley_NA_7.off()
            elif msg==b'NA8off':
                reley_NA_8.off()
            elif msg==b'romper':
                run = False
            else:
                print("Esto no esta programado al momento")
    except:
        print("An exception occurred")
        
        
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
    
    mqtt.subscribe(SUBSCRIBED_CHANNEL) 
    
    # Send 10 messages to test publish messages
    print("Sending messages...")
    for i in range(1):
        #get temperature
        # temperature=read_internal_temp_sensor()
        # Publish temperature to temperature channel it's in JSON format
        # so that AWS will not give Message format error
        temperature = 5.3
        print('{"temp":%s}'% temperature)
        message = {
            'temperature': temperature,
            'humidity': temperature
        }
        mqtt.publish( topic = PUBLISH_CHANNEL, msg = json.dumps(message), qos = 0 )
    # mqtt.publish( topic = 'test', msg = b'{"temp":%s}'% temperature , qos = 0 )
    time.sleep_ms(2000)
    print("done sending messages waiting for messages...")
    
    while (run == True):
        # ---------------------------------------------------------------  Sensor de Luz ------------------------------------------------------------------------------------------ #
        
        # ----------------------------------------------------------- Mensaje de configuracion MQTT ------------------------------------------------------------------------------- #
        mqtt.check_msg()
        time.sleep_ms(100)
        # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- #


# Ejecucion del codigo CORE 
execution_core()
