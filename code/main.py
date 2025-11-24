from machine import Pin, I2C, time_pulse_us
import time
import network
import ubinascii
import machine
import usocket as socket
import ustruct as struct
import ssd1306  # ENSURE ssd1306.py IS ON YOUR PICO

# =========================================
# 1. CONFIGURATION
# =========================================
WIFI_SSID = "Ambika's Galaxy M14 5G"
WIFI_PASSWORD = "Ambika@30"

# --- MQTT CONFIG ---
MQTT_SERVER = "test.mosquitto.org"
MQTT_PORT = 1883
MQTT_TOPIC = "wokwi/flood/monitor"

# --- THINGSPEAK CONFIG (NEW) ---
# Replace with your actual Write API Key from ThingSpeak
THINGSPEAK_API_KEY = "R9862RO7AL9IENNT" 
THINGSPEAK_HOST = "api.thingspeak.com"

# =========================================
# 2. EMBEDDED MQTT CLIENT (UNCHANGED)
# =========================================
class MQTTClient:
    def __init__(self, client_id, server, port=0, user=None, password=None, keepalive=0, ssl=False, ssl_params={}):
        if port == 0:
            port = 8883 if ssl else 1883
        self.client_id = client_id
        self.sock = None
        self.server = server
        self.port = port
        self.ssl = ssl
        self.ssl_params = ssl_params
        self.pid = 0
        self.user = user
        self.pswd = password
        self.keepalive = keepalive
        self.lw_topic = None
        self.lw_msg = None
        self.lw_qos = 0
        self.lw_retain = False

    def _send_str(self, s):
        self.sock.write(struct.pack("!H", len(s)))
        self.sock.write(s)

    def _recv_len(self):
        n = 0
        sh = 0
        while 1:
            b = self.sock.read(1)[0]
            n |= (b & 0x7F) << sh
            if not b & 0x80:
                return n
            sh += 7

    def connect(self, clean_session=True):
        self.sock = socket.socket()
        addr = socket.getaddrinfo(self.server, self.port)[0][-1]
        self.sock.connect(addr)
        if self.ssl:
            import ussl
            self.sock = ussl.wrap_socket(self.sock, **self.ssl_params)
        msg = bytearray(b"\x10\0\0\0\0\0")
        var_header = bytearray(b"\x04MQTT\x04\x02\0\0")
        sz = 10 + 2 + len(self.client_id)
        var_header[6] = clean_session << 1
        if self.user is not None:
            sz += 2 + len(self.user) + 2 + len(self.pswd)
            var_header[6] |= 0xC0
        if self.keepalive:
            var_header[7] |= self.keepalive >> 8
            var_header[8] |= self.keepalive & 0x00FF
        if self.lw_topic is not None:
            sz += 2 + len(self.lw_topic) + 2 + len(self.lw_msg)
            var_header[6] |= 0x04 | (self.lw_qos & 0x1) << 3 | (self.lw_qos & 0x2) << 3
            if self.lw_retain:
                var_header[6] |= 0x20
        i = 1
        while sz > 0x7F:
            msg[i] = (sz & 0x7F) | 0x80
            sz >>= 7
            i += 1
        msg[i] = sz
        self.sock.write(msg, i + 2)
        self.sock.write(var_header)
        self._send_str(self.client_id)
        if self.lw_topic is not None:
            self._send_str(self.lw_topic)
            self._send_str(self.lw_msg)
        if self.user is not None:
            self._send_str(self.user)
            self._send_str(self.pswd)
        resp = self.sock.read(4)
        return resp[2] & 1

    def disconnect(self):
        self.sock.write(b"\xe0\0")
        self.sock.close()

    def publish(self, topic, msg, retain=False, qos=0):
        pkt = bytearray(b"\x30\0\0\0")
        pkt[0] |= qos << 1 | retain
        sz = 2 + len(topic) + len(msg)
        if qos > 0:
            sz += 2
        i = 1
        while sz > 0x7F:
            pkt[i] = (sz & 0x7F) | 0x80
            sz >>= 7
            i += 1
        pkt[i] = sz
        self.sock.write(pkt, i + 1)
        self._send_str(topic)
        if qos > 0:
            self.pid += 1
            self.sock.write(struct.pack("!H", self.pid))
        self.sock.write(msg)

# =========================================
# 3. HARDWARE SETUP
# =========================================
TRIGGER = Pin(14, Pin.OUT)
ECHO = Pin(15, Pin.IN)
BUZZER = Pin(16, Pin.OUT, value=1)  # Start Silent (Inverted)

# --- NEW LED PINS ---
RED_LED = Pin(17, Pin.OUT, value=0)   # Connected to GPIO 17
GREEN_LED = Pin(18, Pin.OUT, value=0) # Connected to GPIO 18
# --------------------

# --- I2C & Display Setup ---
I2C_BUS = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
try:
    display = ssd1306.SSD1306_I2C(128, 64, I2C_BUS)
except:
    print("OLED not found. Check wiring.")
    display = None


# =========================================
# 4. FUNCTIONS
# =========================================

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print("Connecting WiFi...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        timeout = 10
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1
    if wlan.isconnected():
        print('WiFi OK:', wlan.ifconfig()[0])
    else:
        print("WiFi Failed")
    return wlan.isconnected()

def get_distance():
    TRIGGER.low()
    time.sleep_us(2)
    TRIGGER.high()
    time.sleep_us(10)
    TRIGGER.low()
    try:
        dur = time_pulse_us(ECHO, 1, 30000)
        return (dur * 0.0343) / 2 if dur > 0 else -1
    except:
        return -1

def control_leds(status):
    if status == "!!CRIT!!":
        RED_LED.value(1)   
        GREEN_LED.value(0) 
    elif status == "WARNING":
        RED_LED.value(1)   
        GREEN_LED.value(0) 
    elif status == "SAFE":
        RED_LED.value(0)   
        GREEN_LED.value(1) 
    else: 
        RED_LED.value(0)
        GREEN_LED.value(0)

def update_screen(dist, status, mqtt_ok):
    if not display:
        return
    display.fill(0)
    display.text("FLOOD ALERT", 15, 0)
    display.hline(0, 10, 128, 1)
    display.text("MQTT: {}".format("OK" if mqtt_ok else "--"), 0, 18)
    display.text("Level: {:.1f} cm".format(dist), 0, 33)
    display.text("Status:", 0, 48)
    display.text(status, 60, 48)
    display.show()

# --- THINGSPEAK HTTP SEND FUNCTION ---
def send_to_thingspeak(level, status_msg):
    if THINGSPEAK_API_KEY == "YOUR_WRITE_API_KEY_HERE":
        print("Err: Set API Key first")
        return
        
    try:
        # We map status strings to simpler text to ensure URL safety
        # Field 1: Level (Distance)
        # Field 2: Status Text
        
        # Create HTTP Request Manually (uses socket, no extra libs needed)
        addr = socket.getaddrinfo(THINGSPEAK_HOST, 80)[0][-1]
        s = socket.socket()
        s.connect(addr)
        
        path = "/update?api_key={}&field1={:.1f}&field2={}".format(
            THINGSPEAK_API_KEY, level, status_msg)
        
        request = "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n".format(
            path, THINGSPEAK_HOST)
        
        s.write(request.encode())
        s.close()
        print(">> ThingSpeak Updated")
    except Exception as e:
        print(">> ThingSpeak Error:", e)

# =========================================
# 5. MAIN LOGIC
# =========================================
print("--- STARTING COMPLETE SYSTEM ---")
wifi_ok = connect_wifi()

# MQTT Setup
mqtt = MQTTClient("pico-" + ubinascii.hexlify(machine.unique_id()).decode(), MQTT_SERVER, port=MQTT_PORT, keepalive=60)
mqtt_connected = False
try:
    if wifi_ok:
        mqtt.connect()
        mqtt_connected = True
        print("MQTT Connected!")
except:
    print("MQTT Initial Connect Failed")

# Timers
last_pub_mqtt = time.time()
last_pub_thingspeak = time.time()

while True:
    dist = get_distance()
    
    # Handle Logic
    if dist > 0:
        if dist < 8:
            status = "!!CRIT!!"
            BUZZER.off()
            time.sleep(0.3)
            BUZZER.on()
        elif dist < 15:
            status = "WARNING"
            BUZZER.off()
            time.sleep(0.1)
            BUZZER.on()
        else:
            status = "SAFE"
            BUZZER.on()

        control_leds(status)
        
        # --- MQTT UPDATE (Every 5-10 seconds) ---
        if time.time() - last_pub_mqtt >= 10:
            if not mqtt_connected and wifi_ok:
                try:
                    mqtt.connect()
                    mqtt_connected = True
                except: pass
            
            if mqtt_connected:
                try:
                    msg = f'{{"level":{dist:.1f},"status":"{status}"}}'
                    mqtt.publish(MQTT_TOPIC, msg)
                    print(f"MQTT Pub: {msg}")
                    last_pub_mqtt = time.time()
                except:
                    mqtt_connected = False
                    try: mqtt.disconnect()
                    except: pass
        
        # --- THINGSPEAK UPDATE (Every 16+ seconds) ---
        # ThingSpeak Free requires 15s gap minimum
        if time.time() - last_pub_thingspeak >= 16:
            if wifi_ok:
                # Send distance to Field 1, Status to Field 2
                send_to_thingspeak(dist, status)
                last_pub_thingspeak = time.time()

        update_screen(dist, status, mqtt_connected)

    else:
        # Sensor Error
        status = "ERROR"
        control_leds(status)
        print("Sensor Error")
        time.sleep(0.5)

    # Short delay for loop stability
    time.sleep(0.2)