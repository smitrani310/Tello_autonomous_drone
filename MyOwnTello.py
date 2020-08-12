#!/usr/bin/env python
# coding: utf-8

# In[6]:


# coding=utf-8
import logging
import socket
import time
import threading
import cv2
# from accepts import accepts

from threading import Thread


# In[7]:


class ownTello:
    
    # Send and receive commands, client socket
    __UDP_IP = '192.168.10.1'
    __UDP_PORT = 8889
    RESPONSE_TIMEOUT = 7  # in seconds
    TIME_BTW_COMMANDS = 1  # in seconds
    TIME_BTW_RC_CONTROL_COMMANDS = 0.5  # in seconds
    RETRY_COUNT = 3
    last_received_command = time.time()    
    
    # Video stream, server socket
    __VS_UDP_IP = '0.0.0.0'
    __VS_UDP_PORT = 11111

    __STATE_UDP_PORT = 8890
    
    # need to figure more about this section 
    HANDLER = logging.StreamHandler()
    FORMATTER = logging.Formatter('%(filename)s - %(lineno)d - %(message)s')
    HANDLER.setFormatter(FORMATTER)

    LOGGER = logging.getLogger('djitellopy')

    LOGGER.addHandler(HANDLER)
    LOGGER.setLevel(logging.INFO)

    # Tello state
    pitch = -1
    roll = -1
    yaw = -1
    speed_x = -1
    speed_y = -1
    speed_z = -1
    temperature_lowest = -1
    temperature_highest = -1
    distance_tof = -1
    height = -1
    battery = -1
    barometer = -1.0
    flight_time = -1.0
    acceleration_x = -1.0
    acceleration_y = -1.0
    acceleration_z = -1.0
    attitude = {'pitch': -1, 'roll': -1, 'yaw': -1}    
    
    def __init__(self, host=__UDP_IP, port=__UDP_PORT, client_socket=None, enable_exceptions=True, retry_count=RETRY_COUNT):    
    
        self.address = (host, port)
        self.response = None
        self.response_state = None  # to attain the response of the states
        self.stream_on = False
        self.enable_exceptions = enable_exceptions
        self.retry_count = retry_count
        
        if client_socket:
            self.clientSocket = client_socket
        else:
            try:
                self.clientSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Internet, UDP
                self.clientSocket.bind(('', self.__UDP_PORT))  # For UDP response (receiving data)
                print( "Client Socket successfully created")
            except socket.error as err: 
                print( "socket creation failed with error %s" %(err) )

            try:
                self.stateSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.stateSocket.bind(('', self.__STATE_UDP_PORT))
                print( "State Socket successfully created")
            except socket.error as err: 
                print( "socket creation failed with error %s" %(err) )
                
    
        # Run tello udp receiver on background
        thread1 = threading.Thread(target=self.run_udp_receiver, args=(), daemon=True).start()
        
        # Run state reciever on background
        thread2 = threading.Thread(target=self.get_states, args=(), daemon=True).start()
        
        
        
    def run_udp_receiver(self):
        """Setup drone UDP receiver. This method listens for responses of Tello. Must be run from a background thread
        in order to not block the main thread."""
        while True:
            try:
                self.response, _ = self.clientSocket.recvfrom(1024)  # buffer size is 1024 bytes
            except Exception as e:
                self.LOGGER.error(e)
                break

    def get_states(self):
        """This runs on background to recieve the state of Tello"""
        while True:
            try:
                self.response_state, _ = self.stateSocket.recvfrom(256)
                if self.response_state != 'ok':
                    self.response_state = self.response_state.decode('ASCII')
                    list = self.response_state.replace(';', ':').split(':')
                    self.pitch = int(list[1])
                    self.roll = int(list[3])
                    self.yaw = int(list[5])
                    self.speed_x = int(list[7])
                    self.speed_y = int(list[9])
                    self.speed_z = int(list[11])
                    self.temperature_lowest = int(list[13])
                    self.temperature_highest = int(list[15])
                    self.distance_tof = int(list[17])
                    self.height = int(list[19])
                    self.battery = int(list[21])
                    self.barometer = float(list[23])
                    self.flight_time = float(list[25])
                    self.acceleration_x = float(list[27])
                    self.acceleration_y = float(list[29])
                    self.acceleration_z = float(list[31])
                    self.attitude = {'pitch': int(list[1]), 'roll': int(list[3]), 'yaw': int(list[5])}
            except Exception as e:
                self.LOGGER.error(e)
                self.LOGGER.error("Response was is {}".format(self.response_state))
                break
                
                
    def send_control_command(self, command, timeout=RESPONSE_TIMEOUT):
        """Send control command to Tello and wait for its response. Possible control commands:
            - command: entry SDK mode
            - takeoff: Tello auto takeoff
            - land: Tello auto land
            - streamon: Set video stream on
            - streamoff: Set video stream off
            - emergency: Stop all motors immediately
            - up x: Tello fly up with distance x cm. x: 20-500
            - down x: Tello fly down with distance x cm. x: 20-500
            - left x: Tello fly left with distance x cm. x: 20-500
            - right x: Tello fly right with distance x cm. x: 20-500
            - forward x: Tello fly forward with distance x cm. x: 20-500
            - back x: Tello fly back with distance x cm. x: 20-500
            - cw x: Tello rotate x degree clockwise x: 1-3600
            - ccw x: Tello rotate x degree counter- clockwise. x: 1-3600
            - flip x: Tello fly flip x
                l (left)
                r (right)
                f (forward)
                b (back)
            - speed x: set speed to x cm/s. x: 10-100
            - wifi ssid pass: Set Wi-Fi with SSID password
        Return:
            bool: True for successful, False for unsuccessful
        """
        response = None
        for i in range(0, self.retry_count):
            response = self.send_command_with_return(command, timeout=timeout)

            if response.upper() == 'OK':
                return True

        return self.return_error_on_send_command(command, response, self.enable_exceptions)
    
    
    
    
    
    def send_command_with_return(self, command, printinfo=True, timeout=RESPONSE_TIMEOUT):
        """Send command to Tello and wait for its response.
        Return:
            bool: True for successful, False for unsuccessful
        """
        # Commands very consecutive makes the drone not respond to them. So wait at least self.TIME_BTW_COMMANDS seconds
        diff = time.time() * 1000 - self.last_received_command
        if diff < self.TIME_BTW_COMMANDS:
            time.sleep(diff)

        if printinfo:
            self.LOGGER.info('Send command: ' + command)
        timestamp = int(time.time() * 1000)

        
        self.clientSocket.sendto(command.encode('utf-8'), self.address) # send the command 

        while self.response is None:
            if (time.time() * 1000) - timestamp > timeout * 1000:
                self.LOGGER.warning('Timeout exceed on command ' + command)
                return False

        try:
            response = self.response.decode('utf-8').rstrip("\r\n")
        except UnicodeDecodeError as e:
            self.LOGGER.error(e)
            return None

        if printinfo:
            self.LOGGER.info('Response {}: {}'.format(command, response))

        self.response = None

        self.last_received_command = time.time() * 1000

        return response
    
    
    def Entry_SDK_mode(self):
        """Entry SDK mode
        Returns:
            bool: True for successful, False for unsuccessful
        """
        return self.send_control_command("command")    

    
    
    def return_error_on_send_command(self, command, response, enable_exceptions):
        """Returns False and print an informative result code to show unsuccessful response"""
        msg = 'Command ' + command + ' was unsuccessful. Message: ' + str(response)
        if enable_exceptions:
            raise Exception(msg)
        else:
            self.LOGGER.error(msg)
            return False
        
        
        
    def send_read_command(self, command, printinfo=True):
        """Send set command to Tello and wait for its response. Possible set commands:
            - speed?: get current speed (cm/s): x: 1-100
            - battery?: get current battery percentage: x: 0-100
            - time?: get current fly time (s): time
            - height?: get height (cm): x: 0-3000
            - temp?: get temperature (°C): x: 0-90
            - attitude?: get IMU attitude data: pitch roll yaw
            - baro?: get barometer value (m): x
            - tof?: get distance value from TOF (cm): x: 30-1000
            - wifi?: get Wi-Fi SNR: snr
        Return:
            bool: The requested value for successful, False for unsuccessful
        """

        response = self.send_command_with_return(command, printinfo=printinfo)

        try:
            response = str(response)
        except TypeError as e:
            self.LOGGER.error(e)
            pass

        if ('ERROR' not in response.upper()) and ('False' not in response):
            if response.isdigit():
                return int(response)
            else:
                try:
                    return float(response)  # isdigit() is False when the number is a float(barometer)
                except ValueError:
                    return response
        else:
            return self.return_error_on_send_command(command, response, self.enable_exceptions)
        
        

        
    def end(self):
        """Call this method when you want to end the tello object"""
        if self.is_flying:
            self.land()
        if self.stream_on:
            self.streamoff()
        if self.background_frame_read is not None:
            self.background_frame_read.stop()
        if self.cap is not None:
            self.cap.release()
        
    


# In[8]:


class BackgroundFrameRead:
    """
    This class read frames from a VideoCapture in background. Then, just call backgroundFrameRead.frame to get the
    actual one.
    """

    def __init__(self, tello, address):
        tello.cap = cv2.VideoCapture(address)
        self.cap = tello.cap

        if not self.cap.isOpened():
            self.cap.open(address)

        self.grabbed, self.frame = self.cap.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update_frame, args=()).start()
        return self

    def update_frame(self):
        while not self.stopped:
            if not self.grabbed or not self.cap.isOpened():
                self.stop()
            else:
                (self.grabbed, self.frame) = self.cap.read()

    def stop(self):
        self.stopped = True


# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:





# In[ ]:




