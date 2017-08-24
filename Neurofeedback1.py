import struct, socket, string, threading, sched, time, serial, traceback, pyqtgraph.examples
from numpy import float
from struct import pack, unpack
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PyQt4.QtCore import QObject, pyqtSignal, pyqtSlot
from scipy import signal as sig

ser = serial.Serial(port='COM3', baudrate=230400, bytesize=8, parity='N', stopbits=1, timeout=1, xonxoff=0, rtscts=0, dsrdtr=0)
COMMAND_SET_MOTOR_SPEED = 0x90

WINDOW_LEN = 6000 # 6000 points tested ok for display
PLOT_DATA = [0.0]*WINDOW_LEN
TCP_IP = '127.0.0.1'
EMG_IP = '127.0.0.1'
HAND_IP = '192.168.1.109'
TCP_PORT = 50040
EMG_PORT = 50041 # only EMG data, each sample point includes 16 channels
SAMPLES_PER_READ = 27 # Delsys sends 27 sample points every read
BYTES_PER_CHANNEL = 4 # Delsys specs
CHANNELS_PER_SAMPLE = 16
BYTES_PER_SAMPLE = BYTES_PER_CHANNEL * CHANNELS_PER_SAMPLE
BUFFER_SIZE = int(BYTES_PER_SAMPLE * SAMPLES_PER_READ)
RD = [] # Raw data
NPFD = [] # N point average data
LFD = [] # Lfilter data
N_POINT_AVERAGE = 30
EMG_ACQUIRED = 0.0
EMG_Baseline = 0.010
EMG_Saturation = 0.1 # Above which the EMG will be truncated

NUM_BUTTER_ORDER = 2
CUT_OFF_FREQ = 0.03 # normalized
Lfilter_b, Lfilter_a = sig.butter(NUM_BUTTER_ORDER, CUT_OFF_FREQ, analog=False)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.connect((TCP_IP, TCP_PORT))
emg_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
emg_socket.connect((EMG_IP, EMG_PORT))
hand_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
data = server_socket.recv(BUFFER_SIZE)

### Send command "START" to Delsys
server_socket.send("START\r\n\r\n".encode('utf-8'))

####Initial the window
win = pg.GraphicsWindow()
win.setWindowTitle('EMG_Biofeedback')
p = win.addPlot()
p.setYRange(0,0.4)
curve = p.plot(PLOT_DATA)

# Save PLOT_DATA into a file
def save(filename, contents):
    fh = open(filename, 'w')
    fh.write(contents)
    fh.close()

class MySignal(QObject):
    sent = pyqtSignal()
    def __init__(self):
        QObject.__init__(self)
    def send(self):
        self.sent.emit()

@pyqtSlot()
def received():
    curve.setData(PLOT_DATA)

signal = MySignal()
signal.sent.connect(received)


NUM_REPEAT_READ = 10 # arbitrarily decided for smooth display, needs tweak
WANTED_CHANNEL = 1 # Delsys Trigo lab as 16 channels
ADDRESS_OFFSET = (WANTED_CHANNEL-1)*BYTES_PER_CHANNEL
def update():
    global PLOT_DATA, signal, NPFD, EMG_Baseline
    while True:
        # Temporary_Data = []
        for j in range(NUM_REPEAT_READ):
            Temporary_Data = []
            response = emg_socket.recv(BUFFER_SIZE)
            for i in range(SAMPLES_PER_READ):
                ADDRESS = BYTES_PER_SAMPLE*i + ADDRESS_OFFSET
                EMG_CHANNEL_1 = response[ADDRESS:(ADDRESS + BYTES_PER_CHANNEL)]
                emg = struct.unpack('f',EMG_CHANNEL_1)
                EMG_ACQUIRED = abs(1000*emg[0]) # 1000: arbitrary scaling factor
                RD.append(EMG_ACQUIRED)
                Temporary_Data.append(EMG_ACQUIRED)
                #########     N_POINT_AVERAGE   #########################################################
                PLOT_DATA = PLOT_DATA[1:] + [(sum(PLOT_DATA[WINDOW_LEN - N_POINT_AVERAGE:]) + EMG_ACQUIRED) / (N_POINT_AVERAGE + 1)]
            signal.send()
        #########     Positive proportional control   #########################################################
        MOTOR_SPEED = int((PLOT_DATA[WINDOW_LEN - 1] - EMG_Baseline )*(255/(EMG_Saturation - EMG_Baseline)))
        if MOTOR_SPEED <= 0:
            MOTOR_SPEED = 0
        if MOTOR_SPEED >= 255:
            MOTOR_SPEED = 255

        #########     Inverse proportional control   #########################################################
        # MOTOR_SPEED = 255 - int((PLOT_DATA[5999] - EMG_Baseline )*(255/(EMG_Saturation - EMG_Baseline)))
        # if MOTOR_SPEED <= 0:
        #     MOTOR_SPEED = 0
        # if MOTOR_SPEED >= 255:
        #     MOTOR_SPEED = 255

        msg = pack('BB', COMMAND_SET_MOTOR_SPEED, MOTOR_SPEED)
        ser.write(msg)

        NPFD = NPFD + PLOT_DATA[WINDOW_LEN - NUM_REPEAT_READ * SAMPLES_PER_READ:]
        #######         Lfilter         #############################################################
        # PLOT_DATA = PLOT_DATA[270:] + sig.lfilter(Lfilter_b, Lfilter_a, sig.lfilter(Lfilter_b, Lfilter_a, Temporary_Data)).tolist()
        # signal.send()

threads = []
t1 = threading.Thread(target=update)
threads.append(t1)

#The main function,loop forever.
if __name__ == '__main__':
    t1.setDaemon(True)
    t1.start()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

MOTOR_SPEED = 0
msg = pack('BB', COMMAND_SET_MOTOR_SPEED, MOTOR_SPEED)
ser.write(msg)
ser.close()

server_socket.close()
emg_socket.close()

RD = str(RD)
NPFD = str(NPFD)
# LFD = str(LFD)d4
save('5_16_5_Normal_RAW_DATA.txt',RD)
save('5_13_5_Normal_N_POINT_FILTER_DATA.txt',NPFD)
# save('LFILTER_DATA.txt',LFD)
