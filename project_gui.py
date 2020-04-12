from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import random
import pyqtgraph.opengl as gl
import numpy as np
import socket
import sys
from _thread import *
import time
from queue import *
from struct import *
import math
from collections import namedtuple
import Classes

# This is setting up the socket for the server

host = ''
port = 5555
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    s.bind((host, port))
except socket.error as e:
    print(str(e))


# This is the main GUI class

class MainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.central_widget = QtGui.QStackedWidget()
        self.setCentralWidget(self.central_widget)

        self.gui = guiInterface(self)

        self.gui.button1.clicked.connect(self.start)
        self.central_widget.addWidget(self.gui)

        self.gui.button2.clicked.connect(self.startRecord)
        self.central_widget.addWidget(self.gui)

        self.gui.button3.clicked.connect(self.stopRecord)
        self.central_widget.addWidget(self.gui)

        self.gui.button4.clicked.connect(self.resetButtonPressed)
        self.central_widget.addWidget(self.gui)


        self.conn = list()
        self.addr = list()
        self.stringQueue = list()
        self.dataQueue = list()
        self.commandQueue = Queue()
        self.index = 0
        self.startSwitch = False

        self.n = 1
        self.m = 3
        self.y = 0
        self.x = np.array([0,0,0])
        self.scaleFactor = 10
        self.rotationFactor = -1

        self.positions = list()
        self.positions.append(Queue())
        self.positions.append(Queue())


        yi = np.array([0,0,0])
        z = np.array([0,0.5*self.scaleFactor,1*self.scaleFactor])
        pts = np.vstack([self.x, yi, self.rotationFactor*z]).transpose()
        self.trace = gl.GLLinePlotItem(pos=pts, color=(255, 0, 0, 255), width=10, antialias=True)
        self.gui.w.addItem(self.trace)

    def resetButtonPressed(self):
        self.gui.text_edit_widget.appendPlainText("Resetting")
        self.commandQueue.put("reset")        

    def start(self):

        if self.startSwitch == False:

            self.startSwitch = True
            s.listen(5)
            self.gui.text_edit_widget.appendPlainText("Waiting for a connection.")
            QtGui.QApplication.processEvents()

            for i in range(Classes.NUMBER_OF_NODES):
                tempconn, tempaddr = s.accept()
                self.conn.append(tempconn)
                self.addr.append(tempaddr)
                tempString = 'connected to: '+self.addr[self.index][0]+':'+str(self.addr[self.index][1])
                self.gui.text_edit_widget.appendPlainText(tempString)
                QtGui.QApplication.processEvents()
                self.stringQueue.append(Queue())
                self.dataQueue.append(Queue())

                self.index = self.index + 1
                if self.index == Classes.NUMBER_OF_NODES:
                    break
        
            for i in range(Classes.NUMBER_OF_NODES):
                reply = 'Start'
                self.conn[i].sendall(str.encode(reply))
        
            self.gui.text_edit_widget.appendPlainText("Starting the reciever and analyser")
            
            start_new_thread(Classes.threaded_client,(self.conn,self.stringQueue))
            start_new_thread(Classes.threaded_reader,(self.stringQueue,self.dataQueue))
            start_new_thread(Classes.threaded_forward_kinematic,(self.dataQueue,self.commandQueue,self.positions))
            #start_new_thread(self.updater,(self.commandQueue,))
            self.gui.text_edit_widget.appendPlainText("Starting the plotter")


    def set_plotdata(self, points, color, width):
        self.trace.setData(pos=points, color=color, width=width)

    def startRecord(self):
        self.gui.text_edit_widget.appendPlainText("Starting recording")
        self.commandQueue.put("startRecord")
        
    def stopRecord(self):
    	self.gui.text_edit_widget.appendPlainText("Stopping recording")
    	self.commandQueue.put("stopRecord")

    def updater(self,q):

        while True:
            if not self.positions[0].empty():
                elbowTempPos = self.positions[0].get()
                wristTempPos = self.positions[1].get()
                xi = np.array([0,elbowTempPos.x*10,wristTempPos.x*10], dtype='f')
                yi = np.array([0,elbowTempPos.y*10,wristTempPos.y*10], dtype='f')
                zi = np.array([0,elbowTempPos.z*10,wristTempPos.z*10], dtype='f')
                pts = np.vstack([xi, yi, zi*self.rotationFactor]).transpose()
                self.set_plotdata(
                    points=pts,
                    color=(255, 0, 0, 255),
                    width=10
                )
            time.sleep(0.05)

# This class handles setting up the GUI interface and layout
class guiInterface(QtGui.QWidget):
    def __init__(self, parent=None):
        super(guiInterface, self).__init__(parent)
        #layout = QtGui.QHBoxLayout()
        layout = QtWidgets.QGridLayout()

        button_layout = QtWidgets.QVBoxLayout()

        self.button1 = QtGui.QPushButton('Start Server')
        button_layout.addWidget(self.button1)
        self.button2 = QtGui.QPushButton('Start Record')
        button_layout.addWidget(self.button2)
        self.button3 = QtGui.QPushButton('Stop Record')
        button_layout.addWidget(self.button3)
        self.button4 = QtGui.QPushButton('Reset arm position')
        button_layout.addWidget(self.button4)

        layout.addLayout(button_layout, 0, 1)

        self.text_edit_widget = QtGui.QPlainTextEdit()

        # Change font, colour of text entry box
        self.text_edit_widget.setStyleSheet(
            """QPlainTextEdit {background-color: #333;
                               color: #00FF00;
                               text-decoration: underline;
                               font-family: Courier;}""")
        
        #layout.addWidget(self.text_edit_widget)
        layout.addWidget(self.text_edit_widget, 1, 0)  
        self.text_edit_widget.appendPlainText("This is a Status Log")    

        self.w = gl.GLViewWidget()
        self.w.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        self.w.opts['distance'] = 40
        self.w.setWindowTitle('Visualiser')
        #self.w.setGeometry(0, 110, 1920, 1080)
        self.w.show()

        # create the background grids
        gx = gl.GLGridItem()
        gx.rotate(90, 0, 1, 0)
        gx.translate(-10, 0, 0)
        self.w.addItem(gx)
        gy = gl.GLGridItem()
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -10, 0)


        self.w.addItem(gy)
        gz = gl.GLGridItem()
        gz.translate(0, 0, -10)
        self.w.addItem(gz)

        axis = gl.GLAxisItem()
        axis.setSize(3,3,3)
        self.w.addItem(axis)


        #layout.addWidget(self.w)
        layout.addWidget(self.w, 0, 0) 

        layout.setColumnStretch(0, 2)
        layout.setRowStretch(0, 2)

        self.setLayout(layout)

if __name__ == '__main__':
    app = QtGui.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()