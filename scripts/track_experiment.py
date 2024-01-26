import collections
import random
import time
import math
import numpy as np
import csv
import sys
import os

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QPushButton,
    QWidget,
    QLabel,
    QFileDialog,
    QComboBox
)

class CustomPlotWidget(pg.PlotWidget):
    def __init__(self, label = None):
        pg.PlotWidget.__init__(self)

        self.showAxis('left', False)
        self.showAxis('right', True)
        self.showGrid(x=True, y=True, alpha=0.5)
        self.setLabels(**label)


class Window(QWidget):

    def __init__(self, sampleinterval=0.5, timewindow=10., size=(600,350)):
        QWidget.__init__(self)
        self.setWindowTitle("TLAG - Track Experiment")

        # Data stuff
        self._interval = int(sampleinterval*1000)
        self._bufsize = int(timewindow/sampleinterval)
        self.databuffer = collections.deque([0.0]*self._bufsize, self._bufsize)

        # PyQtGraph stuff
        self.app = QtGui.QApplication([])
        # self.plt = pg.plot(title='Dynamic Plotting with PyQtGraph')
        # self.plt.resize(*size)
        # self.plt.showGrid(x=True, y=True)
        # self.plt.setLabel('left', 'amplitude', 'V')
        # self.plt.setLabel('bottom', 'time', 's')
        # self.curve1 = self.plt.plot(self.x, self.y, pen=(0,255,0))
        # self.curve2 = self.plt.plot(self.x, self.y, pen=(0,255,0))
        # self.curve3 = self.plt.plot(self.x, self.y, pen=(0,0,255))

        # win = pg.QGridLayout()

        # win = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")
        # win.resize(1000,600)
        # win.setWindowTitle('pyqtgraph example: Plotting')

        layout = QGridLayout()

        # pw = pg.PlotWidget()
        # pw.show()
        # pw.setWindowTitle('pyqtgraph example: MultiplePlotAxes')
        # p1 = pw.plotItem
        # p1.setLabels(left='axis 1')
        

        ## create a new ViewBox, link the right axis to its coordinate system
        # p2 = pg.ViewBox()
        # p1.showAxis('right')
        # p1.scene().addItem(p2)
        # p1.getAxis('right').linkToView(p2)
        # p2.setXLink(p1)
        # p1.getAxis('right').setLabel('axis2', color='#0000ff')

        self.label = QLabel()
        self.label.setAlignment(QtCore.Qt.AlignRight)
        # self.label.addItem(pg.LabelItem(justify='right'))
        layout.addWidget(self.label, 0, 0, 1, 3)

        self.current_file = "740.csv"
        self.new_file = False
        self.common_path = "/homelocal/opbl11/Documents/TLAG/tlag_manager/results/"

        # Define number of curves to updates
        self.num_curves = 3

        self.files = [
            self.common_path + "temperature/temperature_" + self.current_file,
            self.common_path + "temperature/temperature_" + self.current_file,
            self.common_path + "pressure/pressure_" + self.current_file
        ]

        self.file_log = self.common_path + "logs/log_" + self.current_file.split('.')[0] + '.log'
        
        print(self.files,
              self.file_log)

        # Read t0
        self.t0 = self.get_t0()

        # self.files = [
        #     "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/dhs1100/results/{}".format(self.current_file),
        #     "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/dhs1100/results/{}".format(self.current_file),
        #     "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/Pressure_furnace/{}".format(self.current_file)
        # ]

        labels = [
            {"bottom" : "Time (seconds)", "right" : "Temperature (ºC)"},
            {"bottom" : "Time (seconds)", "right" : "Power (%)"},
            {"bottom" : "Time (seconds)", "right" : "Pressure (mbar)"}
        ]

        self.rows = [(0, 1), (0, 3), (0,2,1)]
        self.legend = ["Temperature", "Power", "Pressure"]

        
        # self.plots = [pg.PlotWidget(title = "{}".format(i)) for i in range(self.num_curves)]
        self.plots = [CustomPlotWidget(labels[i]) for i in range(self.num_curves)]
        
        # Link X axes (time) of all plots
        for i in range(self.num_curves - 1):
           self.plots[i].setXLink(self.plots[i+1])

        # Logarithmic y axes to third plot (pressure)
        self.plots[-1].setLogMode(False, True)
        self.plots[-1].setYRange(-2, 2, 0.04)

        self.colors = [(255,0,0),(0,255,0),(0,0,255)]
        self.ys = [np.zeros(self._bufsize, dtype=float) for _ in range(self.num_curves)]
        self.xs = [np.linspace(-timewindow, 0.0, self._bufsize) for _ in range(self.num_curves)]
        self.curves = [self.plots[i].plot(self.xs[i], self.ys[i], pen=self.colors[i]) for i in range(self.num_curves)] # self.plots[i]
        self.current_indices = [0 for _ in range(self.num_curves)]

        #Labels reles
        self.label_move = QLabel("MOVE")
        self.label_rotary = QLabel("ROTARY")
        self.label_turbo = QLabel("TURBO")

        self.label_rotary.setAlignment(QtCore.Qt.AlignCenter)
        self.label_turbo.setAlignment(QtCore.Qt.AlignCenter)
        self.label_move.setAlignment(QtCore.Qt.AlignCenter)
        
        # combobox to select the file to read
        self.combobox_file = QComboBox()
        self.update_combobox()
        self.combobox_file.activated.connect(self.select_combobox)


        # Button to update the combobox in case new files are created
        self.button_update_combobox = QPushButton("Update file list")
        self.button_update_combobox.clicked.connect(self.update_combobox)


        # Create plot
        
        for i, plot in enumerate(self.plots):
            layout.addWidget(plot, i+1, 0, 1, 3)

        layout.addWidget(self.label_move, i+2, 0, 1, 1)
        layout.addWidget(self.label_rotary, i+2, 1, 1, 1)
        layout.addWidget(self.label_turbo, i+2, 2, 1, 1)


        layout.addWidget(self.combobox_file, i+3, 0, 1, 3)
        layout.addWidget(self.button_update_combobox, i+4, 0, 1, 3)
        
        self.setLayout(layout)
        
        # QTimer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.updateplot)
        self.timer.setInterval(self._interval)
        self.timer.start()

    def get_t0(self):
        with open(self.files[0], 'r') as f:
            reader=csv.reader(f, delimiter=',')
            # We won't use header, but we do this because we need only the data
            header = next(reader)

            reader_list = list(reader)

            return float(reader_list[0][0])

    def update_combobox(self):
        files_dir = os.listdir(self.common_path + "temperature/")
        files_dir = sorted([file.split('_')[-1] for file in files_dir])
        # files_dir = os.listdir("/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/dhs1100/results/")
        self.combobox_file.clear()
        self.combobox_file.addItems(files_dir)

    def select_combobox(self):
        print(self.combobox_file.currentText())
        self.current_file = self.combobox_file.currentText()

        self.files = [
            self.common_path + "temperature/temperature_" + self.current_file,
            self.common_path + "temperature/temperature_" + self.current_file,
            self.common_path + "pressure/pressure_" + self.current_file
        ]

        self.file_log = self.common_path + "logs/log_" + self.current_file.split('.')[0] + '.log'

        self.new_file = True
        self.t0 = self.get_t0()

        # self.files = [
        #     "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/dhs1100/results/{}".format(self.current_file),
        #     "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/dhs1100/results/{}".format(self.current_file),
        #     "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA/tests/Pressure_furnace/{}".format(self.current_file)
        # ]

    def getdata(self):
        frequency = 0.5
        noise = random.normalvariate(0., 1.)
        new = 10.*math.sin(time.time()*frequency*2*math.pi) + noise
        return new

    def read_new_lines(self, file, current_idx, rows, pressure = False): # file, current_index
        # file = r"C:\Users\Jordi\Documents\ICMAB\TLAG-tool\tests\dhs1100\results\test2_temp_power_pressure.csv"

        with open(file) as fd:
            reader=csv.reader(fd, delimiter=',')
            # We won't use header, but we do this because we need only the data
            header = next(reader)

            reader_list = list(reader)

            n = len(reader_list)
            if n < current_idx or self.new_file:
                new_idx = 0
                self.new_file = False
                self.current_indices = [0, 0, 0]
            else:
                new_idx = current_idx


            if not pressure:
                interestingrows=[(float(row[rows[0]]), float(row[rows[1]])) for idx, row in enumerate(reader_list) if idx >= new_idx]
            else:
                interestingrows=[(float(row[rows[0]]), float(row[rows[1]]), float(row[rows[2]])) for idx, row in enumerate(reader_list) if idx >= new_idx]
            
            x = [selected_rows[0] for selected_rows in interestingrows]

            if not pressure:
                y = [selected_rows[1] for selected_rows in interestingrows]
            else:
                y = [selected_rows[1] if selected_rows[2] > 1 else selected_rows[2] for selected_rows in interestingrows]

        return x, y, new_idx
    
    def diff_array(self, arr):
        
        new_arr = np.zeros(len(arr)-1)
        for i in range(len(arr)-1):
            new_arr[i] = arr[i+1] - arr[i]
            
        return new_arr
    
    def derivative_window(self, x, y, n_points = 5):
        # print(self.xs[2][-n_points:], self.ys[2][self.current_indices[2]-1-n_points:self.current_indices[2]-1])
        diffx = self.diff_array(self.xs[2][self.current_indices[2]-1-n_points:self.current_indices[2]-1])
        diffy = self.diff_array(self.ys[2][self.current_indices[2]-1-n_points:self.current_indices[2]-1])
        
        # print(diffx, diffy)
        
        dy = [dy_value/dx_value for dx_value, dy_value in zip(diffx, diffy)]
        # print(dy)
        return np.mean(dy)
        
            

    def update_text(self):
        # Update label
        span_style_size = "<span style='font-size: 18pt'>"
        text_temperature = "<span style='color: red'>Temperature={:0.2f} ºC, </span>".format(self.ys[0][self.current_indices[0]-1])
        text_power = "<span style='color: green'>Power={:0.2f} %, </span>".format(self.ys[1][self.current_indices[1]-1])
        text_pressure = "<span style='color: DeepSkyBlue'>Pressure={:0.3f} mbar </span>".format(self.ys[2][self.current_indices[2]-1])
        dP = self.derivative_window(self.xs[2], self.ys[2])
        text_dpressure = "<span style='color: DeepSkyBlue'>dP={:0.3f} mbar/s </span>".format(dP)
        text_time = "<span style='color: gregrayen'>Time={:0.1f} seconds, </span>".format(self.xs[1][self.current_indices[1]-1] - self.t0)
        span_close = "<\span>"
        self.label.setText(span_style_size + text_time + text_temperature + text_power+ text_pressure + text_dpressure + span_close) #(data1[index], data2[index])

    def update_relay_move(self):

        turbo_state = None
        rotary_state = None
        move_position = None

        with open(self.file_log) as f:
            for line in f.readlines():
                # We won't use header, but we do this because we need only the data
                message = line.split('-')[-1].split()
                if message[0] == 'rotary':
                    if message[-1] == 'closed':
                        rotary_state = False
                    elif message[-1] == 'open':
                        rotary_state = True
                elif message[0] == 'turbo':
                    if message[-1] == 'closed':
                        turbo_state = False
                    elif message[-1] == 'open':
                        turbo_state = True
                        
                elif message[0] == 'MOVE':
                    move_position = message[-1]
                    move_position = round(float(move_position), 1)

        if turbo_state is not None:
            if turbo_state:
                self.label_turbo.setStyleSheet("background-color: lightgreen")
            else:
                self.label_turbo.setStyleSheet("background-color: red")
        else:
            self.label_turbo.setStyleSheet("background-color: ")
        
        if rotary_state is not None:
            if rotary_state:
                self.label_rotary.setStyleSheet("background-color: lightgreen")
            else:
                self.label_rotary.setStyleSheet("background-color: red")
        else:
            self.label_rotary.setStyleSheet("background-color: ")
        """
        if rotary_state is not None:
            if rotary_state and move_position > 7.57:
                self.label_rotary.setStyleSheet("background-color: lightgreen")
            elif rotary_state:
                self.label_rotary.setStyleSheet("background-color: darkorange")
            else:
                self.label_rotary.setStyleSheet("background-color: red")
        else:
            self.label_rotary.setStyleSheet("background-color: ")
        """
        if move_position is not None:
            self.label_move.setText("MOVE ({}%)".format(move_position))
            if move_position > 7.57:
                self.label_move.setStyleSheet("background-color: lightgreen")
            else:
                self.label_move.setStyleSheet("background-color: red")
        





    def updateplot(self):
        for i in range(self.num_curves):
            self.updatecurve(i)

        # Update label
        self.update_text()

        # Update relay state
        self.update_relay_move()


    def updatecurve(self, i): # file, curve, currend_idx

        curr_file = self.files[i]
        curr_idx = self.current_indices[i]
        curr_rows = self.rows[i]

        pressure = False
        if i == 2:
            pressure = True

        x, y, curr_idx = self.read_new_lines(curr_file, curr_idx, curr_rows, pressure)
        len_new_data = len(x)

        if curr_idx == 0:
            # We have reseted the file
            self.current_indices[i] = 0
            self.ys[i] = np.zeros(self._bufsize, dtype=float)
            self.xs[i] = np.zeros(self._bufsize, dtype=float)

        if i == 0 and curr_idx % 10 == 0:
            print(curr_idx, len_new_data)

        if curr_idx+len_new_data >= self.ys[i].shape[0]:
            tmpy = self.ys[i]
            self.ys[i] = np.zeros((curr_idx+len_new_data) * 10)
            self.ys[i][:tmpy.shape[0]] = tmpy

            tmpx = self.xs[i]
            self.xs[i] = np.zeros((curr_idx+len_new_data) * 10)
            self.xs[i][:tmpx.shape[0]] = tmpx
            
        self.xs[i][curr_idx:(curr_idx + len_new_data)] = x
        self.ys[i][curr_idx:(curr_idx + len_new_data)] = y

        # Resize X range only for one axes since all the others are connected
        if len_new_data > 0 and i == 0:
            current_value = max(self.xs[i] - self.t0)
            self.plots[i].setXRange(max(0, current_value - 500), current_value)

        # self.x = self.x + self._interval
        self.curves[i].setData(self.xs[i][:(curr_idx + len_new_data)] - self.t0, self.ys[i][:(curr_idx + len_new_data)])
        self.app.processEvents()

        self.current_indices[i] += len_new_data

        
    def run(self):
        self.app.exec_()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
