import time
import ast

import tango

import sys
#sys.path.insert(1, '/home/jalarruy/Documents/ICMAB/TLAG-tool/')
# sys.path.insert(1, "/homelocal/opbl11/Documents/TLAG/TLAG-tool_SAFA")
# from instruments import ElFlowSelect

# import matplotlib.pyplot as plt


from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QLabel,
    QComboBox,
    QLineEdit,
    QWidget,
    QDialog,
    QMessageBox,
    QFrame,
    QMainWindow, 
    QAction, 
    QTableWidget,
    QTableWidgetItem,
)

from PyQt5.QtGui import (
    QColor,
    QStandardItemModel,
)

from PyQt5.QtCore import Qt, pyqtSignal, QThread


#List available resources
# rm = pyvisa.ResourceManager()
# print(rm.list_resources())

# massflow = ElFlowSelect("COM3")
"""
serialNum2flow = {
    "M21211517B" : 10,
    "M21211517A" : 800,
    "M22200939A" : 250,
    "M15208965A" : 3000
}
flow2serialNum = {value : key for key, value in serialNum2flow.items()}
"""
class Mythread(QThread):
    #  Define signal , Define parameters as str type 
    breakSignal = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.threadactive = True

    def run(self):
        # Behavior to define, such as starting 1 Activity or something 

        while self.threadactive: 
            self.breakSignal.emit()
            time.sleep(1)

    def stop(self):
        self.threadactive = False
        self.wait()

class App(QWidget):

    def __init__(self):
        super().__init__()
        self.title = 'Massflow Controller'
        self.left = 0
        self.top = 0
        # self.width = 325
        # self.height = 380

        self.air = False
        if self.air:
            self.air_or_oxygen = "Air"
        else:
            self.air_or_oxygen = "Oxygen"
        
        self.massflow = tango.DeviceProxy("tlag/ex/massflowbus-01")
        self.serialNum2node = ast.literal_eval(self.massflow.serialNums2node_str())
        # self.massflows_controlled = ["10", "250", "800"] # , "3000"
        self.massflows_controlled = ['M21211517B', 'M21211517A', 'M22200939A']
        
        # Change init reset of massflows to write locked parameters
        
        """
        for flow in self.massflows_controlled:
            node = self.massflow.flow2node[int(flow)]
            self.massflow.request_init_reset(node=node)
            self.massflow.change_init_reset(node=node, mode="unlocked")
            self.massflow.request_init_reset(node=node)
            self.massflow.change_pressure("inlet", node, 3.01)
        """
        
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle(self.title)
        # self.setGeometry(self.left, self.top, self.width, self.height)
        
        self.createTable()

        self.createButtonClose()

        self.createControlPanel()

        # Add box layout, add table to box layout and add box layout to widget
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.tableWidget)
        self.layout.addWidget(self.buttonCloseWidget)
        self.layout.addLayout(self.HBox)
        self.setLayout(self.layout)

        # Start reader
        self.thread = Mythread()
        self.thread.breakSignal.connect(self.update_table)
        self.thread.start()

        # Start alarm in case measure < 0.9 * setpoint
        self.thread_alarm = Mythread()
        self.thread_alarm.breakSignal.connect(self.checkalarm)
        self.thread_alarm.start()

        # Show widget
        self.show()

    def createTable(self):
       # Create table
        header_names = ["Max capacity (ml/min)", "Setpoint (%)", "Measure (%)", "Fsetpoint (ml/min)", "Fmeasure (ml/min)", "Valve Output (%)", "Inlet Pressure (bar)", "Outlet Pressure (bar)"]
        row_names = ["Massflow {}".format(flow) for flow in self.massflows_controlled]

        self.tableWidget = QTableWidget()
        self.tableWidget.setRowCount(len(row_names))
        self.tableWidget.setColumnCount(len(header_names))

        self.tableWidget.setHorizontalHeaderLabels(header_names)
        self.tableWidget.setVerticalHeaderLabels(row_names)

        
        for i in range(len(row_names)):
            for j in range(len(header_names)):
                self.tableWidget.setItem(i,j, QTableWidgetItem("Cell ({},{})".format(i, j)))
        
        self.tableWidget.move(0,0)

    def createButtonClose(self):
        self.buttonCloseWidget = QPushButton("Close all massflows")
        self.buttonCloseWidget.clicked.connect(self.closeAllMassflows)

    def createControlPanel(self):
        self.HBox = QHBoxLayout()
        
        self.createConfigurationPanel()

        self.createStatusPanel()

        self.HBox.addWidget(self.widget1, 0)
        self.HBox.addWidget(self.widget2, 1)

    def createConfigurationPanel(self):
        self.widget1 = QWidget()
        self.widget1.setProperty("coloredcell", True)
        self.widget1.setStyleSheet("*[coloredcell=\"true\"] {background-color:rgb(255,82,82);}")

        self.gridLayout = QGridLayout(self.widget1)

        # Left column
        self.gridLayout.addWidget(QLabel("Configure:"), 1, 0, 1, 2, Qt.AlignCenter)
        self.gridLayout.addWidget(QLabel("{}:".format(self.air_or_oxygen)), 2, 0, Qt.AlignRight)
        self.gridLayout.addWidget(QLabel("Nitrogen:"), 3, 0, Qt.AlignRight)
        self.gridLayout.addWidget(QLabel("Total Pressure:"), 4, 0, Qt.AlignRight)
        self.gridLayout.addWidget(QLabel("PO2:"), 5, 0, Qt.AlignRight)
        self.gridLayout.addWidget(QLabel("Total Flow:"), 6, 0, Qt.AlignRight)

        # Right column
        self.airCombo = QComboBox()
        self.airCombo.addItems(["None"] + self.massflows_controlled)
        self.gridLayout.addWidget(self.airCombo, 2, 1)

        self.nitrogenCombo = QComboBox()
        self.nitrogenCombo.addItems(["None"] + self.massflows_controlled)
        self.gridLayout.addWidget(self.nitrogenCombo, 3, 1)

        self.lineTotalPressure = QLineEdit(self)
        self.gridLayout.addWidget(self.lineTotalPressure, 4, 1)

        self.linePO2 = QLineEdit(self)
        self.gridLayout.addWidget(self.linePO2, 5, 1)

        self.lineTotalFlow = QLineEdit(self)
        self.gridLayout.addWidget(self.lineTotalFlow, 6, 1)

        self.buttonApply = QPushButton("Apply")
        self.gridLayout.addWidget(self.buttonApply, 7, 0, 7, 2)
        self.buttonApply.clicked.connect(self.set_partial_pressure)

    def createStatusPanel(self):
        self.widget2 = QWidget()
        self.widget2.setProperty("coloredcell", True)
        self.widget2.setStyleSheet("*[coloredcell=\"true\"] {background-color:rgb(190,190,190);}")

        self.gridLayout2 = QGridLayout(self.widget2)

        # Left column
        self.gridLayout2.addWidget(QLabel("Current Status:"), 1, 0, 1, 2, Qt.AlignCenter)
        self.gridLayout2.addWidget(QLabel("{}:".format(self.air_or_oxygen)), 2, 0, Qt.AlignRight)
        self.gridLayout2.addWidget(QLabel("Nitrogen:"), 3, 0, Qt.AlignRight)
        self.gridLayout2.addWidget(QLabel("Total Pressure:"), 4, 0, Qt.AlignRight)
        self.gridLayout2.addWidget(QLabel("PO2:"), 5, 0, Qt.AlignRight)
        self.gridLayout2.addWidget(QLabel("Total Flow:"), 6, 0, Qt.AlignRight)

        # Right column
        self.air_status = QLabel("Massflow -")
        self.nitrogen_status = QLabel("Massflow -")
        self.total_pressure_status = QLabel("- bar")
        self.po2_status = QLabel("- bar")
        self.total_flow_status = QLabel("- ml/min")
        self.filler = QLabel("")
        self.gridLayout2.addWidget(self.air_status, 2, 1, Qt.AlignCenter)
        self.gridLayout2.addWidget(self.nitrogen_status, 3, 1, Qt.AlignCenter)
        self.gridLayout2.addWidget(self.total_pressure_status, 4, 1, Qt.AlignCenter)
        self.gridLayout2.addWidget(self.po2_status, 5, 1, Qt.AlignCenter)
        self.gridLayout2.addWidget(self.total_flow_status, 6, 1, Qt.AlignCenter)
        self.gridLayout2.addWidget(self.filler, 7, 1, Qt.AlignCenter)

    def closeAllMassflows(self):
        self.massflow.close_all_massflows()
        self.reset_status()
        self.update_table()

    def reset_status(self):
        try:
            self.air_status.setText("Massflow -")
            self.nitrogen_status.setText("Massflow -")
            self.total_pressure_status.setText("- bar")
            self.po2_status.setText("- bar")
            self.total_flow_status.setText("- ml/min")
        except:
            self.throw_error("Unexpected error ocurred when resetting status")

    def update_table(self):
        try:
            for i, serialNum in enumerate(self.massflows_controlled):
                # node = self.massflow.flow2node
                node = self.serialNum2node[serialNum]
                setpoint = round(self.massflow.request_setpoint(node) * 100, 2)
                measure =  round(self.massflow.request_measure(node) * 100, 2)
                fsetpoint = round(self.massflow.request_fsetpoint(node), 2)
                fmeasure =  round(self.massflow.request_fmeasure(node), 2)
                valveoutput = round(self.massflow.request_valve_output(node) * 100, 2)
                capacity = round(self.massflow.request_capacity(node), 2)
                inlet_pressure = round(self.massflow.request_inlet_pressure(node), 2)
                outlet_pressure = round(self.massflow.request_outlet_pressure(node), 2)
                
                
                # setpoint = 0 # round(random.random(), 2)
                # measure = 0 # round(random.random(), 2)
                column_oder = [capacity, setpoint, measure, fsetpoint, fmeasure, valveoutput, inlet_pressure, outlet_pressure]
                self.n_columns = len(column_oder)
                for j, value in enumerate(column_oder):
                    self.tableWidget.setItem(i, j, QTableWidgetItem(str(value)))
                # self.tableWidget.setItem(i, 8, QTableWidgetItem(str(init_reset)))
                # self.massflow.change_outlet_pressure(node, 3)

            # time.sleep(1)
        except:
            self.thread.stop()
            self.throw_error("Unexpected error ocurred when updating table. Reopen the application.")

    def checkalarm(self):
        try:
            for i, serialNum in enumerate(self.massflows_controlled):
                setpoint = float(self.tableWidget.item(i, 1).text())
                measure = float(self.tableWidget.item(i, 2).text())

                if measure < 0.9*setpoint:
                    for j in range(self.n_columns):
                        self.tableWidget.item(i, j).setBackground(QColor(255,50,50))
                else:
                    for j in range(self.n_columns):
                        self.tableWidget.item(i, j).setBackground(QColor(255,255,255))
        except Exception as e:
            print(e)


    def set_partial_pressure(self):
        # Read inputs
        try:
            air_massflow = self.airCombo.currentText()
            nitrogen_massflow = self.nitrogenCombo.currentText()
            total_pressure = float(self.lineTotalPressure.text())
            po2 = float(self.linePO2.text())
            total_flow = float(self.lineTotalFlow.text())
        except:
            self.throw_error("Something went wrong when processing the inputs")
            return None
        
        # Configure massflows
        try:
            argin_gas2serialNum = (air_massflow, nitrogen_massflow) 
            self.massflow.set_gas2serialNum(argin_gas2serialNum)
            argin_set_gas_partial_pressure = ([po2, total_flow, total_pressure, int(self.air)], ["oxygen"])
            message = self.massflow.set_gas_partial_pressure(argin_set_gas_partial_pressure)
        except Exception as e:
            self.throw_error(str(e))
            return None

        # If success update status
        self.air_status.setText("Massflow {}".format(air_massflow))
        self.nitrogen_status.setText("Massflow {}".format(nitrogen_massflow))
        self.total_pressure_status.setText("{} bar".format(total_pressure))
        self.po2_status.setText("{} bar".format(po2))
        self.total_flow_status.setText("{} ml/min".format(total_flow))

        # If success clear selection
        self.airCombo.clear()
        self.airCombo.addItems(["None"] + self.massflows_controlled)

        self.nitrogenCombo.clear()
        self.nitrogenCombo.addItems(["None"] + self.massflows_controlled)

        self.lineTotalPressure.clear()

        self.linePO2.clear()
        
        self.lineTotalFlow.clear()

        
    def throw_error(self, message = None):
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Trouble!")
        dlg.setIcon(QMessageBox.Question)
        if message is not None:
            dlg.setText(message)
        else:
            dlg.setText("Something went wrong!")
        
        dlg.exec()

 
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())  
