import sys
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
import serial.tools.list_ports
from mainwindow import Ui_MainWindow
import PyQt5.QtCore as QtCore
from SerialConnection import Serial_Connection
from PyQt5.QtCore import QIODevice
from PyQt5.QtSerialPort import QSerialPort, QSerialPort, QSerialPortInfo
from PyQt5.QtGui import QTextCursor, QFont
import serial
import numpy as np
import os

class Serial_Main_Backend(QMainWindow):
    def __init__(self,apps):
        global setted_combobox
        setted_combobox = []

        # call constructor from QMainWindow class
        super().__init__()
        # initialize the main window created with the Qt 5 Designer
        self.window = QMainWindow()
        absolute_path = os.path.dirname(__file__)
        relative_path = r"SyNet.qss"
        full_path = os.path.join(absolute_path, relative_path)
        File = QtCore.QFile(full_path)
        if File.open(QtCore.QFile.ReadOnly | QtCore.QFile.Text):
            qss = QtCore.QTextStream(File)
            # setup stylesheet
            apps.setStyleSheet(qss.readAll())
        self.ui = Ui_MainWindow()
        self.window.setFixedSize(1300, 900)
        self.window.setWindowFlags(QtCore.Qt.FramelessWindowHint)

        width = self.window.frameGeometry().width()
        height = self.window.frameGeometry().height()
        self.ui.define_width_and_height(width, height)
        self.ui.setupUi(self.window)
        self.ui.pushButton_Start.setCheckable(True)
        self.ui.label_Status.setFont(QFont('Arial', 20))

        self.ui.label_current_warnings.setStyleSheet("QLabel { color : Green; font-size: 20px;}")
        self.ui.label_current_warnings.setFont(QFont('Arial', 20))
        self.ui.label_Devices_available.setFont(QFont('Arial', 20))
        self.ui.pushButton_close.clicked.connect(self.ui.close_window)
        self.ui.pushButton_Start.setDisabled(True)
        self.window.show()

        # init placeholder for serial object
        self.__serial = None
        # 1: create a connection from the Button clicked signal to the
        # according function that should be executed then
        self.ui.pushButton_Start.clicked.connect(self.serial_connect)
        # 2: create a connection from the send button clicked signal to the
        # according function that sends the data
        # self.ui.sendButton.clicked.connect(self.send_data_serial)
        self.check_available_ports()
        self.list_of_curves = []
        self.plot_data = []
        self.plot_ptr = []
        self.init_plot_data_and_ptr()
        self.init_curves_to_plots()

    def init_plot_data_and_ptr(self):
        for i in range(0,10):
            setted_combobox.append(False)
            self.plot_data.append(np.zeros(50))
            self.plot_ptr.append(0)

    def init_curves_to_plots(self):
        a = len(self.ui.nodes_widgets_generaL_list)
        nodes = self.ui.nodes_widgets_generaL_list
        for i in range(0,a):
            node = nodes[i]
            self.list_of_curves.append(node[5].plot(self.plot_data[i]))


    def serial_connect(self):
        # ’’’
        # This function gets called when the connect button was pressed.
        # The serial connection should then be initialized with the currently
        # in the ComboBox selected device.
        # ’’’
        if self.ui.pushButton_Start.isChecked():
            self.ui.pushButton_Start.setText("disconnect")
            port = self.ui.comboBox_nodenumbers.currentText()
            port1 = self.connected[0]
            self._serial = QSerialPort()
            self._serial.setPortName(port1)
            self._serial.setBaudRate(QSerialPort.Baud115200)
            self._serial.setDataBits(QSerialPort.Data8)
            self._serial.setParity(QSerialPort.NoParity)
            self._serial.setStopBits(QSerialPort.OneStop)
            self._serial.setFlowControl(QSerialPort.NoFlowControl)
            self._serial.readyRead.connect(self.read_serial)
            if self._serial.open(QIODevice.ReadWrite):
                self.ui.textBrowser_statusMessages.append(f"Connected to port {port}")
                self.send_data_serial()
                self.set_all_connect_Button_to_Checkable()
            else:
                self.ui.textBrowser_statusMessages.append(f"Connection Failed")
                self.ui.label_current_warnings.setText("could not connect to the main Gateway")           
        else:
            self.ui.pushButton_Start.setText("Start")
            self._serial.close()

    def read_serial(self):
        serialdata = self._serial.readAll().data().decode()
        sys.stdout.flush()
        self.dataprocessing(serialdata)
        sys.stdout.flush()
        cursor = self.ui.textBrowser_statusMessages.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.update_connect_button_texts()
        cursor.insertText(serialdata)
        self.ui.textBrowser_statusMessages.setTextCursor(cursor)

    def send_data_serial(self):
        # ’’’
        # This function reads the data from the line edit and sends it overinit_nodetxt
        # the serial port to the attached device.
        # ’’’
        message = "start\n"
        self._serial.write(message.encode("utf-8"))
        cursor = self.ui.textBrowser_statusMessages.textCursor()
        # cursor.movePosition(QTextCursor.End)
        #cursor.insertText(message)
        #self.ui.textBrowser_statusMessages.setTextCursor(cursor)

    def check_available_ports(self):
        """
        Function to get all connected comports for selection in combo box
        """
        comlist = serial.tools.list_ports.comports()
        connected = []
        
        for element in comlist:
            self.ui.textBrowser_statusMessages.append("Following devices connected:\n")
            self.ui.textBrowser_statusMessages.append("Device: " + str(element.device))
            self.ui.textBrowser_statusMessages.append(("Manufacturer: " + str(element.manufacturer)))
            self.ui.textBrowser_statusMessages.append("\n")
            connected.append(element.device)
        if len(connected)== 0:
            self.ui.textBrowser_statusMessages.append("No Devices Connected")
        else:
            self.ui.pushButton_Start.setDisabled(False)
        self.connected = connected
        self.ui.comboBox_nodenumbers.addItems(connected)

    def set_all_connect_Button_to_Checkable(self):
        a = len(self.ui.nodes_widgets_generaL_list)
        nodes = self.ui.nodes_widgets_generaL_list
        for i in range(0,a):
            current_nodes = nodes[i]
            current_nodes[2].setCheckable(True)
            current_nodes[3].setCheckable(True)

    def dataprocessing(self, message1):
        if "routingdata" in message1:
            print("routing data",message1,"routing data end",flush=True)
        if "\n" in message1 and not "sensordata" in message1:
            message = message1.split("\n")
            for i in range(len(message)):
                if not (str(message[i]) == ""):
                    if self.contains_relevantdata(message[i]):
                        self.datamatrix(message=message[i])
        else:
            if "routingdata" in message1 and "sensordata" in message1:
                message = message1.split("\n")
                for i in range(len(message)):
                    if not (str(message[i]) == ""):
                        if self.contains_relevantdata(message[i]):
                            self.datamatrix(message=message[i])
            else:
                self.datamatrix(message1)

    def sensor_data_processor(self, sensordata):
        
        #print("sesor data befor",sensordata)
        sensordata = sensordata.replace("sensordata:", "")
        sensordata = sensordata.split("\n")
        sensordata_dict = {}
        sensordata_cat_list = []
        sensordata = list(filter(None, sensordata))
        #print(sensordata)
        #print("after",sensordata)
        for i in range(0, 5):
            try:
                sensordata1 = sensordata[i].split(" = ")
            except:
                print("sensordata package corrupted")
                #print(sensordata1)
            sensordata_dict[sensordata1[0]] = sensordata1[1]
            if not ("src_addr" in sensordata1[0] or "src_id" in sensordata1[0]):
                sensordata_cat_list.append(sensordata1[0])
        current_node_number = int(sensordata_dict["src_id"])
        current_node_widgets = self.get_node_widgets(current_node_number)
        if not(setted_combobox[current_node_number]):
            current_node_widgets[1].addItems(sensordata_cat_list)
            setted_combobox[current_node_number] = True
        current_data = self.plot_data[current_node_number]
        current_curve = self.list_of_curves[current_node_number]
        if current_node_widgets[2].isChecked():
            current_data_stream = current_node_widgets[1].currentText()
            current_data[:-1] = current_data[1:]
            current_data[-1] = sensordata_dict[str(current_data_stream)]
            self.plot_ptr[current_node_number] += 1
            if "temp" in current_data_stream:
                current_data= current_data / 1000
            if "mv" in current_data_stream:
                pass
            current_curve.setData(current_data)
            current_curve.setPos(self.plot_ptr[current_node_number],0)

    def get_node_widgets(self,node):
        return self.ui.nodes_widgets_generaL_list[node]

    def datamatrix(self, message):
        #print("new Message", message)
        if "routingdata:num_total_nodes" in message:
            splitmessage = message.split(" = ")
            print(splitmessage)
            self.numberofnodes = int(splitmessage[1])
            # print("eas",splitmessage[1])
            # self.ui.setup_nodes(2)
        elif "status:" in message:
            if "status:nodedown:" in message:
                #print(message)
                splitedmessage=message.replace("status:nodedown:","")
                #print(splitedmessage)
                self.update_graph(isnodedown=True,nodedown= splitedmessage)
            else:
                splitmessage = message.split(":")
                self.ui.label_Status.setText("Status: " + splitmessage[1])
        elif "routingdata:nodelist " in message:
            #print(message)
            splitmessage = message.split(" = ")
            #print(splitmessage)
            self.src_adrlist = splitmessage[1].split(",")
            #print(self.src_adrlist)
            #self.numberofnodes = len(self.src_adrlist)
        elif "routingdata:prevnodes " in message:
            splitmessage = message.split(" =")
            self.prevnodes = splitmessage[1].split(",")
            self.update_graph(isupdateadjmx=True)
        elif "sensordata" in message:
            self.sensor_data_processor(message)
        elif "routingdata:adjmx" in message:
            splitmessage = message.split(" = ")
            adjmx = splitmessage[1]
            adjmx = adjmx.split(";")
            adjmx = list(filter(None, adjmx))
            matx = np.fromstring(adjmx[0], dtype=int, sep=",")
            for i in range(1, len(adjmx)):
                matx = np.append(matx, np.fromstring(adjmx[i], dtype=int, sep=","))
            self.adj_arr = matx.reshape(len(adjmx), len(adjmx))
            print("adjmx is equal to", self.adj_arr)
            print(self.adj_arr[0][2])
            
        else:
            pass

    def contains_relevantdata(self, message):
        if "routingdata:num_total_nodes" in message:
            return True
        elif "status:" in message:
            return True
        elif "routingdata:nodelist " in message:
            return True
        elif "routingdata:prevnodes " in message:
            return True
        elif "sensordata:src_addr" in message:
            return True
        elif "routingdata:adjmx" in message:
            return True
        else:
            return False

    def setup_node_pos(self, numberofnodes):
        node_definitions = []
        #print(node_definitions)
        for i in range(0, numberofnodes):
            if i == 2:
                node_definitions.append((0 + 10 * i, i))
            else:
                if (i % 3) == 0:
                    node_definitions.append((i * 10, 10 * i))
                else:
                    node_definitions.append((i, 0 + 10 * i))
        return np.array(node_definitions, dtype=float)

    def init_nodesymbols(self):
        list_sy = ['+']
        for i in range(1, self.numberofnodes):
            list_sy.append('o')
        return list_sy

    def init_nodecon(self,isnodedown=False,nodedown=0,isadjupadted=False):
        node_connections = []
        for i in range(0, self.numberofnodes):
            if isnodedown:
                a = i
                if i == nodedown:
                    b = i
                else:
                    b = int(self.prevnodes[i])
                    node_connections.append((a, b))
            else:
                a = i
                b = int(self.prevnodes[i])
                node_connections.append((a, b))
        if isadjupadted:
            for i in range(0,self.numberofnodes):
                for a in range (0,self.numberofnodes):
                    b = a
                    if self.adj_arr[i][a] == 0:
                        b=i
                    else:
                        node_connections.append((i,b))
                    
        return np.array(node_connections)

    def init_nodetxt(self):
        nodetxt = []
        #print(self.src_adrlist)
        #print(self.numberofnodes)
        for i in range(0, self.numberofnodes):
            nodetxt.append("Node " + str(i) + " : \n" + self.src_adrlist[i])
        return nodetxt
    def update_connect_button_texts(self):
        a = len(self.ui.nodes_widgets_generaL_list)
        nodes = self.ui.nodes_widgets_generaL_list
        for i in range(0,a):
            current_node_widget= nodes[i]
            if current_node_widget[2].isChecked():
                current_node_widget[2].setText("Disconnect")
            else:
                current_node_widget[2].setText("Connect")
            if current_node_widget[3].isChecked():
                current_node_widget[3].setText("stop")
            else:
                current_node_widget[3].setText("Start Recording")

    def addNodelines(self):
        print("nodeadj",len(self.nodesadj))
        linelist = []
        for i in range(0,len(self.nodesadj)):
            if i <= self.numberofnodes:
                linelist.append((0, 255, 0, 255, 4))
            else:
                linelist.append((255, 0, 0, 255, 1))
        return np.array(linelist, dtype=[('red', np.ubyte), ('green', np.ubyte), ('blue', np.ubyte), ('alpha', np.ubyte), ('width', float)])


    def update_graph(self,isnodedown=False,nodedown=0,isupdateadjmx=False):
        # self.verticalLayout_2.addWidget(self.pgwidget)
        if isnodedown:
            nodedown= int(nodedown)
            self.nodespos = self.setup_node_pos(self.numberofnodes)
            self.nodesadj = self.init_nodecon(isnodedown, nodedown)
            self.nodesysmbols = self.init_nodesymbols()
            self.nodeTxt = self.init_nodetxt()
            self.addNodelines()
            self.ui.graph.setData(pos=self.nodespos, adj=self.nodesadj, size=1, symbol=self.nodesysmbols, pxMode=False,
                                  text=self.nodeTxt)
        elif isupdateadjmx:
            self.nodespos = self.setup_node_pos(self.numberofnodes)
            self.nodesadj = self.init_nodecon(isadjupadted=isupdateadjmx)
            self.nodesysmbols = self.init_nodesymbols()
            self.nodeTxt = self.init_nodetxt()
            self.linestyles=self.addNodelines()
            self.ui.graph.setData(pos=self.nodespos, adj=self.nodesadj, size=1, symbol=self.nodesysmbols, pxMode=False,
                                  text=self.nodeTxt,pen=self.linestyles)
        else:
            #print(self.numberofnodes)
            self.nodespos = self.setup_node_pos(self.numberofnodes)
            self.nodesadj = self.init_nodecon()
            self.nodesysmbols = self.init_nodesymbols()
            self.nodeTxt = self.init_nodetxt()
            self.linestyles=self.addNodelines()
            self.ui.graph.setData(pos=self.nodespos, adj=self.nodesadj, size=1, symbol=self.nodesysmbols, pxMode=False,
                                  text=self.nodeTxt,pen=self.linestyles)

        # self.horizontalLayout.addItem(self.graph)
        # self.graphicsView = QtWidgets.QGraphicsView(self.horizontalLayoutWidget)
        # self.graphicsView.setObjectName("graphicsView")

        # self.nodespos = self.setup_nodes(6)
        ## Define positions of nodes

        ## Define the set of connections in the graph
        # self.nodesadj = self.init_random_nodecon(6)

        ## Define the symbol to use for each node (this is optional)
        # self.nodessymbols = ['o', 'o', 'o', 'o', 't', '+']

        ## Define the line style for each connection (this is optional)
        # self.nodeslines = np.array([
        #     (255, 0, 0, 255, 1),
        #     (255, 0, 255, 255, 2),
        #     (255, 0, 255, 255, 3),
        #     (255, 255, 0, 255, 2),
        #     (255, 0, 0, 255, 1),
        #     (255, 255, 255, 255, 4),
        # ], dtype=[('red', np.ubyte), ('green', np.ubyte), ('blue', np.ubyte), ('alpha', np.ubyte), ('width', float)])

        ## Define text to show next to each symbol
        # texts = ["Point %d" % i for i in range(6)]
        # self.graph.setData(pos=self.nodespos, adj=self.nodesadj, size=1, symbol=self.nodessymbols,pxMode=False, text=texts)


if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    window = Serial_Main_Backend(apps=app)
    sys.exit(app.exec_())
