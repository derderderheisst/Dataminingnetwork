import serial
class Serial_Connection():
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=2):
#     ’’’
# Constructor.
# Args:
# port (string): The name of the port to connect to
# baudrate (integer): The baudrate used in the connection
# timeout (integer): Timeout in seconds. Define the time to wait
# for incoming data.
#     ’’’
        self.__port = port
        self.__baudrate = baudrate
        self.__timeout = timeout
        # create serial object and open port
        self.__ser = serial.Serial(port=self.__port, baudrate=self.
                                   __baudrate, timeout=self.__timeout)
    def __del__(self):
        """
        Destructor -> shall close serial port if necessaray
        """
        if(self.__ser.is_open):
            self.__ser.close()

    def write_data(self, data=""):

        if not isinstance(data, str):
            try:
                data = str(data)
            except:
                raise ValueError("Please enter a string or something that can beconvertedto astring.")
        # check if string ends with new line -> if not: add new line
        # this is required, because otherwise the zolertia-re mote doesn’t
        # trigger
        # serial
        # event
        if not data.endswith("\n"):
            data = data + "\n"
        self.__ser.write(data.encode('utf - 8'))

