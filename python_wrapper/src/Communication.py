import telnetlib
from time import sleep

class Communication:
    '''Class for establishing and managing Telnet connections
    Parameters:
        host (str): The hostname or IP address of the remote device.
        port (int): The port number to use for the Telnet connection.

    Attributes:
        host (str): The hostname or IP address of the remote device.
        port (int): The port number to use for the Telnet connection.
        socket (socket): The socket object used for the Telnet connection.
        socketList (list): A list of socket objects representing multiple Telnet connections.
        tnList (list): A list of Telnet objects representing multiple Telnet connections.
        timeout (float): The time to wait between write and read operations on the Telnet connection(s).

    Methods:
        connectSocket(): Establish a single Telnet connection with the specified host and port.
        connectMultipleSocket(ip, index): Establish a Telnet connection with the specified IP address and port, and add it to the list of Telnet connections at the specified index.k
        writeSocket(data): Write data to the single Telnet connection.
        writeMultipleSocket(data, index): Write data to the Telnet connection at the specified index in the list of connections.
        readSocket(): Read data from the single Telnet connection.
        readMultipleSocket(index): Read data from the Telnet connection at the specified index in the list of connections.
    '''

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.socket = None
        self.socketList = []
        self.tnList = []
        self.timeout = 0.005


    def connectSocket(self):
        self.tn = telnetlib.Telnet(self.host, self.port)
        self.socket = self.tn.get_socket()

    def connectMultipleSocket(self, ip, index):
        self.tnList[index] = telnetlib.Telnet(ip, self.port)
        self.socketList[index] = self.tnList[index].get_socket()

    def writeSocket(self, data):
        self.socket.send(data)
        sleep(self.timeout)

    def writeMultipleSocket(self, data, index):
        self.socketList[index].send(data)
        # sleep(self.timeout)

    def readSocket(self):
        return self.socket.recv(1024)

    def readMultipleSocket(self, index):
        return self.socketList[index].recv(1024)