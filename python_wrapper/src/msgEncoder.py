import struct

class MsgEncoder:
    '''Class for encoding messages to be sent over a Telnet connection.
    Parameters:
        msg (list): A list of bytes representing the message to be encoded.
        typeOfMsg (int): An integer representing the type of the message.
        
    Attributes:
        HEADER (bytes): A list of bytes representing the header of the encoded message.
        DIRECTION (dict): A dictionary mapping message direction strings to corresponding bytes.
        MSP_MSG_PARSE (str): A format string for use with the `struct` module to pack the message for encoding.
        
    Methods:
        encoder(msg, typeOfMsg): Encode a message with the specified type and return the encoded message as bytes.
    '''

    def __init__(self):
        self.HEADER = [b'$', b'M']
        self.DIRECTION = {"IN": b'<', "OUT": b'>'}  # IN: to Drone OUT: From Drone
        self.MSP_MSG_PARSE = '<3c2B%iHB'

    def encoder(self, msg, typeOfMsg):
        lenOfMsg = len(msg)
        msg = self.HEADER + [self.DIRECTION["IN"]] + [lenOfMsg * 2] + [typeOfMsg] + msg
        msg = struct.pack(self.MSP_MSG_PARSE[:-1] % lenOfMsg, *msg)

        checksum = 0
        for i in msg[3:]:
            checksum ^= i
        msg += bytes([checksum])
        return msg