import os
os.environ['PYTHONASYNCIODEBUG'] = '1'
import asyncio
import struct
import base64

class TCPClient:

    def __init__(self, device_IP, device_Port):
        self.HoloLens_Connection_IP = device_IP
        self.HoloLens_Connection_Port = 8080
        self.writer = None


    async def connect_to_server(self):
        try:
            _, self.writer = await asyncio.open_connection(self.HoloLens_Connection_IP, self.HoloLens_Connection_Port)
        except ConnectionRefusedError as e:
            print(f'TCPClient : Connection error <{e}>')
            return False
        print('TCPClient : Connected to the server : ', self.writer.get_extra_info('peername'))
        return True


    #connects this client to the hololens server                                      
    async def tcp_echo_client(self,data_msg):
        '''
        Summary:\n
        Function is used to establish a connection to the remote TCP Server and send
        the openpose data back to the HoloLens.

        Parameters:\n
        bytes >> The byte message containing all the data\n
        loop >> Event loop used for sending all the data
        '''

        try:
            self.writer.write(data_msg)
            await self.writer.drain()
        except (ConnectionResetError, ConnectionAbortedError) as e:
            print(f'TCPClient : Connection error <{e}>')


    async def sendData(self,msg_ID,byte_data):
        '''
        Summary:\n
        Function takes in the openpose coordinate data and encodes it to be sent
        to the HoloLens TCP Server.

        Parameters:\n
        string >> The openpose coordinate data
        '''

        if self.writer is None or self.writer.is_closing():
            print('\tTCPClient :: Reconnecting to the server...')
            if not await self.connect_to_server():
                return # Exit if connection fails
            
        print('TCPClient :: Sending data to the server >> ', msg_ID)
        data_msg = TCPClient.encode_data(msg_ID, byte_data)
        await self.tcp_echo_client(data_msg)

    @staticmethod
    def encode_data(msg_ID, data_transmission):
        """
        Encodes the given openpose coordinates into a formatted message.

        Args:
        - openpose_coordinates (str): The openpose coordinates to encode.

        Returns:
        - bytes: The encoded message.
        """
        data = bytes(data_transmission, "utf-8")
        encoded_data = base64.b64encode(data)

        msg_id = struct.pack('!B', msg_ID)
        encoded_msg_id = base64.b64encode(msg_id)

        if msg_ID == 0x01:
            length = struct.pack('!I', len(encoded_data))
            encoded_length = base64.b64encode(length)

            return encoded_msg_id + encoded_length + encoded_data
        
        return encoded_msg_id + encoded_data

