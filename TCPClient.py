import os
os.environ['PYTHONASYNCIODEBUG'] = '1'
import asyncio
import struct
import base64

class TCPClient:

    def __init__(self, device_IP, device_Port):
        self.HoloLens_Connection_IP = device_IP
        self.HoloLens_Connection_Port = 8080


    #connects this client to the hololens server                                      
    async def tcp_echo_client(self,data_msg, loop):
        '''
        Summary:\n
        Function is used to establish a connection to the remote TCP Server and send
        the openpose data back to the HoloLens.

        Parameters:\n
        bytes >> The byte message containing all the data\n
        loop >> Event loop used for sending all the data
        '''

        # establish a connection with the HoloLens
        _, writer = await asyncio.open_connection(self.HoloLens_Connection_IP, self.HoloLens_Connection_Port, loop=loop)

        try:
            writer.write(data_msg)
            await writer.drain()
            writer.write_eof()
        except (ConnectionResetError, ConnectionAbortedError) as e:
            print(f'TCPClient : Connection error <{e}>')


    def sendPoints(self,openpose_coordinates):
        '''
        Summary:\n
        Function takes in the openpose coordinate data and encodes it to be sent
        to the HoloLens TCP Server.

        Parameters:\n
        string >> The openpose coordinate data
        '''

        loop = asyncio.new_event_loop()

        # encoding data bytes
        data = bytes(openpose_coordinates, "utf-8")
        encoded_data = base64.b64encode(data)

        # the length of the encoded bytes
        length = struct.pack('!I', len(encoded_data))
        encoded_length = base64.b64encode(length)

        # the message beginning/validity identifier
        msg_id = struct.pack('!B', 0x01)
        encoded_msg_id = base64.b64encode(msg_id)

        # combine into 1 message
        data_msg = encoded_msg_id + encoded_length + encoded_data  

        loop.run_until_complete(self.tcp_echo_client(data_msg, loop))

        loop.close()

