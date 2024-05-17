import socket
import struct
import sys
import os
import glob
import numpy as np
import cv2
from PIL import Image
import time
from datetime import datetime

from body_from_image import *
from TCPClient import *
from HLCameraCalibration import *
import ImageRectification as ImRec
import CoordinateLogging as cl

tracking_img_pth = '../../../examples/media/tracking/'
calib_folder = '../../../examples/media/calibration_images/'
left_cam_img_pth =  '../../../examples/media/calibration_images/left_camera/'
right_cam_img_pth =  '../../../examples/media/calibration_images/right_camera/'
paths = [tracking_img_pth, calib_folder, left_cam_img_pth, right_cam_img_pth, cl.log_folder, cl.coord_log_dir]

def tcp_server():
    '''
    Summary:
    tcp_server script is used to connect to the remote TCP Client on the HoloLens
    and receive the image bytes sent from the HoloLens. These images are stored and
    processed by openpose then sent back to the HoloLens TCP Server via the local
    TCP Client.
    '''

    serverHost = '' # localhost
    serverPort = 9090
    
    # check that directories are valid/created
    for path in paths:
         directory_check(path)

    # create a stream socket for receiving data
    sSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # attach the socket to the specified port
        sSock.bind((serverHost, serverPort))
        print('\nServer bind to port ', serverPort)
    except socket.error as msg:
        print('Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1])
        return

    sSock.listen(10)
    print('ServerHost (' + str(serverHost) + ') : ServerPort (' + str(serverPort) + ')')
    print('\n\tListening for client connection requests...\n')
    while True:
        try:
            # connect to the remote TCP Client
            conn, addr = sSock.accept()
            print('\tConnected with ' + addr[0] + ' : ' + str(addr[1]) + '\n')
            client = TCPClient(addr[0], addr[1])
            break
        except KeyboardInterrupt:
            sys.exit(0)
        except Exception:
            continue

    acceptable_chars = ['f', 'v']
    log_date = datetime.now()
    log_file_prefix = log_date.strftime("%d%m%Y-%H%M%S")
    #loop for receiving data during app runtime
    while True:
        start_time = datetime.now()
        try:
                header, length, timestamps, image_data = receive_tcp_message(conn)
        except EOFError as e:
                print(f"TCPServer :: receive_tcp_message() :: ERROR \n{e}")
                continue

        print(f"Header received >> {header}")
        if header in acceptable_chars:
            # header 'f' used for sending images from LR front spatial cameras on HoloLens
            if header == 'f':
                # NOTE: there may be an issue in the order of undistorting images, getting openpose
                # coordinates, and calculating depth that is causing the depth calculation to be
                # sparradic
                image_file_L, ts_L, image_file_R, ts_R = spatial_image_conversion(length, image_data, tracking_img_pth, tracking_img_pth, timestamps)

                imgL_pth = tracking_img_pth + image_file_L
                imgR_pth = tracking_img_pth + image_file_R

                # read the stored images
                imgL = cv2.imread(imgL_pth)
                imgR = cv2.imread(imgR_pth)

                frame_left, frame_right = ImRec.undistortRectify(imgL, imgR)    

                # save the image files to the save folder
                file_name_left = str(ts_L)+'_LF.png'
                file_name_right = str(ts_R)+'_RF.png'
                cv2.imwrite(tracking_img_pth + file_name_left, frame_left)
                cv2.imwrite(tracking_img_pth + file_name_right, frame_right)
        
                OP_Coord_String_L = body_from_image.find_points(tracking_img_pth, file_name_left) # Openpose
                OP_Coord_String_R = body_from_image.find_points(tracking_img_pth, file_name_right) # Openpose

                imgL_pth = tracking_img_pth + file_name_left
                imgR_pth = tracking_img_pth + file_name_right

                OP_Coord_String = HLCameraCalibration.Calculate3DCoordiantes(imgL_pth, imgR_pth, OP_Coord_String_L, OP_Coord_String_R) #, OP_Coord_String_L, OP_Coord_String_R
                cl.log_coordinates(OP_Coord_String, log_file_prefix)

            # header 'v' used for sending images from PV camera on HoloLens
            if header == 'v':
                image_file = pv_image_conversion(image_data, tracking_img_pth)

                OP_Coord_String = body_from_image.find_points(tracking_img_pth, image_file) # Openpose

            client.sendPoints(OP_Coord_String)

        # header 'c' used for sending calibration images
        if header == 'c':
            image_file_L, ts_L, image_file_R, ts_R = spatial_image_conversion(length, image_data, left_cam_img_pth, right_cam_img_pth, timestamps) 

        if header == 'x':
             print("\nPerforming OpenCV Camera Calibration...\n")
             ret = HLCameraCalibration.OpenCV_Chess_Calibration(left_cam_img_pth, right_cam_img_pth)

        # header 'e' used for sending application closed indicator
        if header == 'e':
                #images = glob.glob(tracking_img_pth + '*.png')
                #for img in images:
                     #os.remove(img)
                print("\n------------------")
                print("Application Closed")
                print("------------------")
                return # stop execution of script

        end_time = datetime.now()
        print("TCPServer :: Image Processed :: runtime (", end_time-start_time,")")


def spatial_image_conversion(len, image_byte_buffer, save_loc_L, save_loc_R, ts):
    '''
    Summary:
    Method is used to convert the byte image buffer into images. The spatial
    images are received in a combined/single buffer. The left and right spatial
    images are seperated and stored as independent images.

    Parameters:\n
    len >> The number of bytes in the image buffer\n
    image_byte_buffer >> the buffer containing the image bytes\n
    save_loc_* >> the file path for where to store the images\n
    ts >> the timestamp associated to the image

    Returns:\n
    file_name_* >> return the file names so the images can be processed by openpose
    '''

    ts_left, ts_right = struct.unpack(">qq", ts)

    byte_split = int(len/2)

    # convert image bytes into images
    LF_img_np = np.frombuffer(image_byte_buffer[0:byte_split], np.uint8).reshape((480,640))
    LF_img_np = cv2.rotate(LF_img_np, cv2.ROTATE_90_CLOCKWISE)
    RF_img_np = np.frombuffer(image_byte_buffer[byte_split:len], np.uint8).reshape((480,640))
    RF_img_np = cv2.rotate(RF_img_np, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # save the image files to the save folder
    file_name_left = str(ts_left)+'_LF.png'
    file_name_right = str(ts_right)+'_RF.png'
    cv2.imwrite(save_loc_L + file_name_left, LF_img_np)
    cv2.imwrite(save_loc_R + file_name_right, RF_img_np)

    return file_name_left, ts_left, file_name_right, ts_right


def pv_image_conversion(image_byte_buffer, save_loc):
     '''
     Summary:
     Method is used to convert the PV byte image buffer into an image.

     Parameters:\n
     image_byte_buffer >> the buffer containing the image bytes
     save_loc >> the file path of where to save the images

     Returns:\n
     file_name_* >> return the file name so the image can be processed by openpose
     '''
     # convert image bytes into an image
     PV_img_np = np.frombuffer(image_byte_buffer, dtype=np.uint32)
     PV_image = Image.frombytes('RGBA', (424,240), PV_img_np)
    
     # save the image file to the save folder
     timestamp = str(int(time.time()))
     file_name = timestamp + '_PV.png'
     PV_image.save(save_loc + file_name, encoding_errors='ignore')

     return file_name


def receive_tcp_message(sock):
     '''
     Summary:
     Function is used to receive the image byte data from the remote TCP Client. Messages
     are read from the data stream socket and decoded. The receive function gets:
     - The header byte (ie. message start byte/message type identifier)
     - The data length value
     - The data bytes

     Parameters:\n
     Socket >> the data stream socket from which we can read the incoming data

     Returns:\n
     header >> The header value read from the data stream\n
     data_length >> The number of bytes of image data\n
     timestamp_bytes >> The bytes representing the timestamp values\n
     image_bytes >> The image bytes read from the data stream
     '''
     # read in the first byte (header)
     header_byte = sock.recv(1)
     if not header_byte:
          raise EOFError('Socket closed before header received (header)')
     
     header = header_byte.decode('UTF-8')
     if header == 'e' or header == 'x':
          data_length = 0
          timestamp_bytes = b'0'
          image_bytes = b'0'
          return header, data_length, timestamp_bytes, image_bytes 

     # read in the 4 bytes representing the 32 bit data length integer 
     data_length_bytes = sock.recv(4)
     if not data_length_bytes or len(data_length_bytes) < 4:
          raise EOFError('Socket closed before header received (length)')
     
     data_length = int.from_bytes(data_length_bytes, byteorder='big')

     # if images are from spatial cameras, aslo get the timestamps
     # 2 x 8 bytes representing the 64 bit timestamp values
     timestamp_bytes = b'0'
     if header == 'f' or header == 'c':
          timestamp_bytes = b''
          timestamp_bytes = sock.recv(16)
          

          if not timestamp_bytes or len(timestamp_bytes) < 16:
               raise EOFError('Socket closed before header received (timestamp)')

     # read the image bytes from the data stream
     image_bytes = receive_all(sock, data_length)

     return header, data_length, timestamp_bytes, image_bytes


def receive_all(sock, data_len):
     '''
     Summary:\n
     Function reads in the image bytes from the data stream.

     Parameters:\n
     Socket >> The data stream socket from which we can read the incoming data\n
     Integer >> The number of bytes of image data being sent over TCP

     Return:\n
     image_bytes >> The image bytes 
     '''

     image_bytes = b''
     # read all the image bytes (data_len # of bytes) from the data stream
     while len(image_bytes) < data_len:
          _data_buffer = sock.recv(data_len - len(image_bytes))
          if not _data_buffer:
               raise EOFError('Socket closed before header received (image bytes)')
          
          image_bytes += _data_buffer

     return image_bytes
  

def directory_check(dir):
    if not os.path.isdir(dir):
        os.mkdir(dir)

#=========================================================================================

if __name__ == "__main__":


            tcp_server()
