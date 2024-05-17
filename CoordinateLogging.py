import os
import glob

'''
Summary:
CoordinateLogging.py file is used for logging the joint coordinates 
for later processing. The coordinates are stored to a log file during 
runtime of the body tracking application.

This file can also be run to perform a delta calculation of joints between
two coordinate frames. Script will automatically detect new coordinate files
and will run the calculation on them.
'''

log_folder = '/logs/'
coord_log_dir = 'logs/coordinate_logs/'
delta_log_dir = 'logs/delta_logs/'

def log_coordinates(coordinate_string, date, filename='_coordinates.log'):
    '''
    Summary:
    Function writes the joint coordinate strings to a log file.

    Parameters:\n
    coordinate_string >> the calculated 3D coordinate string
    date >> the unique identifier for the log file name






    +
    '''

    log_f = date + filename
    path = os.path.join(coord_log_dir, log_f)

    try:
        with open(path, 'a') as f:
            f.write(coordinate_string + '\n')
    except Exception as e:
        print("error with file >> ", e)


def calculate_deltas(output_log_file, input_coord_file):
    '''
    Summary:
    Function is used to read a coordinate log file and calculates the
    deltas between joints for two coordinate frames.

    Parameters:\n
    output_log_file >> the file reference for writing to the file
    input_coord_file >> the coordinate log file name (excluding the path)
    '''

    print(f'\tReading coordinates from [{coord_log_dir+input_coord_file}]\n\tOutput deltas to [{output_log_file.name}]')

    path = os.path.join(coord_log_dir, input_coord_file)
    with open(path, 'r') as f:
        lines = f.readlines()

    print('\t\tCalculating deltas...')
    # go through all the coordinate frames
    coord_frame_array = []
    for line in lines:
        coordinates = line[3:-4].strip().split('][')
        coord_frame_array.append(coordinates)

        i = len(coord_frame_array)
        if i < 2:
            # there must be at least two frames for a delta
            continue

        # calculate delta between each joint
        output_log_file.write(f'Frames ({i-1} - {i})\n')
        for j in range(len(coord_frame_array[i-2])):
            x1, y1, z1 = map(float, coord_frame_array[i-2][j].split(' '))
            x2, y2, z2 = map(float, coord_frame_array[i-1][j].split(' '))


            delta_x = x1 - x2
            delta_y = y1 - y2
            delta_z = z1 - z2

            output_log_file.write(f'    joint {j:2} deltas >> x:{delta_x:^8.3f} | y:{delta_y:^8.3f} | z:{delta_z:^8.3f}\n')

        output_log_file.write('\n')

    print('\t\tDelta calculation complete.\n')


def directory_check(dir):
    if not os.path.isdir(dir):
        os.mkdir(dir)


#----------------
#  Main Script
#----------------

if __name__ == "__main__":
    '''Run script for delta calculation'''

    print('\nStarting coordinate delta calculation...\n')

    directory_check(delta_log_dir)
    
    coord_logs = glob.glob(coord_log_dir + '*.log')
    delta_logs = glob.glob(delta_log_dir + '*.log')

    if len(delta_logs) == 0:
        delta_logs = ['']

    # what files in coordinate_logs have not been processed
    print('Scanning for unprocessed files...\n')
    calc_files = []
    for c_log in coord_logs:
        file_c = c_log.split('\\')[1]
        file_c_id = file_c.split('_')[0]

        flag = True
        for d_log in delta_logs:
            if file_c_id in d_log:
                flag = False
        
        if flag:
            calc_files.append(file_c)

    # go through all unprocessed files and calculate deltas
    print(f'Found ({len(calc_files)}) unprocessed files\n')
    count = 1
    for file in calc_files:
        print(f'Processing file {count} / {len(calc_files)} >> [{file}]')
        id = file.split('_')[0]
        date, time = map(str, id.split('-'))

        with open(delta_log_dir+id+'_deltas.log', 'w') as f:
            f.write('-- Joint Frame Deltas --\n')
            f.write(f'date (DMY): {date}    time (HMS): {time}\n\n')

            calculate_deltas(f, file)
        count += 1

    print('- Finished -')