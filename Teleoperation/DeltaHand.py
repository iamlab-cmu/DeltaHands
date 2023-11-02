import time
import serial
import math
import numpy as np

class DeltaHandEnv:
    def __init__(self, port, baud):

        self.ser = serial.Serial(port, baud)  # open serial port

        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.minimum_joint_position = 0.0005
        self.maximum_joint_position = 0.0195
        self.done_moving = False
        self.num_motors = 12

        time.sleep(1.0)
        self.ser.reset_input_buffer() #clear incoming usb data
        self.ser.reset_output_buffer() #clear outgoing usb data

    def reset(self):
        self.ser.write(b'<r>')

    def stop(self):
        self.ser.write(b'<s,1>')

    def start(self):
        self.ser.write(b'<s,0>')

    def move_joint_position(self, desired_joint_positions, durations):
        # assert desired_joint_positions.shape[0] == 3
        desired_joint_positions = np.clip(desired_joint_positions,self.minimum_joint_position,self.maximum_joint_position)

        combined_array = np.hstack((np.array(durations).reshape(-1,1),np.array(desired_joint_positions)))
        num_trajectory_points = combined_array.shape[0]
        compressed_array = np.float32(combined_array.flatten())
        msg = '<p,' + str(num_trajectory_points) + ','
        for number in compressed_array:
            msg += str(round(number, 4)) + ','
        msg = msg[:-1] + '>'
        num_messages = int(np.ceil(len(msg) / 60.0))

        for i in range(num_messages):
            print("outgoing message: " + msg)
        self.ser.write(msg.encode())
        # time.sleep(0.1)

    def move_joint_speed_position(self, desired_joint_positions, speeds):

        desired_joint_positions = np.clip(desired_joint_positions,self.minimum_joint_position,self.maximum_joint_position)

        combined_array = np.hstack((np.array(speeds),np.array(desired_joint_positions)))
        num_trajectory_points = combined_array.shape[0]
        compressed_array = np.float32(combined_array.flatten())
        msg = '<m,' + str(num_trajectory_points) + ','
        for number in compressed_array:
            msg += str(round(number, 4)) + ','
        msg = msg[:-1] + '>'
        num_messages = int(np.ceil(len(msg) / 60.0))
        print("MSG: "+msg)
        for i in range(num_messages):
            self.ser.write(msg[i*60:(i+1)*60].encode())
            time.sleep(0.1)

    def move_joint_velocity(self, desired_joint_velocities, durations):

        combined_array = np.hstack((np.array(durations).reshape(-1,1),np.array(desired_joint_velocities)))
        num_trajectory_points = combined_array.shape[0]
        compressed_array = np.float32(combined_array.flatten())

        msg = '<v,' + str(num_trajectory_points) + ','
        for number in compressed_array:
            msg += str(round(number, 9)) + ','
        msg = msg[:-1] + '>'
        num_messages = int(np.ceil(len(msg) / 60.0))

        for i in range(num_messages):
            self.ser.write(msg[i*60:(i+1)*60].encode())
            time.sleep(0.1)

    def close(self):
        self.ser.close()

    def update_joint_positions_and_velocities(self):

        line = self.ser.readline() # read and throw away first incomplete line
        while True:
            line = self.ser.readline() #CHANGE: test complete lines
            try:
                stringline = bytes.decode(line)
                if stringline[0] == 'j':
                    numbers = np.array(stringline[2:].strip().split(','))
                    for i in range(self.num_motors):
                        self.current_joint_positions[i] = float(numbers[i*2])
                        self.current_joint_velocities[i] = float(numbers[i*2+1])
                    return False
                elif stringline[0] == 'd':
                    self.done_moving = True
                    return True
            except:
                print("Errors in reading joint states!")
                pass # Readline did not return a valid string.


    def get_joint_states(self):
        self.update_joint_positions_and_velocities()
        return self.current_joint_positions, self.current_joint_velocities


    def wait_until_done_moving(self, timeout = 6):
        self.done_moving = False
        start_time = time.time()
        elapsed_time = 0.0
        #pid loop is not well-tuned, timeout of 6 seconds may not be enough
        self.ser.reset_input_buffer()
        while not self.done_moving:# and elapsed_time < timeout:
            self.done_moving = self.update_joint_positions_and_velocities()
            elapsed_time = time.time() - start_time
