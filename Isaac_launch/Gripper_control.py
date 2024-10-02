from rtde_io import RTDEIOInterface  # Import the RTDEIOInterface class
import logging
import sys
import rtde as rtde
import rtde as rtde_config
import argparse
import time

class UR3eGripper:
    def __init__(self, ip_address, rtde_receive_interface, port):
        # Initialize the RTDEIOInterface with the specified IP address
        self.rtde_io = RTDEIOInterface(ip_address)
        self.rtde_r = rtde_receive_interface

        # Set the initial state of tool digital outs
        self.rtde_io.setToolDigitalOut(1, False)
        self.rtde_io.setToolDigitalOut(0, False)

        # Parse command-line arguments
        parser = argparse.ArgumentParser()
        parser.add_argument("--frequency", type=int, default=125, help="the sampling frequency in Herz")
        parser.add_argument("--config", default="record_configuration.xml", help="data configuration file to use (record_configuration.xml)")
        parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
        parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
        parser.add_argument("--binary", help="save the data in binary format", action="store_true")
        self.args = parser.parse_args()

        # Set IP address and port for RTDE communication
        self.ip_address = ip_address

        # RTDE connection setup
        if self.args.verbose:
            logging.basicConfig(level=logging.INFO)

        self.conf = rtde_config.ConfigFile(self.args.config)
        self.output_names, _ = self.conf.get_recipe("out")
        self.con = rtde.RTDE(ip_address, port)

        self.connect_and_start()

    def connect_and_start(self):
        # Establish the RTDE connection and start synchronization
        self.con.connect()
        self.con.get_controller_version()

        # Set up the output configuration
        if not self.con.send_output_setup(self.output_names, frequency=self.args.frequency):
            logging.error("Unable to configure output")
            sys.exit()

        # Start the data synchronization
        if not self.con.send_start():
            logging.error("Unable to start synchronization")
            sys.exit()

    def open(self):
        # Inform that the gripper is opening
        print("Gripper is opening")

        # Deactivate the tool digital out 1 and activate tool digital out 0
        self.rtde_io.setToolDigitalOut(1, False)
        self.rtde_io.setToolDigitalOut(0, True)

    def close(self):
        # Inform that the gripper is closing
        print("Gripper is closing")

        # Deactivate the tool digital out 0 and activate tool digital out 1
        self.rtde_io.setToolDigitalOut(0, False)
        self.rtde_io.setToolDigitalOut(1, True)

    def extract_position(self):
        try:
            state = self.con.receive(binary=self.args.binary)
            actual_q = None
            gripper_position = None

            if state is not None:
                for name in self.output_names:
                    value = getattr(state, name, None)
                    
                    if name == "tool_analog_input0":
                        if not isinstance(value, list):
                            tool_analog_input0 = value
                            gripper_position = round(map_value(tool_analog_input0, 0, 10, 0.0, -0.013), 3)

                    elif name == "actual_q":
                        actual_q = value

            return actual_q, gripper_position
        except Exception as e:
            logging.error(f"Error receiving data: {e}")
            self.reconnect()
            return None, None

    def reconnect(self):
        logging.info("Attempting to reconnect...")
        try:
            self.con.disconnect()
        except Exception as e:
            logging.error(f"Error during disconnect: {e}")
        
        time.sleep(2)  # Wait before trying to reconnect
        self.connect_and_start()

def map_value(value, min_input, max_input, min_out, max_out):
    # Ensure the value is within the input range
    value = max(min(value, max_input), min_input)
    
    # Map the value from the input range to the output range
    mapped_value = (value - min_input) / (max_input - min_input) * (max_out - min_out) + min_out
    
    return mapped_value

def map_values_in_list(input_list, min_input, max_input, min_out, max_out):
    return [map_value(value, min_input, max_input, min_out, max_out) for value in input_list]
