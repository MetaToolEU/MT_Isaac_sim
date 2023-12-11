from rtde_io import RTDEIOInterface  # Import the RTDEIOInterface class

class UR3eGripper:
    def __init__(self, ip_address):
        # Initialize the RTDEIOInterface with the specified IP address
        self.rtde_io = RTDEIOInterface(ip_address)

        # Set the initial state of tool digital outs
        self.rtde_io.setToolDigitalOut(1, False)
        self.rtde_io.setToolDigitalOut(0, False)

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
