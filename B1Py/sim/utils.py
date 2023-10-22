class simulationManager():
    def __init__(self, 
                 robot, 
                 lcm_server, 
                 default_cmd):
        
        self.robot = robot
        self.lcm_server = lcm_server
        self.missed_ticks = 0
        self.default_cmd = default_cmd
        self.robot.initialize()
        self.reset_required = True

    def step(self):
        # Read the robot's state and send it to the LCM client
        state = self.robot.readStates()
        self.lcm_server.sendStates(state)
        # Wait for a response from the LCM client timeout=0.1 second
        lcm_cmd = self.lcm_server.getCommands(timeout=0.1)
        if lcm_cmd is not None:
            self.missed_ticks=0
            # Reset the robot if the communication has been off for too long
            if self.reset_required:
                self.robot.initialize()
                self.reset_required=False
                return
            self.robot.setCommands(lcm_cmd)
        else:
            self.robot.setCommands(self.default_cmd)
            self.missed_ticks +=1
        if self.missed_ticks > 10:
            self.reset_required = True