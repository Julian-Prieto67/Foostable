import numpy as np
import time
import asyncio
import math
import moteus

class Rod:
    def __init__(self, Player):
        self.Player = Player #1-4 for goalie, defense, midfield, attack
        self.name = ""
        self.rodPos = 0 ##Current position of rod  (mm)
        self.rodAngle = 0 ##current angle of rod
        self.PlayerBoundaries = [] ##List of lateral reach boundaries of player positions
        self.PlayerPositions = [] ##List of player positions in mm (center of player)
        self.defaultPos = 0 ##Default position of player
        self.ReachBoundary = [] ##List of reach boundaries of player positions (where they can hit the ball)
        self.x_level = 0 ##Current x level of rod (mm)
        self.microstepping = 800 ##number of steps per revolution
        self.mm2step = self.microstepping/(23*math.pi) ##Conversion factor from mm to steps = pulley diameter *pi / steps per revolution

        self.RodRange = np.array([0, 0]) ##Range of rod in mm
        self.playerOffsets = [] ##List of player offsets from the center of the rod 
        ##PLAYER BOUNDARIES
        ##BOUNDARIES ARE DEFINED AS [LOWERBOUND, UPPERBOUND]
        ##EDGE OF ROD IS 19.05mm FROM EDGE OF TABLE and 638.175mm FROM EDGE OF TABLE
        ## THIS IS IGNORED TO ALLOW FOR FULL COVERAGE OF THE TABLE

        self.G1_Boundary = np.array([0, 225.425]) ##Boundary of the 1st goalie position in mm
        self.G2_Boundary = np.array([225.425, 434.975]) ##Boundary of the 2nd goalie position in mm
        self.G3_Boundary = np.array([431.8, 660.4]) ##Boundary of the 3rd goalie position in mm
        self.G1_Offset = 1.125*25.4
        self.G2_Offset = 9.25*25.4
        self.G3_Offset = 17.375*25.4
        self.G_Offsets = [self.G1_Offset, self.G2_Offset, self.G3_Offset]
        self.G_defaultpos = 1550 / self.mm2step
        self.G_xlevel = 3.75*25.4
        self.G_range = [0, 2.55*23 * np.pi]
        ##Alternate starting positions
        #self.defaultpos = 550
        

        self.D1_Boundary = np.array([0, 396.875]) ##Boundary of the 1st defense position in mm
        self.D2_Boundary = np.array([139.7, 660.4]) ##Boundary of the 2nd defense position in mm
        self.D1_Offset = 1.125*25.4
        self.D2_Offset = 10.625*25.4
        self.D_Offsets = [self.D1_Offset, self.D2_Offset]
        self.D_defaultpos = 3000 / self.mm2step
        self.D_xlevel = 9.625*25.4
        self.D_range = [0, 5*23*np.pi]
        ##Alternate starting positions
        #self.defaultpos = 900
        
        self.M1_Boundary = np.array([0, 152.4]) ##Boundary of the 1st mid position in mm
        self.M2_Boundary = np.array([139.7, 273.05]) ##Boundary of the 2nd mid position in mm
        self.M3_Boundary = np.array([260.35, 393.7]) ##Boundary of the 3rd mid position in mm
        self.M4_Boundary = np.array([381, 514.35]) ##Boundary of the 4th mid position in mm
        self.M5_Boundary = np.array([501.65, 660.4]) ##Boundary of the 5th mid position in mm
        self.M1_Offset = 1.125*25.4
        self.M2_Offset = 5.875*25.4
        self.M3_Offset = 10.625*25.4
        self.M4_Offset = 15.375*25.4
        self.M5_Offset = 20.125*25.4
        self.M_Offsets = [self.M1_Offset, self.M2_Offset, self.M3_Offset, self.M4_Offset, self.M5_Offset]
        self.M_defaultpos = 800 / self.mm2step
        self.M_xlevel = 21*25.4
        self.M_range = [0, 1.53125*23*np.pi]

        self.F1_Boundary = np.array([0, 269.875]) ##Boundary of the 1st forward position in mm
        self.F2_Boundary = np.array([203.2, 454.025]) ##Boundary of the 2nd forward position in mm
        self.F3_Boundary = np.array([387.35, 660.4]) ##Boundary of the 3rd forward position in mm
        self.F1_Offset = 1.125*25.4
        self.F2_Offset = 8.375*25.4
        self.F3_Offset = 15.625*25.4
        self.F_offset = [self.F1_Offset, self.F2_Offset, self.F3_Offset]
        self.F_defaultpos = 200/ self.mm2step
        self.F_xlevel = 32.125*25.4
        self.F_range = [0, 3.1375*23*np.pi]


        ##MOTEUS CONTROLLER
        self.motor_state = None
        self.kickTimer = time.time()
        self.kick = False
        self.max_torque = 1.2
        self.vel_lim = 55

        self.Kickball_timer = time.time()
        self.angle_up = math.radians(-90) #puts the rod up (rad)
        self.rest = 0 #puts the rod down (rad)
        self.kickAngle = 1 #(rad) to kick ball from wherever
        # await self.mot.set_stop()
        self.blockingPlayer = None
        self.defineBoundaries()

        
    def defineBoundaries(self):
        # Define the boundaries of where each player can move



        if (self.Player ==1):
            self.name = "Goalie"
            self.PlayerBoundaries.append(self.G1_Boundary) ##Boundary of the 1st goalie position in mm
            self.PlayerBoundaries.append(self.G2_Boundary) ##Boundary of the 2nd goalie position in mm
            self.PlayerBoundaries.append(self.G3_Boundary) ##Boundary of the 3rd goalie position in mm
            self.PlayerPositions.append(self.G1_Offset)
            self.PlayerPositions.append(self.G2_Offset)
            self.PlayerPositions.append(self.G3_Offset)
            self.RodRange = self.G_range
            self.defaultPos = self.G_defaultpos
            self.x_level = self.G_xlevel
            self.playerOffsets = self.G_Offsets
        elif (self.Player ==2):
            self.name = "Defense"
            self.PlayerBoundaries.append(self.D1_Boundary) ##Boundary of the 1st defense position in mm
            self.PlayerBoundaries.append(self.D2_Boundary) ##Boundary of the 2nd defense position in mm
            self.PlayerPositions.append(self.D1_Offset)
            self.PlayerPositions.append(self.D2_Offset)
            self.RodRange = self.D_range
            self.defaultPos = self.D_defaultpos
            self.x_level = self.D_xlevel
            self.playerOffsets = self.D_Offsets
        elif (self.Player ==3):
            self.name = "Midfield"
            self.PlayerBoundaries.append(self.M1_Boundary) ##Boundary of the 1st mid position in mm
            self.PlayerBoundaries.append(self.M2_Boundary) ##Boundary of the 2nd mid position in mm
            self.PlayerBoundaries.append(self.M3_Boundary) ##Boundary of the 3rd mid position in mm
            self.PlayerBoundaries.append(self.M4_Boundary) ##Boundary of the 4th mid position in mm
            self.PlayerBoundaries.append(self.M5_Boundary) ##Boundary of the 5th mid position in mm
            self.PlayerPositions.append(self.M1_Offset)
            self.PlayerPositions.append(self.M2_Offset)
            self.PlayerPositions.append(self.M3_Offset)
            self.PlayerPositions.append(self.M4_Offset)
            self.PlayerPositions.append(self.M5_Offset)
            self.defaultPos = self.M_defaultpos
            self.RodRange = self.M_range
            self.x_level = self.M_xlevel
            self.playerOffsets = self.M_Offsets
        elif (self.Player ==4):
            self.name = "Attack"
            self.PlayerBoundaries.append(self.F1_Boundary) ##Boundary of the 1st forward position in mm
            self.PlayerBoundaries.append(self.F2_Boundary) ##Boundary of the 2nd forward position in mm
            self.PlayerBoundaries.append(self.F3_Boundary) ##Boundary of the 3rd forward position in mm
            self.PlayerPositions.append(self.F1_Offset)
            self.PlayerPositions.append(self.F2_Offset)
            self.PlayerPositions.append(self.F3_Offset)
            self.defaultPos = self.F_defaultpos
            self.RodRange = self.F_range
            self.x_level = self.F_xlevel
            self.playerOffsets = self.F_offset
        else:
            print("Invalid Player Number")
    def returnBlockingPlayer(self):
        return self.blockingPlayer
    def blockBall(self, ballPos):
        ## Block the ball at the given position using the nearest player to the ball
        ## ballPos is the position of the ball in mm

        if any(pos is None for pos in ballPos) or any(pos == 0 for pos in ballPos) or ballPos[0] < self.x_level-50:
            # print("Player " + str(self.Player) + " is in default position")   
            self.setRodPos(self.defaultPos)
            return
        
        # Filter players that can reach the ball
        reachable_players = [player for player, boundary in enumerate(self.PlayerBoundaries) if ballPos[1] >= boundary[0] and ballPos[1] <= boundary[1]]
        if reachable_players == []:
            self.setRodPos(self.defaultPos)
            self.blockingPlayer = None
            return
        # Find the nearest player to the ball among the reachable players
        distances = [abs(ballPos[1] - self.PlayerPositions[player]) for player in reachable_players]
        nearest_player = reachable_players[np.argmin(distances)]
        self.blockingPlayer = nearest_player
        # Adjust the rod position based on the nearest player's position
        self.setRodPos((ballPos[1] - self.PlayerPositions[nearest_player]) + self.rodPos)
        return
    
    async def clearFaults(self):
        await self.mot.set_stop()
    
    def goHome(self):
        ##sets the default angle
        if self.Player == 4:
            self.setRodAngle(self.angle_up)
        else:
            self.setRodAngle(self.rest)
        ##sets the positions
        self.setRodPos(self.defaultPos)

    def setRodPos(self, p):
        ## Set the rod position in mm 
        ## This clips the position to the range of the rods
        ## This position is the center of the rod and updates all the player positions on the rod

        self.rodPos = p
        self.rodPos = np.clip(self.rodPos, self.RodRange[0], self.RodRange[1])
        self.PlayerPositions = [self.rodPos+ pos for pos in self.playerOffsets]

    def returnRodPos(self):
        ##Return the rod position in steps
        PosString = str(int(self.rodPos * self.mm2step)) + '|'
        return PosString
    
    def returnRodPosmm(self):
        ##Return the rod position in steps
        PosString = self.rodPos
        return PosString
    
    def setRodAngle(self, angle):
        ##Set the rod angle in radians
        ##returns the current angle in radians as a float
        self.rodAngle = angle
        self.kick = True

    def checkBall(self, ballPos):
        ##Check if the ball is in the range of the rod
        ##Returns true if the ball is in range of the rod
        if any(pos is None for pos in ballPos) or any(pos == 0 for pos in ballPos):
            return False
        ##checks within an inch of the x-level for each player
        if ballPos[0] > self.x_level - 25.4 and ballPos[0] < self.x_level + 25.4:
            return True
        return False

    def kickAtWill(self, ballPos):
        ##Kick the ball whenever the rod is in position
        self.setRodAngle(self.rest)
        if any(pos is None for pos in ballPos) or any(pos == 0 for pos in ballPos):
            return
        if ballPos[0] > self.x_level - 25.4 and ballPos[0] < self.x_level + 25.4:
            self.setRodAngle(self.kickAngle)
            self.kick = True

    async def RotateRod(self):
        pass

    def returnPlayerPos(self):
        playerPos = []
        for player in self.PlayerPositions:
            playerPos.append((self.x_level, player))
        return playerPos

class RodReal(Rod):
    def __init__(self, Player):
        super().__init__(Player)
        self.mot = moteus.Controller(self.Player)
    
    async def RotateRod(self):
        ##called as much as possible. Will rotate the rod to the currrent rodAngle

        if self.kick and (time.time() - self.kickTimer > 1.3):
            await self.mot.set_position(
            position=self.rodAngle,
            velocity = 55,
            maximum_torque = self.max_torque,
            accel_limit=300.0,
            velocity_limit= self.vel_lim,
            )
            self.kickTimer = time.time()
            self.kick = False
        else:
            # print("enterif ")
            await self.mot.set_position(
            position=self.rodAngle,
            maximum_torque = self.max_torque,
            accel_limit=300.0,
            velocity_limit= self.vel_lim,
            )
        # await asyncio.sleep(0.02)


class RodFake(Rod):
    async def RotateRod(self, kick = False):
        pass
    async def clearFaults(self):
        pass