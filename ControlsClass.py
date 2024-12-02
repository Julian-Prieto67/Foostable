from ArduinoClass import Arduino
from RodClass import RodReal, RodFake
from ArduinoClass import Arduino, ArduinoFake
import np

class Controller():
    def __init__(self, playback = False):
        self.playback = playback

        self.ball_pos_real = np.array([0, 0])
        ##below is the current Ball state in the format [x, y, dx, dy] as a column vector
        self.ball_state = [[self.ball_pos_real(0)][self.ball_pos_real(1)][0][0]]

        ##Dependency Injection for rods and arduino
        if not self.playback:
            self.GRod = RodReal(1) ##initializes goalie rod
            self.DRod = RodReal(2) ##initializes defense rod
            self.MRod = RodReal(3) ##initializes midfield rod
            self.ARod = RodReal(4) ##initializes attack rod
            self.ser = Arduino() ##initializes arduino
        else:
            self.GRod = RodFake(1) ##initializes goalie rod
            self.DRod = RodFake(2) ##initializes defense rod
            self.MRod = RodFake(3) ##initializes midfield rod
            self.ARod = RodFake(4) ##initializes attack rod
            self.ser = ArduinoFake() ##initializes arduino
    def run(self):
        ##Execute the control loop
            pass