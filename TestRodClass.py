from RodClass import Rod
import numpy as np
import asyncio

class TestRodClass():
    def __init__(self):
        self.Goalie = Rod(1)
        self.Defense = Rod(2)
        self.Mid = Rod(3)
        self.Forward = Rod(4)
        self.ENDCHAR = '#'

    def test_instantiation(self):
        print("Testing instantiation of RodClass")
        assert self.Goalie.Player == 1
        assert self.Defense.Player == 2
        assert self.Mid.Player == 3
        assert self.Forward.Player == 4

        ##np.array([19.05, 225.425]) ##Boundary of the 1st goalie position in mm
        # self.G2_Boundary = np.array([225.425, 434.975]) ##Boundary of the 2nd goalie position in mm
        # self.G3_Boundary = np.array([431.8, 638.175])
        
        assert np.allclose(self.Goalie.PlayerBoundaries, np.array([[0, 225.425], [225.425, 434.975], [431.8, 660.4]]))
        assert np.allclose(self.Goalie.PlayerPositions, [28.575, 234.95, 441.325])
        assert np.allclose(self.Goalie.RodRange, [0, 2.55 * self.Goalie.microstepping])

        assert np.allclose(self.Defense.PlayerBoundaries, [[0, 396.875], [139.7, 660.4]])
        assert np.allclose(self.Defense.PlayerPositions, [28.575, 269.875])
        assert np.allclose(self.Defense.RodRange, [0, 5 * self.Defense.microstepping])

        assert np.allclose(self.Mid.PlayerBoundaries, [[0, 152.4], [139.7, 273.05], [260.35, 393.7], [381, 514.35], [501.65, 660.4]])
        assert np.allclose(self.Mid.PlayerPositions, [28.575, 149.225, 269.875, 390.525, 511.175])
        assert np.allclose(self.Mid.RodRange, [0, 1.53125 * self.Mid.microstepping])

        assert np.allclose(self.Forward.PlayerBoundaries, [[0, 269.875], [203.2, 454.025], [387.35, 660.4]])
        assert np.allclose(self.Forward.PlayerPositions, [28.575, 212.725, 396.875])
        assert np.allclose(self.Forward.RodRange, [0, 3.1375 * self.Forward.microstepping])
        pass
    def test_block_function(self):
        # Test the goalie functionality of the RodClass
        ballpos = [200,400]
        goaliepos = self.Goalie.blockBall(ballpos)
        Defensepos = self.Defense.blockBall(ballpos)
        Midpos = self.Mid.blockBall(ballpos)
        Forwardpos = self.Forward.blockBall(ballpos)

        send = str(int(goaliepos)) + str(Defensepos) + str(Midpos) + str(Forwardpos) + self.ENDCHAR

        print(send)

    # Add more test methods as needed

if __name__ == "__main__":
    test = TestRodClass()
    test.test_instantiation()
    test.test_block_function()
