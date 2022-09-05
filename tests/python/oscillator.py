import unittest
from random import random

from dynamic_graph.sot.tools import Oscillator


class OscillatorTest(unittest.TestCase):
    def test_load(self):
        epsilon = random()
        osc = Oscillator("my oscillator")
        osc.setEpsilon(epsilon)
        self.assertEqual(osc.getEpsilon(), epsilon)


if __name__ == "__main__":
    unittest.main()
