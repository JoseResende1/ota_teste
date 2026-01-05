from motor import Motor
from rs485 import RS485
from utils import log

class Controller:
    def __init__(self):
        log("Controller init")
        self.motor = Motor()
        self.bus = RS485()

    def start(self):
        log("Controller start")
        self.bus.send("HELLO")
        self.motor.move()
