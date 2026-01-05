from controller import Controller
from config import DEVICE_NAME
from utils import log


class App:
    def __init__(self):
        log("App init em %s" % DEVICE_NAME)
        self.ctrl = Controller()

    def run(self):
        log("App run")
        self.ctrl.start()
