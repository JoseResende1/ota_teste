from utils import log

class RS485:
    def send(self, msg):
        log("RS485 TX: %s" % msg)
