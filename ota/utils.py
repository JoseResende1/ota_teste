import time

def log(msg):
    print("[%d] %s" % (time.ticks_ms(), msg))
