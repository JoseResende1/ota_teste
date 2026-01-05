# ======================================================
# drv887x.py — controlo suave DRV8874/8876
# ======================================================
from machine import Pin, PWM
from config import CONFIG, debug
import time

class DRV:
    def __init__(self, en_pin, ph_pin, nsleep_pin, motor_id=1):
        self.id = motor_id
        self.en = PWM(Pin(en_pin), freq=CONFIG.get("MOTOR_PWM_FREQ", 20000))
        self.ph = Pin(ph_pin, Pin.OUT)
        self.nsleep = Pin(nsleep_pin, Pin.OUT)
        self.running = False
        self.direction = None
        self.stop()
        debug(f"[DRV{self.id}] STOP inicial")

    def enable(self):
        self.nsleep.value(1)

    def disable(self):
        self.en.duty_u16(0)
        self.nsleep.value(0)
        debug(f"[DRV{self.id}] DISABLE")

    def _ramp_pwm(self, start, end, step_ms=2):
        step = 4096 if start < end else -4096
        for val in range(start, end + step, step):
            val = max(0, min(65535, val))
            self.en.duty_u16(val)
            time.sleep_ms(step_ms)

    def run(self, direction):
        dir_up = direction.lower().startswith("open")
        self.direction = "open" if dir_up else "close"
        self.enable()
        self.ph.value(1 if dir_up else 0)
        self._ramp_pwm(0, 65535, step_ms=CONFIG.get("MOTOR_RAMP_STEP_MS", 2))
        self.running = True
        debug(f"[DRV{self.id}] RUN dir={self.direction.upper()}")

    def stop(self):
        """Paragem suave → COAST"""
        self._ramp_pwm(65535, 0, step_ms=CONFIG.get("MOTOR_RAMP_STEP_MS", 2))
        self.en.duty_u16(0)
        self.ph.value(1)  # roda livre (coast)
        self.running = False
        self.direction = None
        debug(f"[DRV{self.id}] STOP (coast mode)")
