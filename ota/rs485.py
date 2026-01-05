
# ======================================================
# rs485.py — gestão do barramento RS485 com tempos seguros
# ======================================================
from machine import UART, Pin
import time
from config import debug

uart = UART(1, baudrate=115200, tx=13, rx=14, timeout=50)
DE = Pin(5, Pin.OUT)
RE = Pin(4, Pin.OUT)

def set_tx():
    DE.value(1)
    RE.value(1)
    debug("RS485 -> TX mode")

def set_rx():
    DE.value(0)
    RE.value(0)
    debug("RS485 -> RX mode")

def send(msg):
    """Envia mensagem RS485 com temporizações seguras."""
    set_tx()
    time.sleep_ms(4)
    uart.write(msg + "\r\n")
    debug("[TX] " + msg)
    time.sleep_ms(4)
    set_rx()
    time.sleep_ms(2)

def read_line(timeout_ms=0):
    """Lê uma linha RS485 com timeout opcional."""
    if timeout_ms <= 0:
        if uart.any():
            try:
                line = uart.readline()
                if line:
                    line = line.decode().strip()
                    debug(f"[RX] {line}")
                    return line
            except Exception:
                pass
        return None

    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        if uart.any():
            try:
                line = uart.readline()
                if line:
                    line = line.decode().strip()
                    debug(f"[RX] {line}")
                    return line
            except Exception:
                pass
        time.sleep_ms(2)
    return None

def read_lines():
    """Lê todas as linhas disponíveis (modo contínuo)."""
    lines = []
    if uart.any():
        try:
            raw = uart.read()
            if not raw:
                return lines
            try:
                text = raw.decode()
            except Exception:
                text = ''.join(chr(b) for b in raw if 32 <= b < 127)
            for line in text.splitlines():
                line = line.strip()
                if line:
                    debug("[RX] " + line)
                    lines.append(line)
        except Exception as e:
            debug("Erro UART read: " + str(e))
    return lines
