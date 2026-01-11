# ======================================================
# mcp23017.py — gestão das entradas via MCP23017 (I2C)
# ======================================================
from machine import I2C, Pin
from app.config import CONFIG, debug
import time

print("entrou em mcp23017.py")

SDA_PIN = 21
SCL_PIN = 22
RESET_PIN = 27
I2C_ADDR = 0x20
i2c = None

_last_state = None  # guarda último estado lido

# ------------------------------------------------------
def init():
    """Inicializa o MCP23017"""
    global i2c
    try:
        i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        reset = Pin(RESET_PIN, Pin.OUT)
        reset.value(1)

        # Entradas + pull-up interno
        i2c.writeto_mem(I2C_ADDR, 0x00, b'\xFF')  # IODIRA
        i2c.writeto_mem(I2C_ADDR, 0x01, b'\xFF')  # IODIRB
        i2c.writeto_mem(I2C_ADDR, 0x0C, b'\xFF')  # GPPUA
        i2c.writeto_mem(I2C_ADDR, 0x0D, b'\xFF')  # GPPUB

        debug("MCP23017 init OK")
    except Exception as e:
        debug(f"Erro init MCP23017: {e}")

# ------------------------------------------------------
def read_reg(reg):
    try:
        return i2c.readfrom_mem(I2C_ADDR, reg, 1)[0]
    except Exception as e:
        debug(f"Erro ler MCP23017 reg {reg}: {e}")
        return 0xFF

# ------------------------------------------------------
def address():
    """Leitura dos bits GPB0–5 para definir ADDR"""
    try:
        gpb = read_reg(0x13)
        addr = gpb & 0x3F
        debug(f"MCP address={addr}")
        return addr
    except Exception:
        return 0

# ------------------------------------------------------
def endstops_and_faults():
    gpa = read_reg(0x12)
    gpb = read_reg(0x13)

    def bit(val, n):
        return bool(val & (1 << n))

    state = {
        "m1_open":  bit(gpa, CONFIG["M1_OPEN_BIT"]),
        "m1_close": bit(gpa, CONFIG["M1_CLOSE_BIT"]),
        "m2_open":  bit(gpa, CONFIG["M2_OPEN_BIT"]),
        "m2_close": bit(gpa, CONFIG["M2_CLOSE_BIT"]),

        # NFAULT continuam no GPB
        "nf1": not bit(gpb, 6),
        "nf2": not bit(gpb, 7),
    }

    return state
