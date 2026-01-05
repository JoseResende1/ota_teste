# ======================================================
# mcp23017.py — gestão das entradas via MCP23017 (I2C)
# ======================================================
from machine import I2C, Pin
from config import debug
import time

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
    """
    Fins de curso: ATIVOS em HIGH (1).
      - GPA0 → FA1 (aberto M1)
      - GPA1 → FC1 (fechado M1)
      - GPA2 → FA2 (aberto M2)
      - GPA3 → FC2 (fechado M2)
    NFAULT: ativo em LOW (0).
    """
    global _last_state

    gpa = read_reg(0x12)
    gpb = read_reg(0x13)

    # ENDSTOPS = ACTIVE HIGH
    fa1 = bool(gpa & (1 << 0))  # GPA0
    fc1 = bool(gpa & (1 << 1))  # GPA1
    fa2 = bool(gpa & (1 << 2))  # GPA2
    fc2 = bool(gpa & (1 << 3))  # GPA3

    # NFAULT = ACTIVE LOW -> True quando há FALHA
    nf1 = not bool(gpb & (1 << 6))  # GPB6
    nf2 = not bool(gpb & (1 << 7))  # GPB7

    state = {
        "m1_open":  fa1,
        "m1_close": fc1,
        "m2_open":  fa2,
        "m2_close": fc2,
        "nf1": nf1,
        "nf2": nf2,
    }

    if state != _last_state:
        debug(f"[MCP] GPA={bin(gpa)} GPB={bin(gpb)} -> "
              f"FA1:{fa1} FC1:{fc1} FA2:{fa2} FC2:{fc2} NF1:{nf1} NF2:{nf2}")
        _last_state = state.copy()

    return state
