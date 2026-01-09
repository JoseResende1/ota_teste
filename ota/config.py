
# ======================================================
# config.py — parâmetros globais e utilitário de debug
# ======================================================
import json

print("entrou em config")

CONFIG_FILENAME = "config.json"

CONFIG_DEFAULTS = {
    # ==================================================
    # 🐞 DEBUG E LOGS
    # ==================================================
    "DEBUG": True,

    # ==================================================
    # 💡 LED
    # ==================================================
    "LED_BLINK_MS": 1000,

    # ==================================================
    # 🔌 RS485
    # ==================================================
    "RS485_BAUD": 9600,
    "RS485_POLL_MS": 10,
    "BROADCAST_ADDR": 128,

    # ==================================================
    # ❤️ HEARTBEAT
    # ==================================================
    "HEARTBEAT_MS": 5000,
    "HEARTBEAT_IDLE_MS": 3000,     # standby
    "HEARTBEAT_ACTIVE_MS": 300,    # motor a mover
    # ==================================================
    # ⚙️ MOTOR / MOVIMENTO
    # ==================================================
    "MOTOR_TIMEOUT_MS": 30000,
    "MOTOR_INVERT_DELAY_MS": 1000,    # Pausa entre STOP e inversão
    "LONG_PRESS_MS": 1000,           # 2s para arrancar/inverter
    "SHORT_PRESS_MS": 100,           # 0.5s para parar
    "MOTOR_RAMP_STEP_MS": 1,         # Passo da rampa PWM
    "MOTOR_PWM_FREQ": 20000,         # Frequência PWM inaudível
    
    ##leitura de corrente
    "CURRENT_PERIOD_MS": 300,     # envia de 300 em 300ms quando motor ativo
    "IPROPI_R_OHMS": 1200.0,      # o teu resistor pulldown
    "IPROPI_GAIN": 2700.0,        # ajustado para bater 0.48V ~ 1A
    "CURR_AVG_SAMPLES": 10,        # média para estabilizar
    
    
    # ==================================================
    # ⚠️ PROTEÇÃO DE CORRENTE
    # ==================================================
    "ENABLE_OVERCURRENT_PROTECT": True,

    "OVERCURRENT_LIMIT_A": 0.70,        # limite em amperes
    "OVERCURRENT_SAMPLES": 3,            # nº leituras consecutivas
    "OVERCURRENT_PERIOD_MS": 100,        # período de amostragem


    # ==================================================
    # 🧲 FINS DE CURSO
    # ==================================================
    "ENDSTOP_ACTIVE_HIGH": True,

    # ==================================================
    # ⚠️ SINAIS DE FALHA
    # ==================================================
    "ENABLE_NFAULT": True,

    # ==================================================
    # 📨 PROTOCOLO
    # ==================================================
    "ENABLE_ACK": True,
    "ENABLE_NACK": True,
}

CONFIG = CONFIG_DEFAULTS.copy()

def debug(msg):
    if CONFIG.get("DEBUG", True):
        print("[DBG]", msg)

def save_config():
    try:
        with open(CONFIG_FILENAME, "w") as f:
            json.dump(CONFIG, f)
        debug("Config salva.")
    except Exception as e:
        print("[ERR] save_config:", e)

def load_config():
    global CONFIG
    try:
        with open(CONFIG_FILENAME, "r") as f:
            d = json.load(f)
            CONFIG.update(d)
            debug("Config carregada.")
    except Exception:
        debug("Usando configurações por defeito.")
        CONFIG.update(CONFIG_DEFAULTS)
