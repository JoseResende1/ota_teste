# ======================================================
# controller.py — controlo completo: calibração (2 ciclos),
# percentagem, fins de curso, RS485 e gestos manuais
# ======================================================

import time, json, re
from machine import Pin
from app.config import CONFIG, debug
from app import rs485, mcp23017
from app.drv887x import DRV
from machine import ADC
from machine import WDT



print("entrou em controller")

LED = Pin(26, Pin.OUT)
CALIB_FILE = "calib.json"
POS_FILE = "pos.json"


class Controller:
    def __init__(self):
        self.wdt = WDT(timeout=5000)  # timeout em ms (ex: 5 segundos)
        debug("WDT iniciado (5s)")
        
        
        mcp23017.init()
        self.addr = mcp23017.address()
        self.broadcast = CONFIG.get("BROADCAST_ADDR", 128)

        # Motores
        self.m1 = DRV(17, 16, 23, motor_id=1)
        self.m2 = DRV(19, 18, 25, motor_id=2)

        
        # --- ADC corrente motores ---
        self.adc_m1 = ADC(Pin(33))
        self.adc_m2 = ADC(Pin(34))

        self.adc_m1.atten(ADC.ATTN_11DB)  # até ~3.3V
        self.adc_m2.atten(ADC.ATTN_11DB)

        self.adc_m1.width(ADC.WIDTH_12BIT)
        self.adc_m2.width(ADC.WIDTH_12BIT)

        self.last_current_ms = 0
        self.CURRENT_PERIOD_MS = 100  # taxa de amostragem


        # Posições e calibração
        self.positions = {"motor1": 0.0, "motor2": 0.0}
        self.calibration = self.load_calibration()
        self.load_positions()

        self.calib_missing_sent = {1: False, 2: False}

        # Estado do movimento
        self.active_motor = None
        self.active_dir = None
        self.active_start = 0
        
        self.active_start_pos = 0.0   # posição no instante em que arranca
        self.last_pos_tick = 0        # para throttling de gravação (opcional)


        self.is_inverting = False

        # Memorizar último sentido real (para inversão com motor parado)
        self.last_dir = {1: None, 2: None}

        # Estado dos gestos
        self.last_action_time = 0
        self.last_action_type = None
        
        # --- Proteção de corrente ---
        self.overcurrent_count = {1: 0, 2: 0}
        self.overcurrent_latched = {1: False, 2: False}

        # Botões
        self.btn1_pressed = False
        self.btn2_pressed = False
        self.btn1_press_time = 0
        self.btn2_press_time = 0
        self.btn1_short = []
        self.btn2_short = []

        # Timers
        self.last_led = time.ticks_ms()
        self.last_hb = time.ticks_ms()

        rs485.set_rx()
        debug("Sistema iniciado com calibração completa e gestos manuais inteligentes.")

    # --------------------------------------------------
    # Persistência
    # --------------------------------------------------
    def load_calibration(self):
        try:
            with open(CALIB_FILE, "r") as f:
                return json.load(f)
        except:
            return {"motor1": {}, "motor2": {}}

    def save_calibration(self):
        with open(CALIB_FILE, "w") as f:
            json.dump(self.calibration, f)

    def load_positions(self):
        try:
            with open(POS_FILE, "r") as f:
                self.positions.update(json.load(f))
        except:
            pass

    def save_positions(self):
        with open(POS_FILE, "w") as f:
            json.dump(self.positions, f)

    # --------------------------------------------------
    # Heartbeat
    # --------------------------------------------------
    def blink(self):
        LED.value(1 ^ LED.value())

    def heartbeat(self):
        es = mcp23017.endstops_and_faults()
        msg = (
            f"HB,ADDR:{self.addr},"
            f"POS1:{int(self.positions['motor1'])}%,"
            f"POS2:{int(self.positions['motor2'])}%;"
            f"M1_FA:{int(es['m1_open'])},M1_FC:{int(es['m1_close'])};"
            f"M2_FA:{int(es['m2_open'])},M2_FC:{int(es['m2_close'])}"
        )
        debug(f"[HB] POS1={int(self.positions['motor1'])}% "
      f"POS2={int(self.positions['motor2'])}% "
      f"active={self.active_motor.id if self.active_motor else 'None'}")
        rs485.send(msg)

    def is_motor_active(self):
        return self.active_motor is not None
    # --------------------------------------------------
    # Motores
    # --------------------------------------------------
    def start_motor(self, motor, direction):
        self.active_motor = motor            # ← garante sempre
        self.active_dir = direction
        self.last_dir[motor.id] = direction
        self.active_start = time.ticks_ms()

        key = f"motor{motor.id}"
        self.active_start_pos = float(self.positions.get(key, 0.0))

        motor.run(direction)
        debug(f"[M{motor.id}] START → {direction}")


    def stop_motor(self):
        if self.active_motor:
            mid = self.active_motor.id
            key = f"motor{mid}"
            self.active_motor.stop()
            self.save_positions()

            # 🔔 Heartbeat imediato com posição final
            self.heartbeat()
            self.last_hb = time.ticks_ms()

        self.active_motor = None
        self.active_dir = None

    def invert_motor(self, source="manual"):
        if not self.active_motor or self.is_inverting:
            return
        self.is_inverting = True
        old = self.active_dir
        new = "close" if old == "open" else "open"
        self.active_motor.stop()
        time.sleep_ms(CONFIG["MOTOR_INVERT_DELAY_MS"])
        self.start_motor(self.active_motor, new)
        self.is_inverting = False

    # --------------------------------------------------
    # Calibração completa (2 ciclos) — terminar FECHADO (0%)
    # --------------------------------------------------
    def calibrate_motor(self, motor):
        debug(f"[CALIB] Iniciar calibração completa M{motor.id}")

        def move_until(direction, endstop_key, timeout=20000):
            self.start_motor(motor, direction)
            t0 = time.ticks_ms()
            while True:
                es = mcp23017.endstops_and_faults()
                if es[endstop_key]:
                    break
                if time.ticks_diff(time.ticks_ms(), t0) > timeout:
                    motor.stop()
                    debug("CALIB TIMEOUT")
                    return None
                time.sleep_ms(10)
            motor.stop()
            return time.ticks_diff(time.ticks_ms(), t0)

        # ciclo 1
        t_close_1 = move_until("close", f"m{motor.id}_close")
        time.sleep_ms(200)
        t_open_1 = move_until("open", f"m{motor.id}_open")
        time.sleep_ms(200)

        # ciclo 2
        t_close_2 = move_until("close", f"m{motor.id}_close")
        time.sleep_ms(200)

        # NÃO abre no final → termina fechado (0%)
        # tempos abertos: só do ciclo 1 e fase de abertura do ciclo 1
        t_open_2 = t_open_1

        t_close = int((t_close_1 + t_close_2) / 2)
        t_open = int((t_open_1 + t_open_2) / 2)

        key = f"motor{motor.id}"
        self.calibration[key] = {"open_ms": t_open, "close_ms": t_close}
        self.save_calibration()

        # posição final = 0% (fechado)
        self.positions[key] = 0
        self.save_positions()

        rs485.send(
            f"ACK ADDR:{self.addr} CALIB M{motor.id} OPEN:{t_open} CLOSE:{t_close}"
        )

        debug(f"[CALIB] M{motor.id} FINAL POS=0% (fechado)")

        self.calib_missing_sent[motor.id] = False

    #---------------------
    #VERIFICAÇÃO DE FALTA DE CALIBRAÇÃO
    #------------------
    def is_calibrated(self, motor_id):
        key = f"motor{motor_id}"
        calib = self.calibration.get(key)
        if not calib:
            return False
        return calib.get("open_ms", 0) > 0 and calib.get("close_ms", 0) > 0
    
    
    def warn_not_calibrated(self, motor_id):
        if self.calib_missing_sent[motor_id]:
            return

        msg = f"ALERT ADDR:{self.addr} M{motor_id} NOT_CALIBRATED"
        rs485.send(msg)
        debug(f"[WARN] Motor {motor_id} sem calibração")
        self.calib_missing_sent[motor_id] = True
    
    # --------------------------------------------------
    # Atualização de posição
    # --------------------------------------------------
    def update_position(self, key):
        if not self.active_motor or not self.active_dir:
            return
        elapsed = time.ticks_diff(time.ticks_ms(), self.active_start)
        calib = self.calibration.get(key, {})
        if not calib:
            return
        base = calib["open_ms"] if self.active_dir == "open" else calib["close_ms"]
        if base <= 0:
            return
        delta = (elapsed / base) * 100
        if self.active_dir == "open":
            self.positions[key] = min(100, self.positions[key] + delta)
        else:
            self.positions[key] = max(0, self.positions[key] - delta)
        self.save_positions()


    def update_position_live(self):
        if not self.active_motor or not self.active_dir:
            return

        motor = self.active_motor
        mid = motor.id
        key = f"motor{mid}"

        calib = self.calibration.get(key, {})

        # 🔴 Sem calibração válida → alerta UMA vez e congela posição
        if calib.get("open_ms", 0) <= 0 or calib.get("close_ms", 0) <= 0:
            self.warn_not_calibrated(mid)
            return

        # ✅ calibração válida → limpa flag de aviso
        self.calib_missing_sent[mid] = False

        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self.active_start)

        base = calib["open_ms"] if self.active_dir == "open" else calib["close_ms"]
        if base <= 0:
            return

        delta = (elapsed / base) * 100.0

        if self.active_dir == "open":
            pos = self.active_start_pos + delta
        else:
            pos = self.active_start_pos - delta

        pos = max(0, min(100, pos))
        self.positions[key] = pos

        debug(f"[POS] M{mid} = {pos:.1f}%")
    # --------------------------------------------------
    # Percentagem
    # --------------------------------------------------
    def move_to_percent(self, motor, pct):
        key = f"motor{motor.id}"
        calib = self.calibration.get(key, {})

        # 🔴 Sem calibração válida
        if calib.get("open_ms", 0) <= 0 or calib.get("close_ms", 0) <= 0:
            rs485.send(f"NACK ADDR:{self.addr} M{motor.id} SEM_CALIBRACAO")
            debug(f"[WARN] Comando ignorado — M{motor.id} sem calibração")
            return

        cur = self.positions[key]
        delta = pct - cur

        if abs(delta) < 1:
            return

        direction = "open" if delta > 0 else "close"
        base = calib["open_ms"] if delta > 0 else calib["close_ms"]
        ms = abs(delta) / 100 * base

        self.start_motor(motor, direction)
        t0 = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), t0) < ms:
            es = mcp23017.endstops_and_faults()

            if direction == "open" and es[f"m{motor.id}_open"]:
                self.positions[key] = 100
                break

            if direction == "close" and es[f"m{motor.id}_close"]:
                self.positions[key] = 0
                break

            time.sleep_ms(10)

        self.stop_motor()

    # --------------------------------------------------
    # Botões — lógica completa: STOP → INVERTER → STOP → INVERTER
    # --------------------------------------------------
    def handle_button(self, motor, bit, state, t_press, fa, fc, short_list):
        now = time.ticks_ms()
        pressed = (bit == 1)

        if pressed and not state:
            t_press = now

        elif not pressed and state:
            dur = time.ticks_diff(now, t_press)

            # SHORT PRESS
            if 50 <= dur <= 300:
                short_list.append(now)
                short_list[:] = [t for t in short_list if now - t < 3000]

                # 5 short → CALIBRAR
                if len(short_list) >= 5:
                    short_list.clear()
                    self.stop_motor()
                    self.calibrate_motor(motor)
                    return pressed, t_press

                mid = motor.id
                last = self.last_dir[mid]

                # INVERTER/ARRANCAR após STOP
                if self.last_action_type == "stop" and time.ticks_diff(now, self.last_action_time) < 2000:

                    # motor parado → arrancar em direcção oposta
                    if not self.active_motor:
                        new_dir = "close" if last == "open" else "open"
                        self.start_motor(motor, new_dir)
                        self.last_action_type = "invert"
                        self.last_action_time = now
                        return pressed, t_press

                    # motor a mover → inversão normal
                    self.invert_motor()
                    self.last_action_type = "invert"
                    self.last_action_time = now
                    return pressed, t_press

                # STOP normal
                if self.active_motor:
                    self.stop_motor()
                    self.last_action_type = "stop"
                    self.last_action_time = now
                    return pressed, t_press

                # sem inversão — apenas regista
                self.last_action_type = "short"
                self.last_action_time = now
                return pressed, t_press

            # LONG PRESS
            elif 500 <= dur <= 1500:
                if self.active_motor == motor:
                    # inverter com motor em movimento
                    self.invert_motor()
                    self.last_action_type = "invert"
                else:
                    direction = "close" if fa else "open"
                    self.start_motor(motor, direction)
                    self.last_action_type = "long_start"

                self.last_action_time = now
                return pressed, t_press

        return pressed, t_press

    # --------------------------------------------------
    # RS485
    # --------------------------------------------------
    def handle_command(self, cmd):
        su = cmd.upper().strip()

        m = re.search(r"(\d+)\s*-\s*([12])", su)
        if m:
            pct = int(m.group(1))
            motor = self.m1 if int(m.group(2)) == 1 else self.m2
            self.move_to_percent(motor, pct)
            return

        def run(motor, d):
            if self.active_motor == motor and self.active_dir != d:
                self.invert_motor("rs485")
            else:
                self.start_motor(motor, d)

        if su == "ABRIR1":
            run(self.m1, "open")
        elif su == "FECHAR1":
            run(self.m1, "close")
        elif su == "ABRIR2":
            run(self.m2, "open")
        elif su == "FECHAR2":
            run(self.m2, "close")
        elif su == "STOP":
            self.stop_motor()
        elif su == "CALIBRAR1":
            self.calibrate_motor(self.m1)
        elif su == "CALIBRAR2":
            self.calibrate_motor(self.m2)

        rs485.send(f"ACK {cmd}")



    def read_motor_current(self, motor_id, samples=None):
        if samples is None:
            samples = int(CONFIG.get("CURR_AVG_SAMPLES", 8))

        # média de N amostras (reduz ruído)
        acc = 0
        for _ in range(samples):
            if motor_id == 1:
                acc += self.adc_m1.read()
            else:
                acc += self.adc_m2.read()
            time.sleep_ms(2)

        raw = acc / samples

        # ADC -> tensão (aprox.); atenção: ADC do ESP32 não é perfeito
        v = (raw * 3.3) / 4095.0

        r = float(CONFIG.get("IPROPI_R_OHMS", 1200.0))
        gain = float(CONFIG.get("IPROPI_GAIN", 2500.0))

        # I_motor ≈ (V / R) * GAIN
        i = (v / r) * gain
        return v, i


    def send_current_status(self, motor_id):
        v, i = self.read_motor_current(motor_id)

        if motor_id == 1:
            msg = f"CURR,ADDR:{self.addr},M1_V:{v:.2f}V,M1_I:{i:.2f}A"
        else:
            msg = f"CURR,ADDR:{self.addr},M2_V:{v:.2f}V,M2_I:{i:.2f}A"

        # envia no RS485
        rs485.send(msg)

        # envia também para o monitor série (USB)
        print(msg)

        # e mantém o teu debug “bonito”
        debug(f"[CURR] M{motor_id} {i:.2f}A ({v:.2f}V)")


    def check_overcurrent(self, motor_id):
        if not CONFIG.get("ENABLE_OVERCURRENT_PROTECT", True):
            return False

        if self.overcurrent_latched[motor_id]:
            return True

        v, i = self.read_motor_current(motor_id)

        limit = CONFIG.get("OVERCURRENT_LIMIT_A", 1.2)

        if i > limit:
            self.overcurrent_count[motor_id] += 1
            debug(f"[OC] M{motor_id} {i:.2f}A > {limit:.2f}A "
                  f"({self.overcurrent_count[motor_id]})")
        else:
            self.overcurrent_count[motor_id] = 0

        if self.overcurrent_count[motor_id] >= CONFIG.get("OVERCURRENT_SAMPLES", 3):
            self.overcurrent_latched[motor_id] = True
            self.handle_overcurrent_trip(motor_id, v, i)
            return True

        return False
    
    def handle_overcurrent_trip(self, motor_id, v, i):
        debug(f"[EMERG] M{motor_id} OVERCURRENT {i:.2f}A")

        # parar motor imediatamente
        if self.active_motor and self.active_motor.id == motor_id:
            self.active_motor.stop()

        self.active_motor = None
        self.active_dir = None

        # mensagem RS485
        msg = (
            f"ALERT,ADDR:{self.addr},"
            f"M{motor_id}_OVERCURRENT,"
            f"I:{i:.2f}A,V:{v:.2f}V"
        )
        rs485.send(msg)

        # heartbeat imediato
        self.heartbeat()
        self.last_hb = time.ticks_ms()



    # --------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------
    def loop(self):
        while True:
            now = time.ticks_ms()

            # --------------------------------------------------
            # LED heartbeat visual
            # --------------------------------------------------
            if time.ticks_diff(now, self.last_led) > CONFIG["LED_BLINK_MS"]:
                self.blink()
                self.last_led = now

            # --------------------------------------------------
            # MOTOR ATIVO
            # --------------------------------------------------
            if self.active_motor:
                mid = self.active_motor.id

                # Atualização de posição
                self.update_position_live()

                # Corrente + proteção
                if time.ticks_diff(now, self.last_current_ms) >= CONFIG["CURRENT_PERIOD_MS"]:
                    self.send_current_status(mid)        # RS485 + serial
                    self.check_overcurrent(mid)          # proteção
                    self.last_current_ms = now

            # --------------------------------------------------
            # HEARTBEAT (dinâmico)
            # --------------------------------------------------
            hb_period = CONFIG["HEARTBEAT_ACTIVE_MS"] if self.active_motor else CONFIG["HEARTBEAT_IDLE_MS"]

            if time.ticks_diff(now, self.last_hb) >= hb_period:
                self.heartbeat()
                self.last_hb = now

            # --------------------------------------------------
            # MCP23017 / Botões
            # --------------------------------------------------
            gpa = mcp23017.read_reg(0x12)
            b1 = (gpa >> 4) & 1
            b2 = (gpa >> 5) & 1
            es = mcp23017.endstops_and_faults()

            self.btn1_pressed, self.btn1_press_time = self.handle_button(
                self.m1, b1, self.btn1_pressed, self.btn1_press_time,
                es["m1_open"], es["m1_close"], self.btn1_short
            )

            self.btn2_pressed, self.btn2_press_time = self.handle_button(
                self.m2, b2, self.btn2_pressed, self.btn2_press_time,
                es["m2_open"], es["m2_close"], self.btn2_short
            )

            # --------------------------------------------------
            # SEGURANÇA: FINS DE CURSO / TIMEOUT
            # --------------------------------------------------
            if self.active_motor:
                mid = self.active_motor.id
                key = f"motor{mid}"

                if self.active_dir == "open" and es[f"m{mid}_open"]:
                    self.positions[key] = 100
                    self.stop_motor()

                elif self.active_dir == "close" and es[f"m{mid}_close"]:
                    self.positions[key] = 0
                    self.stop_motor()

                elif time.ticks_diff(now, self.active_start) > CONFIG["MOTOR_TIMEOUT_MS"]:
                    rs485.send(f"FAULT,ADDR:{self.addr},M{mid}_TIMEOUT")
                    debug(f"[FAULT] TIMEOUT M{mid}")
                    self.stop_motor()

            # --------------------------------------------------
            # RS485 RX
            # --------------------------------------------------
            for line in rs485.read_lines():
                if not line or not line.upper().startswith("ADDR"):
                    continue

                parts = line.replace("ADDR:", "", 1).split(None, 1)
                if len(parts) == 2 and parts[0].isdigit():
                    addr = int(parts[0])
                    if addr in (self.addr, self.broadcast):
                        self.handle_command(parts[1].strip())

            # --------------------------------------------------
            # WATCHDOG
            # --------------------------------------------------
            self.wdt.feed()

            time.sleep_ms(CONFIG["RS485_POLL_MS"])

                
            

