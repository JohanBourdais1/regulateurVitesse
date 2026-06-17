import struct, time, threading, random, os, serial

SERIAL_PORT  = "/dev/cu.usbserial-0001"
BAUD_RATE    = 115200

MSG_SETPOINT = 0x01
MSG_SPEED    = 0x02
MSG_MODE_SET = 0x05
MSG_OUTPUT   = 0x80
MSG_STATS    = 0x83
MSG_ALARM    = 0x85

MOTOR_GAIN    = 1.65
DRAG_COEFF    = 0.9
MODEL_RESPONSE= 1.2
MAX_SPEED     = 220.0

results = []

def record(name, passed, detail=""):
    results.append((name, passed, detail))
    mark = "BON " if passed else "FAUX "
    print(f"  {mark} [{'PASS' if passed else 'FAIL'}] {name}" +
          (f" — {detail}" if detail else ""))


class StressTester:
    def __init__(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        self.lock = threading.Lock()

        self.speed       = 50.0
        self.setpoint    = 90.0
        self.last_output = 0.0
        self.running     = True

        # Collecte thread-safe
        self.output_times   = []
        self.output_values  = []
        self.stats_frames   = []
        self.alarm_messages = []

        # Keepalive : envoie SPEED en continu en arrière-plan
        self._keepalive_on = threading.Event()
        self._keepalive_on.clear()

    # ── Protocole ────────────────────────────────────────────────

    def _crc(self, data: bytes) -> int:
        c = 0
        for b in data: c ^= b
        return c

    def build(self, msg_type: int, payload: bytes = b"") -> bytes:
        length = len(payload) + 1
        header = struct.pack("<HB", length, msg_type)
        body   = header + payload
        return bytes([0xAA]) + body + bytes([self._crc(body)])

    def send(self, msg_type: int, payload: bytes = b""):
        with self.lock:
            self.ser.write(self.build(msg_type, payload))

    def send_raw(self, data: bytes):
        with self.lock:
            self.ser.write(data)

    def send_float(self, t, v): self.send(t, struct.pack("<f", v))
    def send_mode(self, m):     self.send(MSG_MODE_SET, bytes([m]))

    # ── Réception (byte-by-byte, robuste) ────────────────────────

    def receiver(self):
        """Parseur octet par octet — résistant aux ESP_LOG parasites."""
        state = "WAIT_START"
        buf   = b""
        expected = 0

        while self.running:
            try:
                byte = self.ser.read(1)
            except Exception:
                break
            if not byte:
                continue
            b = byte[0]

            if state == "WAIT_START":
                if b == 0xAA:
                    buf   = b""
                    state = "READ_LEN_LO"

            elif state == "READ_LEN_LO":
                buf += byte
                state = "READ_LEN_HI"

            elif state == "READ_LEN_HI":
                buf += byte
                expected = struct.unpack("<H", buf)[0]
                if expected == 0 or expected > 130:
                    state = "WAIT_START"
                else:
                    state = "READ_BODY"

            elif state == "READ_BODY":
                buf += byte
                # buf = LEN(2) + TYPE(1) + PAYLOAD(expected-1) + CRC(1)
                if len(buf) == expected + 3:  # 2(LEN) + expected + 1(CRC)
                    crc_recv = buf[-1]
                    body     = buf[:-1]  # LEN + TYPE + PAYLOAD
                    if self._crc(body) == crc_recv:
                        msg_type = body[2]
                        payload  = body[3:]
                        self._handle(msg_type, payload)
                    state = "WAIT_START"

    def _handle(self, msg_type, payload):
        if msg_type == MSG_OUTPUT and len(payload) >= 4:
            val = struct.unpack("<f", payload[:4])[0]
            with self.lock:
                self.last_output = val
                self.output_times.append(time.time())
                self.output_values.append(val)
        elif msg_type == MSG_STATS and len(payload) >= 16:
            vals = struct.unpack("<4I", payload[:16])
            with self.lock:
                self.stats_frames.append(vals)
        elif msg_type == MSG_ALARM:
            msg = payload.decode(errors="ignore")
            with self.lock:
                self.alarm_messages.append((time.time(), msg))
            print(f"\n    [ALERTE ECU] : {msg}")

    # ── Keepalive ─────────────────────────────────────────────────

    def _keepalive_loop(self):
        """Thread: envoie SPEED toutes les 100ms quand activé."""
        last = time.time()
        while self.running:
            if self._keepalive_on.is_set():
                now = time.time()
                dt  = now - last
                last = now
                accel = MODEL_RESPONSE * (MOTOR_GAIN * self.last_output
                                          - DRAG_COEFF * self.speed)
                self.speed = max(0.0, min(MAX_SPEED,
                                          self.speed + accel * dt))
                self.send_float(MSG_SPEED, self.speed)
            else:
                last = time.time()
            time.sleep(0.1)

    def keepalive_start(self):
        self.send_float(MSG_SETPOINT, self.setpoint)
        self.send_mode(2)
        self._keepalive_on.set()
        time.sleep(0.3)  # laisser l'ECU passer en AUTO

    def keepalive_stop(self):
        self._keepalive_on.clear()

    # ── Helpers ───────────────────────────────────────────────────

    def wait_output(self, n=1, timeout=2.0) -> int:
        """Attend n nouveaux OUTPUT. Retourne le nombre reçu."""
        with self.lock:
            before = len(self.output_values)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                got = len(self.output_values) - before
            if got >= n:
                return got
            time.sleep(0.05)
        with self.lock:
            return len(self.output_values) - before

    def wait_alarm(self, timeout=3.0) -> bool:
        with self.lock:
            before = len(self.alarm_messages)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                if len(self.alarm_messages) > before:
                    return True
            time.sleep(0.05)
        return False

    def last_stats_snap(self):
        with self.lock:
            return self.stats_frames[-1] if self.stats_frames else None

    # ═══════════════════════════════════════════════════════════════
    # TESTS
    # ═══════════════════════════════════════════════════════════════

    def test_output_periodicity(self):
        print("\n[TEST 1] Périodicité OUTPUT (100ms, jitter < 50ms)")
        self.keepalive_start()
        with self.lock:
            self.output_times.clear()
        time.sleep(5.0)
        self.keepalive_stop()

        with self.lock:
            times = list(self.output_times)

        if len(times) < 5:
            record("Périodicité OUTPUT", False, f"seulement {len(times)} reçus")
            return
        intervals = [times[i+1]-times[i] for i in range(len(times)-1)]
        avg     = sum(intervals)/len(intervals)*1000
        max_dev = max(abs(t*1000-100) for t in intervals)
        record("Périodicité — moyenne ~100ms", 90 < avg < 115,
               f"moy={avg:.1f}ms")
        record("Périodicité — jitter < 50ms",  max_dev < 50,
               f"écart max={max_dev:.1f}ms")

    def test_concatenated_frames(self):
        print("\n[TEST 2] Trames concaténées (20 frames d'un bloc)")
        self.keepalive_start()
        got_before = self.wait_output(3, timeout=1.0)  # calibration

        blob = b"".join(self.build(MSG_SPEED, struct.pack("<f", 70.0))
                        for _ in range(20))
        self.send_raw(blob)
        new = self.wait_output(3, timeout=2.0)
        self.keepalive_stop()

        record("Trames concaténées — OUTPUT continue", new >= 3,
               f"+{new} OUTPUT après injection")

    def test_fragmented_frame(self):
        print("\n[TEST 3] Trame fragmentée (envoyée en 3 morceaux)")
        self.keepalive_start()
        with self.lock:
            alarms_before = len(self.alarm_messages)

        frame = self.build(MSG_SPEED, struct.pack("<f", 75.0))
        self.send_raw(frame[:3]);  time.sleep(0.3)
        self.send_raw(frame[3:6]); time.sleep(0.3)
        self.send_raw(frame[6:]);  time.sleep(0.3)

        new = self.wait_output(3, timeout=1.5)
        self.keepalive_stop()
        with self.lock:
            alarms_after = len(self.alarm_messages)

        record("Trame fragmentée — pas de faux failsafe",
               alarms_after == alarms_before)
        record("Trame fragmentée — OUTPUT continue", new >= 3,
               f"+{new} OUTPUT")

    def test_bad_crc(self):
        print("\n[TEST 4] CRC invalides (10 trames corrompues)")
        self.keepalive_start()
        # On injecte des mauvais CRC PENDANT la régulation normale
        for _ in range(10):
            frame = bytearray(self.build(MSG_SPEED, struct.pack("<f", 60.0)))
            frame[-1] ^= 0xFF
            self.send_raw(bytes(frame))
        new = self.wait_output(5, timeout=2.0)
        self.keepalive_stop()

        record("CRC invalides — ECU continue à fonctionner", new >= 5,
               f"+{new} OUTPUT pendant injection CRC invalides")

    def test_bad_crc_stats(self):
        print("\n[TEST 5] Compteur rx_crc_err dans STATS")
        self.keepalive_start()
        time.sleep(1.5)  # attendre au moins 1 STATS

        snap_before = self.last_stats_snap()
        for _ in range(15):
            frame = bytearray(self.build(MSG_SPEED, struct.pack("<f", 60.0)))
            frame[-1] ^= 0xFF
            self.send_raw(bytes(frame))

        time.sleep(2.0)  # attendre 2 STATS
        snap_after = self.last_stats_snap()
        self.keepalive_stop()

        if snap_before and snap_after:
            delta = snap_after[1] - snap_before[1]
            record("rx_crc_err incrémenté dans STATS", delta >= 5,
                   f"+{delta} erreurs CRC comptées")
        else:
            record("rx_crc_err incrémenté dans STATS", False,
                   "STATS non disponibles")

    def test_unknown_ids(self):
        print("\n[TEST 6] IDs inconnus")
        self.keepalive_start()
        for bad_id in [0x10, 0x42, 0x7F, 0xFE, 0xA0]:
            self.send(bad_id, os.urandom(4))
        new = self.wait_output(5, timeout=2.0)
        self.keepalive_stop()

        record("IDs inconnus — ECU continue", new >= 5,
               f"+{new} OUTPUT après injection")

    def test_bad_length(self):
        print("\n[TEST 7] LEN aberrants (0, 0xFFFF, 200)")
        self.keepalive_start()
        self.send_raw(bytes([0xAA, 0x00, 0x00, 0x00]))
        self.send_raw(bytes([0xAA, 0xFF, 0xFF, 0x01, 0x00]))
        self.send_raw(bytes([0xAA, 0xC8, 0x00, 0x01, 0x00]))
        new = self.wait_output(5, timeout=2.0)
        self.keepalive_stop()

        record("LEN aberrants — pas de crash", new >= 5,
               f"+{new} OUTPUT après injection")

    def test_0xaa_parasite(self):
        print("\n[TEST 8] Byte 0xAA parasite au milieu d'une trame")
        self.keepalive_start()
        frame    = self.build(MSG_SPEED, struct.pack("<f", 65.0))
        poisoned = frame[:4] + bytes([0xAA]) + frame[4:]
        self.send_raw(poisoned)
        new = self.wait_output(5, timeout=2.0)
        self.keepalive_stop()

        record("0xAA parasite — parser se resynchronise", new >= 5,
               f"+{new} OUTPUT après injection")

    def test_flood(self):
        print("\n[TEST 9] Flood 200 trames aléatoires")
        self.keepalive_start()
        blob = b"".join(
            self.build(random.randint(0, 0xFF),
                       os.urandom(random.randint(0, 8)))
            for _ in range(200))
        self.send_raw(blob)
        new = self.wait_output(5, timeout=3.0)
        self.keepalive_stop()

        record("Flood 200 trames — OUTPUT tient sous charge", new >= 5,
               f"+{new} OUTPUT pendant/après flood")

    def test_stats_counters(self):
        print("\n[TEST 10] Cohérence compteurs STATS")
        self.keepalive_start()
        time.sleep(3.0)  # 3 trames STATS minimum
        self.keepalive_stop()

        with self.lock:
            frames = list(self.stats_frames)

        if len(frames) < 2:
            record("Compteurs STATS disponibles", False,
                   f"seulement {len(frames)} STATS reçues")
            return

        monotone = all(frames[i][j] <= frames[i+1][j]
                       for i in range(len(frames)-1) for j in range(4))
        last = frames[-1]
        record("STATS cumulatives (toujours croissantes)", monotone,
               f"dernier: ok={last[0]} crc_err={last[1]} drop={last[2]} tx={last[3]}")
        record("rx_valid > 0",   last[0] > 0,  f"rx_valid={last[0]}")
        record("tx_output > 0",  last[3] > 0,  f"tx_output={last[3]}")

    def test_failsafe_timeout(self):
        print("\n[TEST 11] Failsafe timeout (silence 2s)")
        # Démarrer la régulation puis couper totalement
        self.keepalive_start()
        self.wait_output(3, timeout=1.5)
        self.keepalive_stop()  # silence total

        with self.lock:
            alarms_before = len(self.alarm_messages)

        # Attendre le failsafe (2s) + marge
        got_alarm = self.wait_alarm(timeout=4.0)

        with self.lock:
            last_out = self.last_output

        record("Failsafe timeout — ALARM 0x85 reçue", got_alarm,
               f"+{int(got_alarm)} ALARM")
        record("Failsafe timeout — output forcé à 0",
               abs(last_out) < 1e-3, f"output={last_out:.3f}")

    def test_failsafe_recovery(self):
        print("\n[TEST 12] Reprise après failsafe")
        # L'ECU est en MODE_OFF après le failsafe
        with self.lock:
            self.output_values.clear()

        self.keepalive_start()
        new = self.wait_output(5, timeout=3.0)
        self.keepalive_stop()

        record("Reprise après failsafe — OUTPUT reprend", new >= 5,
               f"+{new} OUTPUT après MODE_SET(AUTO)")

    def test_rapid_mode_switches(self):
        print("\n[TEST 13] Commutations rapides OFF/AUTO × 5")
        self.keepalive_start()
        with self.lock:
            alarms_before = len(self.alarm_messages)

        for _ in range(5):
            self.send_mode(0);  time.sleep(0.05)
            self.send_mode(2);  time.sleep(0.05)
        self.send_mode(2)

        new = self.wait_output(5, timeout=2.0)
        self.keepalive_stop()

        with self.lock:
            alarms_after = len(self.alarm_messages)

        record("Commutations rapides — pas d'ALARM inattendue",
               alarms_after == alarms_before,
               f"+{alarms_after - alarms_before} ALARM")
        record("Commutations rapides — OUTPUT reprend", new >= 5,
               f"+{new} OUTPUT")

    # ═══════════════════════════════════════════════════════════════
    # LANCEMENT
    # ═══════════════════════════════════════════════════════════════

    def run(self):
        rx = threading.Thread(target=self.receiver, daemon=True)
        ka = threading.Thread(target=self._keepalive_loop, daemon=True)
        rx.start(); ka.start()

        print("=" * 60)
        print("  STRESS TEST ECU  ")
        print("=" * 60)

        try:
            self.send_mode(0); time.sleep(0.3)  # état propre

            self.test_output_periodicity()
            self.test_concatenated_frames()
            self.test_fragmented_frame()
            self.test_bad_crc()
            self.test_bad_crc_stats()
            self.test_unknown_ids()
            self.test_bad_length()
            self.test_0xaa_parasite()
            self.test_flood()
            self.test_stats_counters()
            self.test_failsafe_timeout()
            self.test_failsafe_recovery()
            self.test_rapid_mode_switches()

        except KeyboardInterrupt:
            print("\n  Interrompu.")
        finally:
            self.running = False
            self.keepalive_stop()
            self.send_mode(0)
            time.sleep(0.2)
            self.ser.close()

        print("\n" + "=" * 60)
        print("  BILAN")
        print("=" * 60)
        passed = sum(1 for _, ok, _ in results if ok)
        total  = len(results)
        for name, ok, detail in results:
            mark = "BON " if ok else "FAUX "
            print(f"  {mark} {name}" + (f" ({detail})" if detail else ""))
        print("-" * 60)
        print(f"  {passed}/{total} tests passés")
        print("=" * 60)


if __name__ == "__main__":
    StressTester().run()
