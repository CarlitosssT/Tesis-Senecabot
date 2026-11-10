"""Envia una secuencia de angulos al motor a traves de la ESP32.

La ESP32 corre Arduino/motor_control/motor_control.ino y traduce los comandos de
texto ("p <grados> <dps>", "e", "s") a tramas CAN. Este script solo habla por el
puerto serie, no necesita un adaptador USB-CAN.

Ejemplos:
    python myactuator_esp32/motor_sequence.py --port COM3 0 90 180:60 -45
    python myactuator_esp32/motor_sequence.py --port COM3 --file secuencia.csv
    python myactuator_esp32/motor_sequence.py --port COM3 --file secuencia.csv --repeat 3

Formato de un paso: angulo[:velocidad_dps[:pausa_s]]
Formato del CSV (una linea por paso, # para comentarios): angulo,velocidad,pausa
"""
import argparse
import re
import sys
import time

import serial

MAX_ANGLE_DEG = 360.0  # mismo limite que MAX_ANGLE_DEG en motor_control.ino
MAX_SPEED_DPS = 300    # mismo limite que MAX_SPEED_DPS en motor_control.ino
ANGLE_RE = re.compile(r"Angulo:\s*(-?\d+)")


class Step:
    def __init__(self, angle, speed, hold):
        if abs(angle) > MAX_ANGLE_DEG:
            raise ValueError(f"angulo {angle} fuera de +-{MAX_ANGLE_DEG:g} grados")
        if not 1 <= speed <= MAX_SPEED_DPS:
            raise ValueError(f"velocidad {speed} fuera de 1-{MAX_SPEED_DPS} dps")
        if hold < 0:
            raise ValueError(f"pausa {hold} negativa")
        self.angle = angle
        self.speed = speed
        self.hold = hold

    def __repr__(self):
        return f"Step({self.angle:g} grados, {self.speed} dps, pausa {self.hold:g} s)"


def parse_step(text, default_speed, default_hold):
    parts = [p.strip() for p in re.split(r"[:,]", text) if p.strip() != ""]
    if not parts or len(parts) > 3:
        raise ValueError(f"paso invalido: {text!r}")
    angle = float(parts[0])
    speed = int(float(parts[1])) if len(parts) > 1 else default_speed
    hold = float(parts[2]) if len(parts) > 2 else default_hold
    return Step(angle, speed, hold)


def load_file(path, default_speed, default_hold):
    steps = []
    with open(path, encoding="utf-8") as f:
        for number, raw in enumerate(f, start=1):
            text = raw.split("#", 1)[0].strip()
            if not text:
                continue
            try:
                steps.append(parse_step(text, default_speed, default_hold))
            except ValueError as exc:
                raise ValueError(f"{path} linea {number}: {exc}") from exc
    return steps


class MotorLink:
    def __init__(self, ser):
        self.ser = ser

    def send(self, command):
        self.ser.write((command + "\n").encode("ascii"))

    def read_angle(self, timeout=1.0):
        """Pide el estado y devuelve el angulo en grados, o None si no hay respuesta."""
        self.ser.reset_input_buffer()
        self.send("e")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace")
            match = ANGLE_RE.search(line)
            if match:
                return int(match.group(1))
        return None

    def move_to(self, step, tolerance, poll_s):
        self.send(f"p {step.angle:g} {step.speed}")
        start = self.read_angle()
        if start is None:
            raise TimeoutError("el motor no responde (revisa el CAN y la alimentacion)")
        distance = abs(step.angle - start)
        timeout = distance / step.speed * 1.5 + 3.0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            angle = self.read_angle()
            if angle is not None and abs(angle - step.angle) <= tolerance:
                return angle
            time.sleep(poll_s)
        raise TimeoutError(
            f"no llego a {step.angle:g} grados en {timeout:.1f} s (ultimo angulo: {angle})")

    def stop(self):
        self.send("s")


def run_sequence(link, steps, tolerance, poll_s, repeat):
    for lap in range(1, repeat + 1):
        for index, step in enumerate(steps, start=1):
            print(f"[{lap}/{repeat}] paso {index}/{len(steps)}: {step}")
            angle = link.move_to(step, tolerance, poll_s)
            print(f"    llego a {angle} grados")
            if step.hold:
                time.sleep(step.hold)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("steps", nargs="*", help="pasos angulo[:velocidad[:pausa]]")
    parser.add_argument("--port", required=True, help="puerto de la ESP32, por ejemplo COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--file", help="CSV con los pasos (angulo,velocidad,pausa)")
    parser.add_argument("--speed", type=int, default=60, help="velocidad por defecto en dps")
    parser.add_argument("--hold", type=float, default=0.5, help="pausa por defecto en segundos")
    parser.add_argument("--tolerance", type=float, default=2.0,
                        help="error de angulo aceptado en grados")
    parser.add_argument("--poll", type=float, default=0.1, help="intervalo de lectura en s")
    parser.add_argument("--repeat", type=int, default=1, help="veces que se repite la secuencia")
    args = parser.parse_args()

    try:
        steps = [parse_step(s, args.speed, args.hold) for s in args.steps]
        if args.file:
            steps += load_file(args.file, args.speed, args.hold)
    except (ValueError, OSError) as exc:
        sys.exit(f"Error en la secuencia: {exc}")
    if not steps:
        sys.exit("No hay pasos: da angulos en la linea de comandos o usa --file")

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(2.5)  # la ESP32 se reinicia al abrir el puerto
        ser.reset_input_buffer()
        link = MotorLink(ser)
        try:
            run_sequence(link, steps, args.tolerance, args.poll, args.repeat)
            print("Secuencia terminada")
        except KeyboardInterrupt:
            print("\nInterrumpido, parando el motor")
        except TimeoutError as exc:
            print(f"Error: {exc}")
            sys.exit(1)
        finally:
            link.stop()


if __name__ == "__main__":
    main()
