"""Leap Motion + Arduino:
Cuenta dedos extendidos (1-5) y envía el número al Arduino por Serial.
Además imprime en consola lo que responde el Arduino.
"""

import time
import serial
import leap

print("== CARGANDO leap_fingers_to_arduino.py ==")


class MyListener(leap.Listener):
    def __init__(self, arduino):
        super().__init__()
        self.arduino = arduino
        self.last_count = None  # último número enviado

    # -------- utilitario: enviar número de dedos --------
    def send_count(self, count: int):
        if not self.arduino:
            return

        # Solo nos interesan 1–5 dedos
        if not (1 <= count <= 5):
            return

        # Evitar spamear el mismo valor
        if count == self.last_count:
            return

        msg = f"{count}\n"
        self.arduino.write(msg.encode("ascii"))
        self.last_count = count
        print(f"[PYTHON] Enviado a Arduino: {count}")

        # Leer respuesta rápida del Arduino si la hay
        time.sleep(0.05)
        try:
            if self.arduino.in_waiting:
                line = self.arduino.readline().decode(errors="ignore").strip()
                if line:
                    print(f"[ARDUINO] {line}")
        except Exception as e:
            print("Error leyendo del Arduino:", e)

    # ---------------- eventos Leap ----------------------

    def on_connection_event(self, event):
        print("Conectado al servicio de Leap.")

    def on_device_event(self, event):
        try:
            with event.device.open():
                info = event.device.get_info()
        except leap.LeapCannotOpenDeviceError:
            info = event.device.get_info()

        print(f"Dispositivo Leap encontrado: {info.serial}")

    def on_tracking_event(self, event):
        try:
            print(f"Frame {event.tracking_frame_id} con {len(event.hands)} mano(s).")

            if len(event.hands) == 0:
                print("Sin manos detectadas.")
                return

            # Tomamos solo la primera mano
            hand = event.hands[0]

                        # En tu binding, hand.digits ya es una LISTA (no diccionario)
            digits_attr = getattr(hand, "digits", [])

            # Si por alguna razón fuera dict en otra versión, lo cubrimos:
            if isinstance(digits_attr, dict):
                digits_iter = digits_attr.values()
            else:
                digits_iter = digits_attr  # lista normal

            extended_digits = [
                d for d in digits_iter
                if getattr(d, "is_extended", False)
            ]
            count = len(extended_digits)


            print(f"Dedos extendidos: {count}")
            self.send_count(count)

        except Exception as e:
            print("Error en on_tracking_event:", e)


def main():
    print("== INICIANDO main() ==")

    # -------- abrir serial con Arduino --------
    PORT = "COM7"      # <-- ajusta aquí tu puerto (COM3, COM4, etc.)
    BAUD = 115200

    try:
        arduino = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)
        arduino.reset_input_buffer()
        print(f"Serial con Arduino abierto en {PORT} a {BAUD} bps.")

        # Prueba rápida opcional al iniciar (puedes comentarla si quieres)
        print("Enviando prueba '5' al Arduino...")
        arduino.write(b"5\n")
        time.sleep(0.1)
        if arduino.in_waiting:
            line = arduino.readline().decode(errors="ignore").strip()
            if line:
                print(f"[ARDUINO][TEST] {line}")
    except Exception as e:
        print("ERROR abriendo el puerto serial con Arduino:", e)
        arduino = None

    # -------- configurar Leap --------
    print("Creando conexión con Leap...")
    my_listener = MyListener(arduino)
    connection = leap.Connection()
    connection.add_listener(my_listener)

    print("Abriendo conexión Leap. Ctrl+C para salir.")
    running = True
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)
        while running:
            time.sleep(0.05)


if __name__ == "__main__":
    print("__name__ == '__main__', llamando a main()...")
    main()
