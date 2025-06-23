import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 4210))    #ip address of rpi and its socket

print("Awaiting gesture data...")

previousGesture = None

while True:
    try:
        data, addr = sock.recvfrom(1024)
        decoded = data.decode('utf-8')
        parsed = json.loads(decoded)
        gesture = parsed.get("gesture")

        if gesture and gesture != previousGesture:
            print(f"\nGesture Detected: {gesture}")
            previousGesture = gesture

            if gesture == "Aim":
                def handle_aim():
                    print("AIM: Run YOLO → Calculate center → Move servo")
                    # Placeholder logic for now
                    # Later: Run detection + angle mapping + servo
                handle_aim()

            elif gesture == "Shoot":
                def handle_shoot():
                    print("SHOOT: Firing laser!")
                    # Placeholder logic
                    # Later: GPIO HIGH → 1 sec → GPIO LOW
                handle_shoot()

            elif gesture == "Killswitch":
                def handle_killswitch():
                    print("KILLSWITCH: Disabling servo & laser")
                    # Placeholder logic
                    # Later: Set a global safety flag
                handle_killswitch()

        time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
