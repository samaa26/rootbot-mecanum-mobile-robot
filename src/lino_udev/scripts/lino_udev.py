#!/usr/bin/env python3
# import sys
# import os
# import signal
# import io
# from gi import require_version
# require_version('GUdev', '1.0')
# from gi.repository import GLib, GUdev


# udev_rules = ""

# def callback(client, action, device, user_data):
#     global udev_rules

#     vendor_name = device.get_property("ID_VENDOR_ENC")
#     device_id = device.get_property("ID_SERIAL_SHORT")
#     model_id = device.get_property("ID_MODEL_ID")
#     vendor_id = device.get_property("ID_VENDOR_ID")

#     if vendor_id in ["10c4", "067b"]:
#         kernel = "ttyUSB?"
#     else:
#         kernel = "ttyACM?"

#     if action == "add":
#         print(f"{vendor_name} detected! {vendor_id}")

#         while True:
#             reply = input("Create uDev name? (Y/N) ").strip().upper()
#             if reply == "Y":
#                 sym_link = input("What do you want to name this port? : ").strip()
#                 udev_string = (
#                     f'KERNEL=="{kernel}", SUBSYSTEM=="tty", '
#                     f'ATTRS{{idVendor}}=="{vendor_id}", ATTRS{{idProduct}}=="{model_id}", '
#                     f'ATTRS{{serial}}=="{device_id}", MODE="0666", SYMLINK+="{sym_link}"\n'
#                 )
#                 udev_rules += udev_string
#                 break
#             elif reply == "N":
#                 break

#         print("Plug in next device. Press CTRL+C to save your udev rules.\n")

# def save_udev_rules():
#     global udev_rules
#     while True:
#         save_rules = input("Do you want to save your uDev rules? (Y/N) ").strip().upper()
#         if save_rules == "Y":
#             if udev_rules:
#                 with open("58-lino.rules", "w") as file:
#                     file.write(udev_rules)
#                 print("\n58-lino.rules saved.")
#                 print("\nRUN: $ sudo cp 58-lino.rules /etc/udev/rules.d")
#                 print("\nRestart the computer once done.")
#             break
#         elif save_rules == "N":
#             break

# if __name__ == "__main__":
#     print("Plug in your USB device\n")

#     # Initialize GUdev client
#     client = GUdev.Client(subsystems=["usb"])
#     client.connect("uevent", callback, None)

#     try:
#         loop = GLib.MainLoop()
#         loop.run()
#     except KeyboardInterrupt:
#         save_udev_rules()

#     sys.exit()

# import glib
# import gudev
from gi import require_version
require_version('GUdev', '1.0')
from gi.repository import GLib, GUdev
import sys
import signal
import io
import os

def callback(client, action, device, user_data):
    vendor_name = device.get_property("ID_VENDOR_ENC")
    device_id = device.get_property("ID_SERIAL_SHORT")
    model_id = device.get_property("ID_MODEL_ID")
    vendor_id = device.get_property("ID_VENDOR_ID")
    if vendor_id == "10c4" or vendor_id == "067b":
        kernel = "ttyUSB?"
    else:    
        kernel = "ttyACM?"

    if action == "add":
        global udev_rules

        print(f"{vendor_name} detected! {vendor_id}")

        while True:
            reply = input("Create uDev name? (Y/N) ")
            if reply.upper() == 'Y':
                sym_link = input("What do you want to name this port? : ")
                udev_string = "KERNEL==\"%s\", SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"%s\", ATTRS{idProduct}==\"%s\", ATTRS{serial}==\"%s\", MODE=\"0666\" SYMLINK+=\"%s\"\r\n" %(kernel, vendor_id, model_id, device_id, sym_link)
                udev_rules += udev_string
                break

            elif reply.upper() == 'N':
                break

            else:
                pass

        print ("Plug in next device. Press CTRL+C to save your udev rules.\r\n")

if __name__ == "__main__":
    print ("Plug in your usb device\r\n")
    client = GUdev.Client(subsystems=["usb"])
    client.connect("uevent", callback, None)
    udev_rules=""

    try:
        loop = GLib.MainLoop()
        loop.run()

    except KeyboardInterrupt:
        while True:
            save_rules = input(" Do you wany to save your uDev_rules?(Y/N) : ")
            if save_rules.upper() == 'Y':
                if len(udev_rules) > 0:
                    with open("58-lino.rules", "w", encoding="utf-8") as file:
                        file.write(udev_rules)
                    print ("\r\n58-lino.rules saved.")
                    print ("\r\nRUN: $ sudo cp 58-lino.rules /etc/udev/rules.d")
                    print ("\r\nRestart the computer once done.")
                    break
                else:
                    break

            elif save_rules.upper() == 'N':
                break
            else:
                pass
sys.exit()
