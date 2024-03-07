import serial
import platform
import os

# 
# helpers
# 
def display_menu(items):
    for index, item in enumerate(items):
        print(f"{index + 1}. {item}")

def select_item(items):
    display_menu(items)
    while True:
        try:
            choice = int(input("Please select an item by entering its number: "))
            if 1 <= choice <= len(items):
                return items[choice - 1]
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

# 
# 
# Setup the serial port
# 
# 
if True:
    connection_path = ""
    is_windows_os = platform.system().lower() == 'nt'
    is_mac_os = platform.system().lower() == 'darwin'
    if is_windows_os:
        # note: might need to try different COM ports
        connection_path = "COM4"
    elif is_mac_os:
        # List all files in the directory
        usb_serial_paths =  [ f"/dev/{file_name}" for file_name in os.listdir('/dev') if file_name.startswith("/dev/tty.usbserial-") ]
        if len(usb_serial_paths):
            raise Exception(f'''I don't see any available USB connections (e.g. nothing matches /dev/tty.usbserial-XXXX)''')
        
        print("Which USB connection do you think it is?")
        connection_path = select_item(usb_serial_paths)
    else:
        connection_path = "/dev/ttyUSB0"
    
    ser = serial.Serial(connection_path, 57600)



ser.write(b"1")
def move_joint(base_pitch, base_yaw, head_roll, head_pitch):
    """
        base_pitch: leaning forward/back
        base_yaw: left and right swivel
        head_roll: top of the head goes to the left, bottom of the head goes to the right
        head_pitch: nodding head up down
    """
    assert base_pitch >= 60 and base_pitch <= 150 # these bounds (just this one joint) are probably overly restrictive (untested)
    assert base_yaw >= 50 and base_yaw <= 180
    assert head_roll >= 0 and head_roll <= 180
    assert head_pitch >= 30 and head_pitch <= 120
    
    base_pitch  = f"{int(base_pitch)}".rjust(3, "0")
    base_yaw    = f"{int(base_yaw)}".rjust(3, "0")
    head_roll   = f"{int(head_roll)}".rjust(3, "0")
    head_pitch  = f"{int(head_pitch)}".rjust(3, "0")
    
    ser.write(bytes(f"""{base_pitch}{base_yaw}{head_roll}{head_pitch}\n""", "utf-8"))