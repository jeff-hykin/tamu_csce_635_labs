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
def move_joint(torso_rot_angle, head_rotation_angle, head_tilt_angle):
    assert torso_rot_angle >= 50 and torso_rot_angle <= 180        # is this yaw? or pitch
    assert head_rotation_angle >= 0 and head_rotation_angle <= 180 # is this roll or yaw? (out of pitch,yaw,roll)
    assert head_tilt_angle >= 30 and head_tilt_angle <= 120        # is this roll or pitch? (out of pitch,yaw,roll)
    
    torso_value       = f"{int(torso_rot_angle)}".rjust(3, "0")
    head_rotate_value = f"{int(head_rotation_angle)}".rjust(3, "0")
    head_tilt_value   = f"{int(head_tilt_angle)}".rjust(3, "0")
    
    ser.write(bytes(f"""090{torso_value}{head_rotate_value}{head_tilt_value}\n""", "utf-8"))
# torso_joint
# neck_swivel
# head_tilt
# head_nod