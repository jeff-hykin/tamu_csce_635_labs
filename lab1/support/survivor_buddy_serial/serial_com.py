import os
import time

# 
# helpers
# 
def ask_user_select_item(items):
    for index, item in enumerate(items):
        print(f"{index + 1}. {item}")
    while True:
        try:
            choice = int(input("Please select an item by entering its number: "))
            if 1 <= choice <= len(items):
                return items[choice - 1]
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

# this function probably makes no sense out of context
def _increment_joint_value(each_diff, running_value, target_value):
    if each_diff == 0:
        pass
    if each_diff > 0:
        running_value += 1
        if running_value > target_value:
            running_value = target_value
    else:
        running_value -= 1
        if running_value < target_value:
            running_value = target_value
    
    return running_value
# 
# 
# Setup the serial port
# 
# 
class SurvivorBuddy:
    """
        NOTE:
            - "pitch" is a motion like nodding your head "yes"
            - "yaw" is a motion like nodding your head "no"
            - "roll" is motion like tilting your head to one side in confusion/questioning
    """
    neck_pitch_min = 35;neck_pitch_max = 150; # bigger = more forwards
    neck_yaw_min   = 20;neck_yaw_max   = 160; # smaller = MY left, survivor buddy's right
    head_roll_min  = 0 ;head_roll_max  = 180; # bigger = counterclockwise from MY persepctive 
    head_pitch_min = 30;head_pitch_max = 120; # bigger= down
    
    # phone pass: 3112
    def __init__(self, port_address=None, baud_rate=115200, windows_default_adress="COM4", linux_default_address="/dev/ttyUSB0"):
        import threading
        import serial
        connection_path = ""
        if port_address != None:
            connection_path = port_address
        else:
            import platform
            is_windows_os = platform.system().lower() == 'nt'
            is_mac_os = platform.system().lower() == 'darwin'
            if is_windows_os:
                # note: might need to try different COM ports
                connection_path = windows_default_adress
            elif is_mac_os:
                # List all files in the directory
                usb_serial_paths =  [ f"/dev/{file_name}" for file_name in os.listdir('/dev') if file_name.startswith("tty.usbserial-") ]
                if len(usb_serial_paths) == 0:
                    raise Exception(f'''I don't see any available USB connections (e.g. nothing matches /dev/tty.usbserial-XXXX)''')
                if len(usb_serial_paths) == 1:
                    connection_path = usb_serial_paths[0]
                else:
                    print("Which USB connection do you think it is?")
                    connection_path = ask_user_select_item(usb_serial_paths)
            else:
                connection_path = linux_default_address
        
        self.connection_path = connection_path
        self.scheduled_actions = []
        self.positions = [90,90,90,90]
        
        self.connection = serial.Serial(connection_path, baud_rate)
        
        while self.connection.in_waiting:
            print(self.connection.readline())
        self.connection.write(b"1")
        while self.connection.in_waiting:
            print(self.connection.readline())
        
        # thread is only needed for move_joint_slowly (otherwise the sleep() in the thread would slow everything else down)
        self.still_running = False
        def thing():
            while self.still_running:
                while self.connection.in_waiting:
                    self.connection.readline()
                if len(self.scheduled_actions) > 1:
                    action, positions = self.scheduled_actions.pop(0)
                    neck_pitch, neck_yaw, head_roll, head_pitch = positions
                    neck_pitch  = f"{int(neck_pitch)}".rjust(3, "0")
                    neck_yaw    = f"{int(neck_yaw)}".rjust(3, "0")
                    head_roll   = f"{int(head_roll)}".rjust(3, "0")
                    head_pitch  = f"{int(head_pitch)}".rjust(3, "0")
                    self.connection.write(bytes(f"""{neck_pitch}{neck_yaw}{head_roll}{head_pitch}\n""", "utf-8"))
                    self.positions = positions
                    # without this sleep, even 1000 scheduled actions get executed more or less instantly
                    time.sleep(wait_time)
        
        self.thread = threading.Thread(target=thing)
        self.thread.start()
    
    def __del__(self):
        self.still_running = False
        self.thread.join()
    
    def move_joint(self, neck_pitch, neck_yaw, head_roll, head_pitch, speed=40):
        """
            neck_pitch: leaning forward/back, bigger = more forwards
            neck_yaw: left and right swivel, smaller = more to OUR left, survivor buddy's right
            head_roll: top of the head goes to the left, bottom of the head goes to the right, bigger = counterclockwise from MY persepctive 
            head_pitch: nodding head up down, bigger = down
            speed: 1 to 100
        """
        # NOTE: survivor buddy can actually move a bit faster than speed 1, but it very very very much risks damage to the parts from whiplash
        assert speed <= 100 and speed >= 0.1, "Speed of an action must be in the range 0.1 to 100" 
        assert neck_pitch >= SurvivorBuddy.neck_pitch and neck_pitch <= SurvivorBuddy.neck_pitch_max 
        assert neck_yaw   >= SurvivorBuddy.neck_yaw   and neck_yaw   <= SurvivorBuddy.neck_yaw_max   
        assert head_roll  >= SurvivorBuddy.head_roll  and head_roll  <= SurvivorBuddy.head_roll_max  
        assert head_pitch >= SurvivorBuddy.head_pitch and head_pitch <= SurvivorBuddy.head_pitch_max
        
        positions = list(self.positions)
        self.scheduled_actions.clear()
        scheduled_actions = []
        
        diffs = [ (each1 - each2) for each1, each2 in zip((neck_pitch, neck_yaw, head_roll, head_pitch),positions)]
        neck_pitch_diff, neck_yaw_diff, head_roll_diff, head_pitch_diff = diffs
        new_neck_pitch, new_neck_yaw, new_head_roll, new_head_pitch = positions
        
        for _ in range(int(max(abs(each) for each in diffs))):
            # neck_pitch, neck_yaw, head_roll, head_pitch = self.positions = [ base+change for change, base in zip(diffs, self.positions) ]
            new_neck_pitch = _increment_joint_value(neck_pitch_diff, new_neck_pitch, neck_pitch)
            new_neck_yaw   = _increment_joint_value(neck_yaw_diff, new_neck_yaw, neck_yaw)
            new_head_roll  = _increment_joint_value(head_roll_diff, new_head_roll, head_roll)
            new_head_pitch = _increment_joint_value(head_pitch_diff, new_head_pitch, head_pitch)
            
            scheduled_actions.append(
                (
                    new_neck_pitch, new_neck_yaw, new_head_roll, new_head_pitch
                ),
                0.002/(speed/100)
                # ex: speed=100    => 0.002 wait time
                # ex: speed= 50    => 0.004 wait time
                # ex: speed= 0.1   => 2.000 wait time (2 full seconds, times the number of sub-steps; insanely slow)
            )
        
        # add all of them at the end to reduce thread-locking problems
        self.scheduled_actions += scheduled_actions
        