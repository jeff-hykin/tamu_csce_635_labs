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
class SurvivorBuddySerial:
    """
    Example:
        survivor_bud = SurvivorBuddySerial()
        survivor_bud.set_joints(
            # NOTE: - "pitch" is a motion like nodding your head "yes"
            #       - "yaw" is a motion like nodding your head "no"
            #       - "roll" is motion like tilting your head to one side in confusion/questioning
            torso_pitch=90, bigger = more forwards
            torso_yaw=90, smaller = OUR left, survivor buddy's right
            head_roll=90, bigger = counterclockwise from OUR persepctive 
            head_pitch=90, bigger= down
            speed=40, # out of 100
        )
    """
    torso_pitch_min = 35;torso_pitch_max = 150; # bigger = more forwards
    torso_yaw_min   = 20;torso_yaw_max   = 160; # smaller = OUR left, survivor buddy's right
    head_roll_min  = 0 ;head_roll_max  = 180; # bigger = counterclockwise from OUR persepctive 
    head_pitch_min = 30;head_pitch_max = 120; # bigger= down
    
    # phone pass: 3112
    def __init__(
        self,
        port_address=None,
        baud_rate=115200,
        windows_default_address="COM4",
        linux_default_address="/dev/ttyUSB0",
        mac_default_address=None, # mac auto-detects name
        inital_positions=[90,90,90,90],
        logging=False,
        include_legacy_survivor_buddy_support=True,
    ):
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
                connection_path = windows_default_address
            elif is_mac_os:
                # List all files in the directory
                usb_serial_paths =  [ f"/dev/{file_name}" for file_name in os.listdir('/dev') if file_name.startswith("tty.usbserial-") ]
                if len(usb_serial_paths) == 0:
                    raise Exception(f'''[SurvivorBuddy Controller] I don't see any available USB connections (e.g. nothing matches /dev/tty.usbserial-XXXX)''')
                if len(usb_serial_paths) == 1:
                    connection_path = usb_serial_paths[0]
                else:
                    if mac_default_address in usb_serial_paths:
                        connection_path = mac_default_address
                    else:
                        print("Which USB connection do you think it is?")
                        print("(Note: you can have this auto-default)")
                        print("(just add SurvivorBuddy(mac_default_address=/dev/somethin'))")
                        connection_path = ask_user_select_item(usb_serial_paths)
            else:
                connection_path = linux_default_address
        
        self.connection_path = connection_path
        self.positions = inital_positions
        initial_delay_time = 0.004 # seconds
        self.scheduled_actions = [
            # move to initial positions
            (initial_delay_time, inital_positions)
        ]
        
        self.connection = serial.Serial(connection_path, baud_rate)
        
        # thread is only needed for move_joint_slowly (otherwise the sleep() in the thread would slow everything else down)
        self.still_running = True
        def thread_function():
            while self.still_running:
                while self.connection.in_waiting:
                    response = self.connection.readline()
                    if logging:
                        print(response.decode('utf-8').replace("\r\n","\n"))
                
                # This little section is only needed to handle survivor buddy arduino code that
                # 1. expects a "1" at the start
                # 2. randomly, despite what the code says, will reset itself and expect a 1 again
                # as long as it has a \n, sending the 1 every time shouldn't have adverse affects (the arduino code ignores emtpy/incomplete lines)
                if include_legacy_survivor_buddy_support:
                    self.connection.write(b"1\n")
                    while self.connection.in_waiting:
                        response = self.connection.readline()
                        if logging:
                            print(response.decode('utf-8').replace("\r\n","\n"))
                
                if len(self.scheduled_actions) > 1:
                    positions, wait_time = self.scheduled_actions.pop(0)
                    torso_pitch, torso_yaw, head_roll, head_pitch = positions
                    torso_pitch  = f"{int(torso_pitch)}".rjust(3, "0")
                    torso_yaw    = f"{int(torso_yaw)}".rjust(3, "0")
                    head_roll    = f"{int(head_roll)}".rjust(3, "0")
                    head_pitch   = f"{int(head_pitch)}".rjust(3, "0")
                    self.connection.write(bytes(f"""{torso_pitch}{torso_yaw}{head_roll}{head_pitch}\n""", "utf-8"))
                    self.positions = positions
                    # without this sleep, even 1000 scheduled actions get executed more or less instantly
                    time.sleep(wait_time)
        
        self.thread = threading.Thread(target=thread_function)
        self.thread.start()
    
    def __del__(self):
        self.still_running = False
        self.thread.join()
    
    def set_joints(self, torso_pitch, torso_yaw, head_roll, head_pitch, speed=10):
        """
            torso_pitch: leaning forward/back, bigger = more forwards
            torso_yaw: left and right swivel, smaller = more to OUR left, survivor buddy's right
            head_roll: top of the head goes to the left, bottom of the head goes to the right, bigger = counterclockwise from MY persepctive 
            head_pitch: nodding head up down, bigger = down
            speed: 1 to 100
        """
        # NOTE: survivor buddy can actually move a bit faster than speed 1, but it very very very much risks damage to the parts from whiplash
        assert speed <= 100 and speed >= 0.1, "Speed of an action must be in the range 0.1 to 100" 
        assert torso_pitch >= SurvivorBuddySerial.torso_pitch_min and torso_pitch <= SurvivorBuddySerial.torso_pitch_max 
        assert torso_yaw   >= SurvivorBuddySerial.torso_yaw_min   and torso_yaw   <= SurvivorBuddySerial.torso_yaw_max   
        assert head_roll   >= SurvivorBuddySerial.head_roll_min  and head_roll  <= SurvivorBuddySerial.head_roll_max  
        assert head_pitch  >= SurvivorBuddySerial.head_pitch_min and head_pitch <= SurvivorBuddySerial.head_pitch_max
        
        positions = list(self.positions)
        self.scheduled_actions.clear() # interrupt any existing actions
        scheduled_actions = []
        
        diffs = [ (each1 - each2) for each1, each2 in zip((torso_pitch, torso_yaw, head_roll, head_pitch),positions)]
        torso_pitch_diff, torso_yaw_diff, head_roll_diff, head_pitch_diff = diffs
        new_torso_pitch, new_torso_yaw, new_head_roll, new_head_pitch = positions
        
        for _ in range(int(max(abs(each) for each in diffs))):
            # torso_pitch, torso_yaw, head_roll, head_pitch = self.positions = [ base+change for change, base in zip(diffs, self.positions) ]
            new_torso_pitch = _increment_joint_value(torso_pitch_diff, new_torso_pitch, torso_pitch)
            new_torso_yaw   = _increment_joint_value(torso_yaw_diff, new_torso_yaw, torso_yaw)
            new_head_roll   = _increment_joint_value(head_roll_diff, new_head_roll, head_roll)
            new_head_pitch  = _increment_joint_value(head_pitch_diff, new_head_pitch, head_pitch)
            
            scheduled_actions.append(
                (
                    (
                        new_torso_pitch, new_torso_yaw, new_head_roll, new_head_pitch
                    ),
                    0.002/(speed/100)
                    # ex: speed=100    => 0.002 wait time
                    # ex: speed= 50    => 0.004 wait time
                    # ex: speed= 0.1   => 2.000 wait time (2 full seconds, times the number of sub-steps; insanely slow)
                )
            )
        
        # add all of them at the end to reduce thread-locking overhead
        self.scheduled_actions += scheduled_actions

if __name__ == '__main__':
    print("testing survivor_buddy connection")
    survivor_bud = SurvivorBuddySerial(
        port_address=None,
        baud_rate=115200,
        windows_default_address="COM4",
        linux_default_address="/dev/ttyUSB0",
        mac_default_address=None, # mac auto-detects name
        inital_positions=[90,90,90,90],
        logging=False,
        include_legacy_survivor_buddy_support=False,
    )
    # 
    # cli loop 
    # 
    print(f'''Type q [Enter] to quit''')
    while 1:
        response = input("\nEnter joint positions. Units=degrees. (space or comma separated)\n")
        if response.lower() == 'q' or response.lower() == 'quit' or response.lower() == 'exit':
            break
        
        chunks = [""]
        for each in response:
            is_valid = each == "." or each == "-" or each.isdigit()
            if not is_valid and chunks[-1] != "":
                chunks.append("")
            if is_valid:
                chunks[-1]+=each
        
        if len(chunks[-1]) == 0:
            chunks.pop()
        
        try:
            new_joints = [ int(each)+90 for each in chunks ]
            survivor_bud.set_joints(*new_joints, speed=10)
        except Exception as error:
            print(error)
            print()
    
    survivor_bud.__del__()
    print("done")