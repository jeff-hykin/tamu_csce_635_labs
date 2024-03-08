import sys
import os
from os.path import isabs, isfile, isdir, join, dirname, basename, exists, splitext
from os import remove, getcwd, makedirs, listdir, rename, rmdir
from shutil import move
import glob
import random
import math
import time

try:
    import pretty_errors
    pretty_errors.configure(
        separator_character = '*',
        filename_display    = pretty_errors.FILENAME_EXTENDED,
        line_number_first   = True,
        display_link        = True,
        lines_before        = 5,
        lines_after         = 2,
        line_color          = pretty_errors.RED + '> ' + pretty_errors.default_config.line_color,
        code_color          = '  ' + pretty_errors.default_config.line_color,
        truncate_code       = True,
        display_locals      = True
    )
except Exception as error:
    pass


logging = False

# 
# basics
# 
if True:
    def clip_value(value, minimum, maximum):
        if value == None:
            return None
        if value < minimum:
            value = minimum
        if value > maximum:
            value = maximum
        return value
    
    prev_time = time.time()
    def time_since_prev():
        """
        Example:
            time_since_prev()
            sleep(0.5) # seconds
            print(time_since_prev()) # prints 0.5 (approximately)
        """
        global prev_time
        now = time.time()
        output = now - prev_time
        prev_time = now
        return output

import time
class Timer:
    """
    Note: this is hacky, just for faking noise, probably dont use it
    """
    def __init__(self):
        self.start_time = time.time()
        self.prev_time = self.start_time
        self.timer_start = None
    
    @property
    def time_since_start(self):
        return time.time() - self.start_time
    
    @property
    def time_since_prev(self):
        now = time.time()
        output = now - self.prev_time
        self.prev_time = now
        return output
    
    def start_timer(self, amount):
        self.timer_start = time.time()
        self.timer_amount = amount
    
    def check_timer(self):
        if self.timer_start == None:
            return None
        now = time.time()
        if now - self.timer_start > self.timer_amount:
            return True
        else:
            return False


fake_noise_helper = Timer()
fake_noise_helper.started = False
fake_noise_helper.waiting_duration = 0
fake_noise_helper.action_duration = 0
fake_noise_helper.just_sent_clap = False
fake_noise_helper.prev_mode = None
audio_chunk_size = 2048
def generate_fake_continous_noise(delay=5, duration=4, noise_volume=1, spike_index=124, should_clap=False, interrupt_after_clap_lag=0.5):
    """
        generate_fake_continous_noise(
            delay=5, # means startup delay of 5 seconds
            duration=5 # means 5sec of silence followed by 5 seconds of continuous noise
            noise_volume=1, # means the values in the array will be 1
        )
    """
    import numpy
    output = numpy.zeros(audio_chunk_size)
    assert interrupt_after_clap_lag < duration/2, "The interrupt needs to be within the duration of the clap"
    mode = None
    if not fake_noise_helper.time_since_start > delay:
        print(f'''faker: waiting on start delay''')
    else:
        fake_noise_helper.started = True
        if should_clap:
            mode = int((int(time.time()) / duration) % 3) # 3=noise,silence,clap
        else:
            mode = int((int(time.time()) / duration) % 2) # 2=noise,silence
        
        if mode == 0:
            print(f'''faker: continuous noise''')
            fake_noise_helper.just_sent_clap = False
        elif mode == 1:
            print(f'''faker: sending no noise''')
            fake_noise_helper.just_sent_clap = False
            output = numpy.ones(audio_chunk_size)
        elif mode == 2:
            if fake_noise_helper.just_sent_clap:
                if fake_noise_helper.check_timer():
                    print(f'''faker: after clap: interrupt ''')
                    output = numpy.ones(audio_chunk_size)
                else:
                    print(f'''faker: after clap: lag time''')
            elif fake_noise_helper.just_sent_clap == None:
                print(f'''faker: after interrupt ()''')
            else:
                print(f'''faker: sending clap''')
                fake_noise_helper.just_sent_clap = True
                fake_noise_helper.start_timer(interrupt_after_clap_lag)
                
                output = numpy.zeros(audio_chunk_size)
                output[spike_index] = noise_volume # one giant spike
    
    fake_noise_helper.prev_mode = None
    return numpy.zeros(audio_chunk_size)

# 
# joints
# 
class JointPositions:
    """
        Note:
            Everything is in degrees
        
        Example:
            JointPositions(
                torso_joint=5, # larger = more forwards
                neck_swivel=5, # more negative means more to your left side (the survivor buddy's right side)
                head_tilt=5, # 
                head_nod=5, # more negative = face the cieling
            )
    """
    torso_joint_max = 40;torso_joint_min = -40 
    neck_swivel_max = 40;neck_swivel_min = -40 
    head_tilt_max   = 40;head_tilt_min   = -40
    head_nod_max    = 40;head_nod_min    = -40
    
    def __init__(self, list_input=None, *, torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None,):
        if isinstance(list_input, type(None)):
            self.torso_joint = torso_joint
            self.neck_swivel = neck_swivel
            self.head_tilt   = head_tilt
            self.head_nod    = head_nod
        elif isinstance(list_input, (list,tuple)):
            if len(list_input) < 4:
                raise Exception(f"""\n\nWhen creating a JointPositions() object, a list was given, but it didn't contain 4 elements. It needs to contain 4 elements (torso_joint,neck_swivel,head_tilt,head_nod)\nInstead it contained: {list_input}""")
            self.torso_joint = list_input[0]
            self.neck_swivel = list_input[1]
            self.head_tilt   = list_input[2]
            self.head_nod    = list_input[3]
        elif isinstance(list_input, JointPositions):
            self.torso_joint = list_input.torso_joint
            self.neck_swivel = list_input.neck_swivel
            self.head_tilt   = list_input.head_tilt
            self.head_nod    = list_input.head_nod
        else:
            raise Exception(f"""\n\nWhen creating a JointPositions() object, an unknown type was given: {list_input}""")
        
        # ensure within acceptable ranges   
        self.clip_values()
    
    def clip_values(self):
        self.torso_joint  = clip_value(self.torso_joint, minimum=self.torso_joint_min, maximum=self.torso_joint_max)
        self.neck_swivel  = clip_value(self.neck_swivel, minimum=self.neck_swivel_min, maximum=self.neck_swivel_max)
        self.head_tilt    = clip_value(self.head_tilt  , minimum=self.head_tilt_min  , maximum=self.head_tilt_max  )
        self.head_nod     = clip_value(self.head_nod   , minimum=self.head_nod_min   , maximum=self.head_nod_max   )
        return self
    
    @property
    def as_list(self):
        self.clip_values()
        return [
            self.torso_joint,
            self.neck_swivel,
            self.head_tilt,
            self.head_nod,
        ]
    
    def __repr__(self):
        return f"[ torso_joint={self.torso_joint:.0f}°, neck_swivel={self.neck_swivel:.0f}°, head_tilt={self.head_tilt:.0f}°, head_nod={self.head_nod:.0f}°,  ]"


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
            neck_pitch=90, bigger = more forwards
            neck_yaw=90, smaller = OUR left, survivor buddy's right
            head_roll=90, bigger = counterclockwise from OUR persepctive 
            head_pitch=90, bigger= down
            speed=40, # out of 100
        )
    """
    neck_pitch_min = 35;neck_pitch_max = 150; # bigger = more forwards
    neck_yaw_min   = 20;neck_yaw_max   = 160; # smaller = OUR left, survivor buddy's right
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
        self.still_running = False
        def thread_function():
            while self.still_running:
                while self.connection.in_waiting:
                    response = self.connection.readline()
                    if logging:
                        print(response.decode('utf-8'))
                
                # This little section is only needed to handle survivor buddy arduino code that
                # 1. expects a "1" at the start
                # 2. randomly, despite what the code says, will reset itself and expect a 1 again
                # as long as it has a \n, sending the 1 every time shouldn't have adverse affects (the arduino code ignores emtpy/incomplete lines)
                if include_legacy_survivor_buddy_support:
                    self.connection.write(b"1\n")
                    while self.connection.in_waiting:
                        response = self.connection.readline()
                        if logging:
                            print(response.decode('utf-8'))
                
                if len(self.scheduled_actions) > 1:
                    action, positions = self.scheduled_actions.pop(0)
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
    
    def set_joints(self, torso_pitch, torso_yaw, head_roll, head_pitch, speed=40):
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
                    new_torso_pitch, new_torso_yaw, new_head_roll, new_head_pitch
                ),
                0.002/(speed/100)
                # ex: speed=100    => 0.002 wait time
                # ex: speed= 50    => 0.004 wait time
                # ex: speed= 0.1   => 2.000 wait time (2 full seconds, times the number of sub-steps; insanely slow)
            )
        
        # add all of them at the end to reduce thread-locking overhead
        self.scheduled_actions += scheduled_actions

survivor_bud = None
def send_to_survivor_bud(joint_positions, speed=40):
    global survivor_bud
    survivor_bud = survivor_bud or SurvivorBuddySerial(
        port_address=None,
        baud_rate=115200,
        windows_default_address="COM4",
        linux_default_address="/dev/ttyUSB0",
        mac_default_address=None, # mac auto-detects name
        inital_positions=[90,90,90,90],
        logging=False,
        include_legacy_survivor_buddy_support=True,
    )
    
    survivor_bud.set_joints(
        torso_pitch=joint_positions.torso_joint+90, # on hardware: larger = more forwards
        torso_yaw=joint_positions.neck_swivel+90,   # on hardware: smaller = OUR left, survivor buddy's right
        head_roll=joint_positions.head_tilt+90,     # on hardware: bigger = counterclockwise from OUR persepctive 
        head_pitch=joint_positions.head_pitch+90,   # on hardware: bigger= down
        speed=speed,
    )

# 
# cli tool
# 
def convert_args(raw_args):
    """
    Example:
        convert_args(["arg1","-f", "--something", "10"])
        # returns:
        ["arg1"], { "f":True, "something": 10 }
    
    Summary:
        takes sys.argv as an argument
        for example something like `python main.py arg1 -f --something 10`
        and turns it into ["arg1"], { "f":True, "something": 10 }
        
        Single dash means a boolean flag
        Double dash means its a key-value pair
    
    Details:
        Returns a list and a dictionary with positional and keyword arguments.

        -This function assumes flags must start with a "-" and and cannot be a 
            number (but can include them).
        
        -Flags should either be followed by the value they want to be associated 
            with (i.e. -p 5) or will be assigned a value of True in the dictionary.

    """
    from collections import defaultdict
    def str_is_float(string):
        try:
            float(string)
            return True
        except ValueError:
            return False
    
    def str_is_int(string):
        return string.lstrip('+-').isdigit()
    
    
    args = []
    kwargs = defaultdict(lambda *args: None)
    count = 0
    
    raw_args = list(raw_args)
    while len(raw_args) > 0:
        next_raw_argument = raw_args.pop(0)
        # If the current argument starts with "-", then it's a key
        if next_raw_argument.startswith('--'):
            if len(raw_args) == 0:
                raise Exception(f'''
                    
                    This argument: {next_raw_argument}
                    expects a value after it (-key value), however it was the last argument
                    Maybe you meant: -{next_raw_argument}
                    (which is just a flag, e.g. no value)
                    
                ''')
            
            next_arg_is_possibly_negative_number = False
            try:
                float(raw_args[0])
                next_arg_is_possibly_negative_number = True
            except Exception as error: pass
            
            if raw_args[0].startswith('-') and not next_arg_is_possibly_negative_number:
                raise Exception(f'''
                    
                    This argument: {next_raw_argument}
                    expects a value after it (-key value)
                    However it was follow by another key/flag (--key {raw_args[0]})
                    Maybe this would be valid: -{next_raw_argument} {raw_args[0]}
                    (which is just a flag, e.g. no value)
                    
                ''')
            # consume the next element as the value
            kwargs[next_raw_argument] = raw_args.pop(0)
        # its a flag
        elif next_raw_argument.startswith("-") and not str_is_float(next_raw_argument):
            kwargs[next_raw_argument] = True
        # Else, it's a positional argument without flags
        else:
            args.append(next_raw_argument)
    
    # 
    # convert number arguments to be numbers
    # 
    for each_index, each_value in enumerate(list(args)):
        if str_is_int(each_value):
            args[each_index] = int(each_value)
        if str_is_float(each_value):
            args[each_index] = float(each_value)
        if each_value.lower() == "false":
            args[each_index] = False
        if each_value.lower() == "true":
            args[each_index] = True
        if each_value.lower() == "none":
            args[each_index] = None
    for each_key, each_value in kwargs.items():
        if isinstance(each_value, str):
            if str_is_int(each_value):
                kwargs[each_key] = int(each_value)
            if str_is_float(each_value):
                kwargs[each_key] = float(each_value)
            if each_value.lower() == "false":
                kwargs[each_key] = False
            if each_value.lower() == "true":
                kwargs[each_key] = True
            if each_value.lower() == "none":
                kwargs[each_key] = None
    
    # 
    # make the -name vs -name irrelevent 
    # 
    for each_key, each_value in list(kwargs.items()):
        if isinstance(each_key, str):
            if each_key.startswith("--"):
                del kwargs[each_key]
                kwargs[each_key[2:]] = each_value
            elif each_key.startswith("-"):
                del kwargs[each_key]
                kwargs[each_key[1:]] = each_value
            else:
                kwargs[each_key] = each_value
    
    return args, kwargs