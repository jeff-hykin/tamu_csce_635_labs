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
            print(f'''faker: sending no noise''')
            fake_noise_helper.just_sent_clap = False
        elif mode == 1:
            print(f'''faker: continuous noise''')
            fake_noise_helper.just_sent_clap = False
            output = numpy.ones(audio_chunk_size)*noise_volume
        elif mode == 2:
            if fake_noise_helper.just_sent_clap:
                if fake_noise_helper.check_timer():
                    print(f'''faker: after clap: interrupt ''')
                    output = numpy.ones(audio_chunk_size)*noise_volume
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
    return output

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

from helper_scripts.physical_serial_test import SurvivorBuddySerial

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
        head_pitch=joint_positions.head_nod+90,   # on hardware: bigger= down
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