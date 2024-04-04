# When is this Due?

- Part1: Wed, April 10th, in class 
- Part2: Friday, April 12th, in class

# What do you need to do?

- part 1: hand-write (not typed/printed, not digital, old school hand written) the setup steps for spot *twice*, and turn in those peices of paper 
    - setup instructions are at the bottom of this post
- part 2: videos, code, and questions
    - ***handwrite*** everything except the code! Turn it in on Friday
    - what are the three "quirks" mentioned in the videos
    - answer these conceptual questions
        - when running at full speed, do you think spot's obstacle avoidance will stop it from hitting the wall? (We'll find out Friday!)
        - in the `hello_spot.py`, go to the "Robot standing twisted." part. What do think would happen (short timeline) if we duplicated the `command_client.robot_command` with `yaw=0` and `pitch=0.4`? (We'll also find out Friday!)
        - do you think spot uses one of the three systems architectures? which (if any) and why?
        - do you think the spot controller has some form of innate releasing mechanisms? why or why not?
        - do you think the spot controller has some level of behavior coordination?
    - answer these "correct answer" questions
        - whats the difference between a blocking and non-blocking function?
        - what was wrong/missing from the "setting up spot" video???
        - what sensors does spot have?
        - why do you think there is there a `time.sleep(3)` in the code? (code file is linked below)
    - modify code (without running it)
        - download this `hello_spot.py` file
        - there is a section that mentions "absolute body control"
        - within that section there are time durations for each movement
        - double the amount of time between those movements!
        - upload the modified code to canvas
    


## Spot Setup Instructions

Stage 0
1. Set a timer for however long you plan to use spot, and when the timer goes off make sure you fully/entirelly pull the battery out of spot!
2. Seriously its ~$8,000 mistake if you forget to remove the battery
3. Get spot out of the case, holding by the legs
4. Take a picture of the battery slot
5. Boot up the tablet
6. Put the battery in spot
7. Grab handle-straps on shoulders and flip spot
8. Power button

Stage 1: tablet controls
1. Open the spot app, and connect to spot
2. Login with the info from the back of the picture
3. Enable the safety mechanisms

Stage 2: computer controls
1. Make sure you actually set a timer for physically pulling out the battery!!
2. Install the dependencies
- python
- pip install bosdyn-client bossing-mission bosdyn-choreography-client
- git clone https://github.com/boston-dynamics/spot-sdk.git
- cd ./spot-sdk/python/examples/hello_spot
- pip install -r requirements.txt
3. Put in the username and password from the picture
- export BOSDYN_CLIENT_USERNAME=
- export BOSDYN_CLIENT_PASSWORD=
4. Disconnect the tablet from spot
5. Connect your computer to the spot wifi
6. Run hello_spot.py
