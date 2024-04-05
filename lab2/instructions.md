# When is this Due?

- Part A: Wed, April 10th, in class 
- Part B: Friday, April 12th, in class

# What do you need to do?

- part A: hand-write (not typed & printed) the numbered setup steps for spot (if its a bullet point, rather than a number, you can skip it)
    - NOTE! be ready to be quized on them on Friday; because you will be asked to do some of these steps.
- part B: videos, code, and questions
    - ***handwrite*** everything except the code! Turn it in on Friday
    - 1. Watch these videos, with the following questions in mind:
        - "Correct answer" questions:
            - What are the three "quirks" mentioned in the videos?
            - whats the difference between a blocking and non-blocking function?
            - what was wrong/missing from the "setting up spot" video???
            - what sensors does spot have?
        - Conceptual questions:
            - when running at full speed, do you think spot's obstacle avoidance will stop it from hitting the wall? (We'll find out Friday!)
            - what gaits (book terminology) did you see and not see? why do you think the designers made those decision to include/exclude them?
            - do you think spot uses a central pattern generator? Why or why not?
            - do you think the spot controller has some form of innate releasing mechanisms? Make sure to mention obstacle avoidance and stair traversal when justifying your answer.
            - do you think the spot controller has some level of behavior coordination? why or why not?
            - do you think spot uses one of the three systems architectures? which (if any) and why?
        - Code-change questions:
            - why do you think there is there a `time.sleep(3)` in the code? (code file is linked below)
            - when you modified the code (instructions below)
                - what do you think change #1 would do?
                - what do you think change #2 would do?
    - 2. Modify code (without running it)
        - download this `hello_spot.py` file
        - change #1: go to the "Robot standing twisted." part
            - Find the `command_client.robot_command(` part 
            - 1. Duplicate the whole function call, and edit the one further down
            - 2. Inside of it change `yaw` to be 0 and `pitch` to be 0.4
        - change #2: go to the section that mentions "absolute body control"
            - change the `z` value of the second `math_helpers.Quat(` object to `0.4`
            - within that section there are time durations for each movement
            - double the amount of time between those movements
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
- cd ../../../..
3. Put in the username and password from the picture you took
- export BOSDYN_CLIENT_USERNAME=
- export BOSDYN_CLIENT_PASSWORD=
4. Disconnect the tablet from spot
5. Connect your computer to the spot wifi
6. Activate the emergency stop connection
- `cd ./spot-sdk/python/examples/estop/ && python hello_estop.py 192.168.80.3`
7. Run the hello spot code
- `cd ./spot-sdk/python/examples/hello_spot/ && python hello_spot.py 192.168.80.3`
 
