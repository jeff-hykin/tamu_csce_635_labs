# When is this Due?

- Friday, April 12th, in class

# What do you need to do?

Four things:

1. Handwrite the setup steps (15min)
2. Handwrite (not typed!) answers to video questions
3. Submit a python file to canvas
4. Show up on Friday


Details:

1. Here's the Spot setup steps:
    - Stage 0
      1. Set a timer for however long you plan to use spot. When the timer goes off fully/entirelly pull the battery out of spot!
      2. Seriously its ~$8,000 mistake if you forget to remove the battery
      3. Get spot out of the case, holding by the legs
      4. Take a picture of the battery slot
      5. Boot up the tablet
      6. Put the battery in spot
      7. Grab handle-straps on shoulders and flip spot
      8. Power button

    - Stage 1: tablet controls
        1. Open the spot app, and connect to spot
        2. Login with the info from the picture you took
        3. Enable the safety mechanisms

    - Stage 2: computer controls
        1. Make sure you actually set a timer for physically pulling out the battery!!
        2. Install the dependencies
            - python
            - `pip install bosdyn-client bossing-mission bosdyn-choreography-client`
            - `git clone https://github.com/boston-dynamics/spot-sdk.git`
            - `cd ./spot-sdk/python/examples/hello_spot`
            - `pip install -r requirements.txt`
            - `cd ../../../..`
        3. Put in the username and password from the picture you took
            - `export BOSDYN_CLIENT_USERNAME=`
            - `export BOSDYN_CLIENT_PASSWORD=`
        4. Disconnect the tablet from spot
        5. Connect your computer to the spot wifi
        6. Activate the emergency stop connection
            - `cd ./spot-sdk/python/examples/estop/ && python hello_estop.py 192.168.80.3`
        7. Run the hello spot code
            - `cd ./spot-sdk/python/examples/hello_spot/ && python hello_spot.py 192.168.80.3`
    

2. Here's [a link to the videos](https://drive.google.com/drive/folders/17rnci2pySNYl2OrrDaLOxTbo7YW6nqRJ?usp=drive_link), and the questions:
    - Note: there should be 4 videos, the last one is getting uploaded tonight (Tue)
    -  (***handwrite*** everything!) here's the  [Questions PDF](https://raw.githubusercontent.com/jeff-hykin/tamu_csce_635_labs/master/lab2/spot_questions.pdf)
      
3. Here's the code task
    - Download this [`hello_spot.py`](https://github.com/boston-dynamics/spot-sdk/blob/master/python/examples/hello_spot/hello_spot.py) file
    - Make the following two modifications (without running it)
    - Change #1: go to the "Robot standing twisted." part
        - Find the `command_client.robot_command(` part 
        - 1. Duplicate the whole function call, and edit the one further down
        - 2. Inside of it change `yaw` to be 0 and `pitch` to be 0.4
    - Change #2: go to the section that mentions "absolute body control"
        - change the `z` value of the second `math_helpers.Quat(` object to `0.4`
        - within that section there are time durations for each movement
        - double the amount of time between those movements
    - Upload the modified code to canvas