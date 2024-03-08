# Face detection

## Due when?

In person demo Fri March 8th

## What needs to be done

- Demonstrate using a face API
- Be ready to demonstrate behaviors on rviz
- Get code running on the physical bot
- Run the `run/check` command

## Tasks

- Be able to use a face API
    - You can use any API, but `insightface` should have been installed as part of the setup instructions
    - I recommend testing with an image file first (before trying to use the images from `when_video_chunk_received`)
    - insightface doesn't have great documentation, so try searching github (all of github) for `from insightface.app import FaceAnalysis` to see examples
    - Show how to get the number of faces and the x,y coordinates of a face

- Make a talking sound
    - in the `main.js` file, similar to how you can make face/eye expressions, you can also use `playSound("Howdy!", {pitch:1,}).then(()=>console.log("finished making a sound"))`
    
- Create a "look at" behavior
    - If theres one or more faces visible, pick one and have survivor buddy look towards it
    - e.g. if the face moves to one side, survivor buddy should turn its head to keep looking at it
    - if the face moves up/down survivor buddy should angle its head to keep looking at it

- Create the following behaviors:
    - **Note1**: these are in order of precedence/priority. If two of them use the motors, then the one higher on the list should interrupt/override the one that is later on the list.
    - **Note2**: if one behavior uses the face, and another uses the motors, don't require that one interrupts the other.
    - **Note3**: either remove old (week3) behaviors or make them "play nice" with the behaviors below.
    - Loud noises (spike or continuous) cause the fearful behavior
        - e.g. interrupts any other rviz motion
    - No stimulus for some amount of time causes survivor buddy to go into a neutral state
    - Being in the neutral state, seeing a face, but the face being too close should cause the startled behavior
    - Seeing a face in a neutral state should trigger the happy face expression
    - After the initial response (startled or happy), if one (or more) faces are still detected, have survivor buddy do the "look at" behavior
    - If a face gets too close, survivor buddy should lean back
        - Its fine to use hacky ways of measuring "too close"
    - If a face was in the middle of survivor buddy's view, then disappears, it should trigger the curious behavior
    - If there is a moderate level of noise for some period of time, survivor buddy should start talking (and should stop when/if the noise dies down)
        - There is a helper function for testing this inside of project_tools.py (do `git pull;run/pull_changes`)
        - in `main.py` find the `from project_tools` then below it add `from project_tools import generate_fake_continous_noise` 
        - then make this change:  
        ```py
        def when_audio_chunk_received(chunk): # <- find this 
            data = numpy.array(chunk.data)
            # add this line:
            data = generate_fake_continous_noise(delay=5, duration=4, noise_volume=0.4)
            # it will now alternate every 4 seconds between silence and noise that has a constant volume of 0.4
        ```

- Deploy behaviors to the physical robot
    1. Plug in survivor_buddy's USB cable into your laptop
    2. Make sure it is being detected by running the command `ls /dev/ttyUSB*`, you should see /dev/ttyUSB0 as the output of the previous command.
    3. To test if the connection works run `run/test_serial`
    4. Now repeat the steps of running on rviz (4 terminals) BUT, instead of `run/4_python` do `run/4_python --send_to_rviz False`