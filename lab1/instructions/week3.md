# Multi behavior

## Due when?

In person by Fri March 1st

## What needs to be done

- Run the `run/check` command
- Be ready to demonstrate any of the behaviors on on rviz
- Alternatively, submit a screen recording that
    - shows the code structure (e.g. parameters/schema)
    - demo **all** of the behaviors in Rviz

## Tasks

- NOTE: its okay to NOT follow schemas. These tasks kind of show how the nice schema approach isn't always completely practical

- create two more actions (also involving >=2 degrees of freedom):
    1. fearful
        - involve the face expressions
    2. startled
        - no facial expression required
        - have an intensity parameter for the rviz movement, higher intensity causes a stronger reaction
   
- follow the behavioral schema again with the following characteristics
    - Have survivor buddy return to its original pose after time time of no sound stimulus
    - Have survivor buddy "fall asleep" after lots of time with no stimulus
    - Continuous loud noise at any time causes fearful
    - Clapping when awake triggers curious
    - Clapping when asleep triggers startled (louder the clap, the greater the reaction)
