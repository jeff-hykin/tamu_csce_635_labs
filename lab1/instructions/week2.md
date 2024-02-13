# Following a schema

## Due when?

In person by Fri Feb 16, 1:50pm (or earlier)

## What needs to be done

- Run the `run/check` command (Select week 2, and do all the checkbox items)
- Be ready to explain code / code structure (parameters / schema's)
- Be ready to demonstrate any of the behaviors (below) on rviz
- Alternatively, submit a screen recording that
    - shows the code structure
    - demo **all** of the behaviors in Rviz


## Tasks

- Create clap detection
    - NOTE: this is different than just a loudness-threshold, it needs to only trigger on a spike
    - I recommend playing recorded audio into the mic to get consistent results
    - There should be some kind of sensitivity parameter
    - Even if your code is decent, its probably to be finnicky. You just need a ~60% success rate
- Create a code structure(s) that follows the behavioral schema
    - E.g. behavioral schema class that takes parameters as an input (ex: constructor arguments)
         - If you're not familiar with object oriented programming, watch programming (not conceptual) YouTube videos about it
    - Create a perception schema that includes clapping
    - Create a motor schema for the curious action
- Create the following behavior:
    - Clapping should trigger the curious action
    - Have survivor buddy return to its original pose after `N` seconds in a naturalistic way (e.g. slowly)


### Lab Late Policy
- entirely late (more than 1 week) =  zero
- partly late = additional work
   - To get the first 50% of credit, you will need to record/upload/submit a video (instead of demoing in person)
   - To get the second 50% of assignment credit (for a full 100%)
     - help other students during study hall (+30min of help for each day late)
     - demo the next assignment by Thursday morning (e.g. one day early)
     - If you're busy during study hall, we can schedule another time to help
