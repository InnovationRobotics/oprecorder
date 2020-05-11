# oprecorder - Operator Recorder
This software package includes several recorders:
- oprecorder.py that records topics related to the bobcat simulation - Each time the callback of the action (joystick) is called, it changes the world state accordingly and saves the:
Date&Time, world_state, grade (N/A) by adding a row in the csv file: oprecorder.csv
- rcrecorder.py that records topics related to the real car and the algoryx simulation - Each time the callback of the action (rcoverride) is called, it changes the world state accordingly and saves the:
Date&Time, world_state, grade (N/A) by adding a row in the csv file. All the pointclouds are saved. A bagfile is saved as well. It is stored in $HOME/recordings.
- periodicrecorder.py that records every period of time. The default period is 100ms



