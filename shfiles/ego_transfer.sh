#!/bin/bash
 
 # --- for every Terminal-tab

gnome-terminal --tab "0" -- bash -c "python3 ego_transfer.py typhoon_h480 0;exec bash"
gnome-terminal --tab "1" -- bash -c "python3 ego_transfer.py typhoon_h480 1;exec bash"
gnome-terminal --tab "2" -- bash -c "python3 ego_transfer.py typhoon_h480 2;;exec bash"

 
