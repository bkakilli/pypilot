
Install python environment and python-pip
> sudo apt-get install python-pip python-dev

Install dronekit
> sudo pip install dronekit

dronekit installation will also install pymavlink

Check other pymavlink installations if any
> sudo find / -name "mavutil.py"

If there is more than 1 instance of "mavutil.py", remove all the pymavlink directories recursively except the one here:

/usr/local/lib/python2.7/dist-packages/pymavlink
> sudo rm -rf path/to/pymavlink/to/be/removed/

Upgrade pymavlink since dronekit installs not the latest one
> sudo pip install pymavlink --upgrade

Navigate to /usr/local/lib/python2.7/dist-packages/pymavlink

Rename mavutil.pyc to mavutil.pyc.bak 
> cd /usr/local/lib/python2.7/dist-packages/pymavlink
> sudo mv mavutil.pyc mavutil.pyc.bak

Open mavutil.py file with a text editor, go to line 1537 (With nano, press Alt+G then enter 1537). Find the interpret_px4_mode function definition. In If-statement of the function, find the line:
> elif custom_sub_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
change it to
> elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
Save it

Return to your home and create a project directory
> cd ~ && mkdir pixhawk_controller

Copy the drone.py file here and run it.
> python drone.py
