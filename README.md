
Install python environment, python-pip, dronekit. Then upgrade pymavlink.
~~~~
sudo apt-get install python-pip python-dev
sudo pip install dronekit
~~~~

dronekit installation will also install pymavlink. Check other pymavlink installations if any. 
If there is more than 1 instance of "mavutil.py", remove all the pymavlink directories recursively except the one here:

/usr/local/lib/python2.7/dist-packages/pymavlink

~~~~
sudo find / -name "mavutil.py"
sudo rm -rf path/to/pymavlink/to/be/removed/
~~~~

Upgrade pymavlink since dronekit installs not the latest one
~~~~
sudo pip install pymavlink --upgrade
~~~~

Clone this repository, cd into it and run the controller.
~~~~
git clone https://github.com/bkakilli/PixhawkControl
cd PixhawkControl
python drone.py
~~~~
