
Install python environment, python-pip, dronekit. Then upgrade pymavlink.
(Advanced users: You can use virtualenv if you like)
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

Clone this repository, cd into it in Raspberry Pi.
~~~~
git clone https://github.com/bkakilli/PixhawkControl
cd PixhawkControl
~~~~

Make sure you have the right serial connection between Raspberri Pi and Pixhawk.
(Telem2 port should be used to connect Pixhawk from Raspberry Pi)

Modify config.py based on your application and hardware.

Run controller.py
~~~~
python controller.py
~~~~

More documentation will be provided.
