## PyPilot
A python based modular auto-piloting firmware for onboard companion systems.

### Overview
PyPilot firmware is suitable for any user who wants to control their aerial vehicle either autonomously or manually. It allows users/developers to use their own aerial systems and their own control and guidance algorithms on a plug-and-play basis. The firmware consists of a core piloting backbone and various control, guidance, position estimation, and mission planning schemes. It allows users to define their own hardware configurations as well as their own control and guidance scheme so that they don't have to stick to a certain type of a vehicle, flight controller, control algorithm, or guidance scheme. It is written in pure python, so anybody with basic Python knowledge can easily adapt PyPilot in their project.

PyPilot demo of a quadcopter with Pixhawk controller (at Guided-NoGPS mode) and Vicon position estimation system:

[![PyPilot on running.](https://img.youtube.com/vi/8WUousk9y-Y/0.jpg)](https://www.youtube.com/watch?v=8WUousk9y-Y)

### Installation (Debian based)
Install python environment and python-pip. (Advanced users: You can use virtualenv if you like)
~~~~
sudo apt-get install python-dev python-pip
~~~~

PyPilot does not depend on any 3rd party APIs or libraries. However, for Pixhawk connection the built-in modules uses dronekit and pymavlink. Therefore, those must be installed if you want to use the built-in modules for Pixhawk.

Install dronekit and curses. Dronekit installation will also install pymavlink but you should upgrade pymavlink since dronekit may not install the latest one.
~~~~
sudo pip install dronekit
sudo apt-get install libncurses5-dev
sudo pip install pymavlink --upgrade
~~~~

If there is error upgrading pymavlink, find and remove all pymavlink installations and install pymavlink again.

~~~~
sudo find / -name "mavutil.py"
sudo rm -rf path/to/pymavlink/to/be/removed/
sudo rm -rf path/to/pymavlink/to/be/removed/
...
sudo rm -rf path/to/pymavlink/to/be/removed/
sudo pip install pymavlink --upgrade
~~~~

Clone this repository, cd into it.
~~~~
git clone https://github.com/bkakilli/pypilot
cd pypilot
~~~~

If you are using Pixhawk, make sure you have the right serial connection between your compainion computer (Raspberry Pi may be?) and Pixhawk.
(Telem2 port should be used to connect Pixhawk from Raspberry Pi)

Modify config.py based on your application and hardware.

Run pypilot.py
~~~~
python pypilot.py
~~~~

More documentation will be provided.
