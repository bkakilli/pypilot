## PyPilot
A python based highly modular auto-piloting firmware for onboard companion systems.

### Overview
PyPilot firmware is suitable for any user who wants to control their aerial vehicle either autonomously or manually. But it's power is that it allows users/developers to use their own aerial systems and their own control and guidance algorithms on a plug-and-play basis. The firmware consists of a core piloting backbone and various control, guidance, position estimation, and mission planning schemes. It allows users to define their own hardware configurations as well as their own control and guidance scheme so that they don't have to stick to a certain type of a vehicle, flight controller, control algorithm, or guidance scheme.

Additionally, PyPilot offers a beautiful Curses based CLI (command line interface) as ground control/monitoring station over a simple SSH connecton.

There are many already-defined vehicles and flight controllers, and many user simply can plug their onboard controller (a.k.a companion computer) to their vehicle and run an autonomous mission. PyPilot can also be used for manual flights, with your keyboard and the user interface provided. Our effort is to extend the compatibility of PyPilot across as many hardware/software as possible. Therefore you are more than welcome to integrate your own configuration and control/guidance model with PyPilot by pull requests.

### FAQ
**1. Why PyPilot instead of ROS?**

PyPilot is not a replacement for ROS. ROS is higly advanced and you should always consider it on systems that involves many sensors and complex configurations between them, and on very time-sensitive systems that requires very efficient control loop iterations. On the other hand PyPilot offers;

  * Simple plug-and-play structure
  * Good python environment
  * Many already defined ready-to-fly configurations
  * Simple ground control station

**2. Do I need to be connected to my vehicle all the time?**

Short answer is no. Long answer; if you have another way of monitoring it (by your eyes, by another ground station etc.) and if you have a way of taking the control back (meaning that if you are sure that after take-off it is not going to fly to its freedom unless you want it?), you don't need to be connected. But in most cases you may want to maintain a live connection with PyPilot running on your vehicle's companion computer.

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
