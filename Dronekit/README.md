# Dronekit tutorial

### This is all on Windows

## Installation

from the labelImg tutorial you should already have python3 and anaconda installed, if not you can get them here:
[python](https://www.python.org/downloads/), [anaconda](https://www.anaconda.com/download/#windows), [git](https://gitforwindows.org/).

I also recommend to use [PyCharm](https://www.jetbrains.com/pycharm/) as a text editor as it include support for anaconda.

you should also have [mission planner](http://ardupilot.org/planner/docs/mission-planner-installation.html) installed.

This is all for the requirements, now we will move on to the installation of the project itself.

If you didn't clone the repository yet, open anaconda prompt and run:

```
git clone https://github.com/ESTIASYSTEM/DUAV2019.git
```

then, go to the dronekit directory:

```
cd DUAV2019/Dronekit
```

and create a python2 conda environnement and activate it:

```
conda create -n duav python=2.7
conda activate duav
```

Now, install the required library:

```
conda install pil numpy
conda install --channel https://conda.anaconda.org/menpo opencv=2
pip install mavproxy dronekit dronekit-sitl
```

Finally you will need a [Win32 Python Extension for Accessing Video Devices.](http://videocapture.sourceforge.net)
Instructions on how to install are provided in the link above but here's is what you have to do:

First open a file explorer and locate the folder corresponding to the duav conda environnement that we created earlier:

`C:\Users\<yourUserName>\Anaconda3\envs\duav` or `Appdata/Local/Continuum/anaconda3/envs/duav`

there should be a folder named DLLs and another one named Lib.

Assuming that you are using python2.7 for x64 architecture we provided the necessary files in `DUAV2019/Dronekit/Python27.zip`.
Unzip it and copy the content in the duav folder mentionned above, it should merge the folders DLLs and Lib to the ones already existing.

This should be all you need. 

## Testing the code

We will now run several tests to check that everything was installed correctly and make sure we don't take too much risks with the drone.

### Testing the camera

We will start by testing that we can receive the video captured by the camera.

Open an anaconda prompt, go to the dronekit directory and activate your duav environnement:

```
cd DUAV2019/Dronekit
conda activate duav
```

the first test will check that you have everything you need installed.

```
python test_cam_connection.py
```

this should open a window named preview displaying the video captured by your integrated webcam. (if you're on a laptop)

press `esc` to close the window.

Now to test the connection with the drone's camera, make sure you have everything setup according to the tutorial to install and connect the camera to your computer (you should be able to open the video stream in vlc).

Open the file `test_cam_connection.py` and change `cam = Device()` - line6 - to `cam = Device(1)` then save and run again:

```
python test_cam_connection.py
```

### Testing Taking Off

We will now test that we are capable to connect dronekit to our drone and make it takeoff then wait for 30seconds and land.

First we will test that we are capable to send command to a simulated drone, this time you will need 3 anaconda prompt, 
in each of them go to the dronekit directory and activate your duav environnement:

```
cd DUAV2019/Dronekit
conda activate duav
```

In the first one start the drone simulation:

```
dronekit-sitl copter-3.3 --home=43.443632, -1.553146,42,0
```

In the second one open the connection between the drone and your computer:

```
python Scripts/mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
```

At this point you should open mission planner and check that it can connect to the drone. 
Once it is connected and you see the drone's parameters, you can send orders to the simulated
drone and check it's behavior in mission planner.

In the third prompt:

```
python test_arm_and_takeoff.py
```

