# sr_speech_control

A node for controlling various systems using speech.
Commands are being sent to a topic `sr_speech_control` in a form of std_msgs
String and can be intercepted by other nodes in order to execute actions.

## Usage

To start the node run:
```
rosrun sr_speech_control speech_control.py
```

The node will permanenly listen to the microphone input, will use Google speech
recognition to translate audio to text, check if text starts with a trigger
word `shadow` and if so will publish text after the trigger word to topic
`sr_speech_control`.

## Known problems

Testing revealed that microphone devices available within Docker container are
significantly worse than on the host machine. That can be easily demonstrated
by testing Google Chrome browser speech recognition on the host and inside
Docker container. Known workaround for that is to use pulseaudio server on host
machine for sound capture. For that install `paprefs` program on the host:
```
sudo apt-get install paprefs
```
and run it. In "Network Server" tab, and check the "Enable network access to
local sound devices" checkbox and other two sub-checkboxes in order not to
require authentications. You might need to reboot host machine for this setting
to be used.
Run `pax11publish` utility program to find out pulseaudio server port (most
likely 4713).
On the container run:
```
export "PULSE_SERVER=tcp:<host IP address>:<host pulseaudio port>"
```
For example if your host IP address is `192.168.1.2`, then run:
```
export "PULSE_SERVER=tcp:192.168.1.2:4713"
```
Alternatively it is possible to map unix sockets instead of tcp but it requires
adding new parameters when launching Docker container.

To use node with pulseaudio microphone specify `prefer_microphone` parameter:
```
rosrun sr_speech_control speech_control.py _prefer_microphone:=pulse
```
