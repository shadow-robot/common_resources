# sr_speech_control

A node for controlling various systems using speech.
Commands are being sent to a chosen topic in a form of std_msgs
String and can be intercepted by other nodes in order to execute actions.

## Usage

To start the node run:
```
rosrun sr_speech_control speech_control.py
```

The node will permanenly listen to the microphone input, will use Google speech
recognition to translate audio to text, check if text starts with a chosen trigger word and if so, will publish word after the trigger word to a `sr_speech_control` topic (by default) .

The class used has five parameters that can be provided in order to modify the behaviour:
- `trigger_word` - sets a word that preceeds a command to be sent
- `command_words_and_feedback` - dictionary of commands that are allowed to be sent and corresponding sound feedback words
- `similar_words_dict_path` - path to a yaml file containing dictionary of words that are easily mistaken for a trigger word or one of the command words
- `non_speaking_duration` - seconds of non-speaking audio to keep on both sides of the recording
- `pause_threshold` - seconds of non-speaking audio before a phrase is considered complete
- `output_device_name` - the device name on which the voice feedback confirmation should be played. Most probably this will be the devices brand or model (to verify use `lsusb`).

An example usage can be seen in the `speech_control.py` file:
```python
    trigger_word = "shadow"
    command_words_and_feedback = {"grasp": "grasped", "release": "released", "disable": "disabled",                   
                                  "enable": "enabled", "engage": "engaged", "open": "opened"}
    similar_words_dict_path = rospkg.RosPack().get_path('sr_speech_control') + '/config/similar_words_dict.yaml'

    sc = SpeechControl(trigger_word, command_words_and_feedback, similar_words_dict_path=similar_words_dict_path,
                       output_device_name="Logitech")
    sc.run()

```
