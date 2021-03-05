# sr_speech_control

Package for controlling various systems using speech. Commands are being sent to a topic (default: `speech_control`) in a form of std_msgs String and can be intercepted by other nodes in order to execute actions.

# Usage

Create a `SpeechControl` class object and run it using the `run()` method. Depending on the trigger word that you have chosed, commands (from the commands list specified in the class constructor) proceeded by that word will be executed, and all other words and phrases will be ignored. E.g., if your trigger word is `Shadow` and you say `execute grasp`, no messages will be send to the topic. On the other hand, if you say `shadow grasp`, a a `grasp` message will be sent.

Example class implementation:

```python
    trigger_word = "shadow"
    command_words = ["grasp", "release", "disable", "enable"]
    sc = SpeechControl(trigger_word, command_words)
    sc.run()
```