import io
import os
from gtts import gTTS
from playsound import playsound

def confirm_voice_command(command = "No command"):
    language = 'en'
    command_formatted = "{} - executed".format(command)
    filename = "test_gtts.wav"
    try:
        gtts_obj = gTTS(text=command_formatted, lang=language, slow=False)
        gtts_obj.save(filename)
        #os.system('mpg321 --stereo {}'.format(filename))
        playsound(filename)
        os.remove(filename)
        print("Played")
    except Exception:
        print("Failed")

confirm_voice_command("HELLO")