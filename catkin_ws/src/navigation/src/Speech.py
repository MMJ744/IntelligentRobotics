#!/usr/bin/env python
import random
import time

import speech_recognition as sr
from gtts import gTTS
import os
import playsound

def listen(show):
    recognizer = sr.Recognizer()
    microphone = sr.Microphone(device_index=0)

    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        audio = recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source, phrase_time_limit=5)

    return recognizer.recognize_google(audio, show_all=show)

def speech(text):
    print("speech:" + text)
    filename = str(text) + '.mp3'
    filename = filename.replace(' ','')
    filename = filename.replace('*','')
    if not os.path.exists(filename):
        tts = gTTS(text=text, lang='en-gh')
        tts.save(filename)
    playsound.playsound(filename)
    
print(listen(True))
