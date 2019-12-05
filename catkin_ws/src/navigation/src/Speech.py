#!/usr/bin/env python
import random
import time

import speech_recognition as sr
from gtts import gTTS
import os
import playsound

def listen():
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"]="KEYS.JSON"
    r = sr.Recognizer()
    file = sr.Microphone()

    with file as source:
        audio = r.adjust_for_ambient_noise(source)
        audio = r.listen(source)
        
    try:
        recog = r.recognize_google_cloud(audio, language = 'en-US')
        print("You said: " + recog)
    except sr.UnknownValueError as u:
        print(u)
        print("Google Cloud Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Cloud Speech Recognition service; {0}".format(e))  

def speech(text):
    print("speech:" + text)
    filename = str(text) + '.mp3'
    filename = filename.replace(' ','')
    if not os.path.exists(filename):
        tts = gTTS(text=text, lang='en-gh')
        tts.save(filename)
    playsound.playsound(filename)

print(listen())