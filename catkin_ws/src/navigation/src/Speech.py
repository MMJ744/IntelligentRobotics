#!/usr/bin/env python
import random
import time

import speech_recognition as sr
from gtts import gTTS
import os
import playsound

def listen():
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        # recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source, phrase_time_limit=3)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"
    print(response["transcription"])
    return response["transcription"]

def speech(text):
    print("speech:" + text)
    filename = str(text) + '.mp3'
    filename = filename.replace(' ','')
    if not os.path.exists(filename):
        tts = gTTS(text=text, lang='en-gh')
        tts.save(filename)
    playsound.playsound(filename)

def navigate(where):
    print('Going to ' + where)
