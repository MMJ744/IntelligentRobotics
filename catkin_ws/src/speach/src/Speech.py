import random
import time

import speech_recognition as sr
from gtts import gTTS
import os
import pyaudio

def listen():
    # create recognizer and mic instances
    pyaudio.PyAudio().open(format=pyaudio.paInt16,
                        rate=44100,
                        channels=1, #change this to what your sound card supports
                        input_device_index=6, #change this your input sound card index
                        input=True,
                        output=False,
                        frames_per_buffer=1024)
    recognizer = sr.Recognizer()
    microphone = sr.Microphone(device_index=6)

    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

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
    print(response)
    return response

def speech(text):
    print(text)
    #tts = gTTS(text='Hello, do you have a booking', lang='en')
    #tts.save("text.mp3")