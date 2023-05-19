from gtts import gTTS
from tempfile import TemporaryFile
from pygame import mixer
from time import sleep

def speakDuration(text):
    words = text.split()
    readSpeed = 175
    readTime = len(words) / readSpeed
    return readTime + 4

def speak(text):
    tts = gTTS(text, lang='en', tld='co.uk')
    mixer.init()
    sf = TemporaryFile()
    tts.write_to_fp(sf)
    sf.seek(0)
    mixer.music.load(sf)
    mixer.music.play()

    sleep(speakDuration(text))

