# simple_whisper_integration.py
#
# Description:
# This script demonstrates a basic integration of OpenAI's Whisper model for
# voice command transcription and simple intent parsing. It serves as the
# foundational code for Chapter 12 of the Physical AI & Humanoid Robotics book.
#
# The script performs the following steps:
# 1. Records a short audio clip from the microphone.
# 2. Saves the audio clip as a WAV file.
# 3. Uses the Whisper ASR model to transcribe the audio into text.
# 4. Parses the transcribed text to identify a simple command and associated entities.
#
# Dependencies:
# - openai-whisper: The core speech recognition model.
# - sounddevice: For capturing audio from the microphone.
# - scipy: For writing the captured audio to a WAV file.
#
# Installation:
# pip install openai-whisper sounddevice scipy
#
# Note: This script requires a working microphone connected to your system.
# It also requires the Whisper model to be downloaded on the first run, which
# may take some time depending on the model size chosen (e.g., "base", "tiny").

import os
import whisper
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np

# --- Configuration ---
MODEL_SIZE = "base"         # Whisper model size (e.g., "tiny", "base", "small", "medium", "large")
SAMPLE_RATE = 16000         # Audio sample rate (Whisper prefers 16kHz)
RECORD_SECONDS = 5          # Duration of audio recording in seconds
AUDIO_FILENAME = "command.wav" # Temporary file to store the recorded audio

# --- 1. Audio Recording ---
def record_audio(duration, sample_rate):
    """
    Records audio from the default microphone for a given duration. 
    
    Args:
        duration (int): The recording duration in seconds.
        sample_rate (int): The audio sample rate.
        
    Returns:
        numpy.ndarray: The recorded audio data.
    """
    print(f"Recording for {duration} seconds...")
    try:
        audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
        sd.wait()  # Wait until recording is finished
        print("Recording complete.")
        return audio_data
    except Exception as e:
        print(f"Error during audio recording: {e}")
        print("Please ensure you have a working microphone and the 'sounddevice' library is installed correctly.")
        return None

def save_audio_to_wav(filename, audio_data, sample_rate):
    """
    Saves the recorded audio data to a WAV file.
    
    Args:
        filename (str): The path to the output WAV file.
        audio_data (numpy.ndarray): The audio data to save.
        sample_rate (int): The audio sample rate.
    """
    try:
        write(filename, sample_rate, audio_data)
        print(f"Audio saved to {filename}")
    except Exception as e:
        print(f"Error saving audio file: {e}")

# --- 2. Speech-to-Text Transcription ---
def transcribe_audio(model, audio_path):
    """
    Transcribes the given audio file using the Whisper model.
    
    Args:
        model: The loaded Whisper model.
        audio_path (str): The path to the audio file to transcribe.
        
    Returns:
        str: The transcribed text, or an empty string if transcription fails.
    """
    if not os.path.exists(audio_path):
        print(f"Audio file not found at: {audio_path}")
        return ""
        
    print("Transcribing audio... this may take a moment.")
    try:
        result = model.transcribe(audio_path)
        transcribed_text = result.get("text", "").strip()
        print("Transcription complete.")
        return transcribed_text
    except Exception as e:
        print(f"Error during transcription: {e}")
        return ""

# --- 3. Simple Intent Parsing ---
def parse_command(text):
    """
    Parses transcribed text to extract a simple intent and entities using keywords.
    
    Args:
        text (str): The transcribed text from the user.
        
    Returns:
        dict: A dictionary containing the 'intent' and 'entities', or None if no command is found.
    """
    text = text.lower()
    command = None
    
    # Define keywords for different intents
    fetch_keywords = ["get", "bring", "fetch", "retrieve"]
    move_keywords = ["go to", "move to", "navigate to"]
    
    # Check for fetch intent
    for keyword in fetch_keywords:
        if keyword in text:
            command = {"intent": "FETCH_OBJECT", "entities": {}}
            # Very basic entity extraction: find the object after the keyword
            try:
                # Find what comes after the keyword
                obj_str = text.split(keyword, 1)[1].strip()
                # A more robust parser would be needed for real applications
                command["entities"]["object_description"] = obj_str
            except IndexError:
                command["entities"]["object_description"] = "unknown"
            break
            
    # Check for move intent if no other command was found
    if not command:
        for keyword in move_keywords:
            if keyword in text:
                command = {"intent": "NAVIGATE_TO", "entities": {}}
                try:
                    # Find what comes after the keyword
                    loc_str = text.split(keyword, 1)[1].strip()
                    command["entities"]["location"] = loc_str
                except IndexError:
                    command["entities"]["location"] = "unknown"
                break

    return command

# --- Main Execution Block ---
def main():
    """
    Main function to run the voice command recognition pipeline.
    """
    print("Initializing Whisper model...")
    try:
        model = whisper.load_model(MODEL_SIZE)
    except Exception as e:
        print(f"Error loading Whisper model: {e}")
        print("Please ensure 'openai-whisper' and its dependencies (like 'ffmpeg') are installed correctly.")
        return
        
    # Record audio from the microphone
    audio_data = record_audio(RECORD_SECONDS, SAMPLE_RATE)
    
    if audio_data is not None:
        # Save the recorded audio
        save_audio_to_wav(AUDIO_FILENAME, audio_data, SAMPLE_RATE)
        
        # Transcribe the audio to text
        transcribed_text = transcribe_audio(model, AUDIO_FILENAME)
        
        if transcribed_text:
            print(f"\nTranscribed Text: '{transcribed_text}'")
            
            # Parse the command
            command = parse_command(transcribed_text)
            
            if command:
                print("\n--- Command Parsed ---")
                print(f"Intent: {command['intent']}")
                print("Entities:")
                for entity, value in command['entities'].items():
                    print(f"  - {entity}: {value}")
                print("----------------------")
            else:
                print("\nNo specific command intent recognized.")
        else:
            print("Could not transcribe the audio.")
            
        # Clean up the audio file
        if os.path.exists(AUDIO_FILENAME):
            os.remove(AUDIO_FILENAME)

if __name__ == "__main__":
    main()
