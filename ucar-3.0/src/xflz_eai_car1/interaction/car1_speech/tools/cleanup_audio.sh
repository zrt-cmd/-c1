#!/bin/bash
# Clean up audio processes that may be blocking the device

echo "Checking for audio processes..."

# Check if audio device is in use
if lsof /dev/snd/pcmC0D0c /dev/snd/pcmC0D0p 2>/dev/null; then
    echo "Audio device is busy. Killing processes..."
    pkill -9 arecord
    pkill -9 aplay
    sleep 1
    echo "Done. Audio device should now be available."
else
    echo "Audio device is available."
fi
