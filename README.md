# Video Player.

This is a distributed video and audio player for the Allosphere. It can player
equirectangular surround videos or regular rectangular videos thorugh omni-
rendering.

The application takes an argument that can be a video file or a video player
"session".

# Usage

Press '[' or ']' to seek 10 seconds back or forward.

Press space bar to pause/play.

Press *Tab* to show/hide the GUI.

# Session format

The session is described in a TOML file. The session file must end in ".toml"
or it will be used as a vide file.

## Video

The video is described in the TOML file like this:

```
videoFile = "LastWhispers_040719_ambix_360.mp4"
windowed=true
position=[0,0,-5]
quaternion=[1,0,0,0]
scale=[1.0,1.0,1.0]
```

if *windowed* is true, the renderers will display a slab with the video instead
of mapping the video to a sphere. The position and scale only affect this mode.
However, the quaternion can be used to rotate the sphere.

If no *videoFile* is provided, this can serve as an audio only player. 

## Audio

Audio is managed in the configuration file using the same format and configuration
for the mulichannel file player in allolin_playground tools/audio.

```
[[audioFile]]
name = "mc_out0016_v000313_series0001_ALL630.wav"
outChannels = [16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 
                26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
                36, 37, 38, 39, 40, 41, 42, 43, 44, 45 ]
gain = 1.0

[[audioFile]]
name = "mc_out0016_v000313_series0001_ALL612L.wav"
outChannels = [48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59]
gain = 1.0
```

You can add any number of audio files that map to audio device channels (indexed
from 0). The number of elements in outChannels must match the number of channels 
in the audio file.
