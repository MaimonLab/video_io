# video_io

ROS2 package to play and save videos

- [Installation instructions](#installation_instructions)

<a name=installation_instructions></a>

# Installation instructions

Install FFMPEG through terminal:

```bash
sudo apt install ffmpeg
```

# Hardware acceleration

For use the GPU for hardware acceleration during the encoding step, you will need a compatible GPU (*e.g.* NVIDIA GeForce 20-series and newer) and the correct drivers. For NVIDIA cards, install relevant drivers and CUDA using terminal:

```bash
sudo apt-get install nvidia-driver-VERSION
sudo apt-get install cuda-toolkit-VERSION
```

To check that FFMPEG now recognizes `cuda` as a hardware acceleration option, run the following line in terminal and check that the output list contains `cuda`:

```bash
ffmpeg -hwaccels
```

# ROS options

* `image_topic`: ROS topic to listen to for images to save [*default*: camera/image]
* `output_fps`: playback FPS of the output video (**not** affected by FPS of incoming stream or ROS timestamps) [*default*: 60.0]
* `record_every_nth_frame`: save only every n-th frame that is received by the ROS subscriber [*default*: 1]
* `burn_timestamp`: burn the current timestamp and frame number on each frame [*default*: False]
* `quit_after_s_seconds`: close this node after a given number of seconds (disabled if less than 0) [*default*: -1]
* `codec`: codec to use for encoding the output video; common codecs (and their relative **performance** and resulting **file size**) include:
  * `raw`: each frame is saved as independent RAW; **fastest**, **largest**
  * `mjpeg` (*default*): each frame is saved as independent JPEG images with no timewise compression; **fast**, **large**
  * `h264`: a very commonly used video compression standard; **slow**, **small**
  * `h265` or `hevc`: a commonly used high-efficiency video compression standard; **average**, **small**
  * Run: `ffmpeg -encoders` to check all codecs available for encoding on your computer 
  > Note: to use GPU hardware acceleration, you must specify a compatible codec. For `-hwaccel cuda`, this is generally denoted with a prefix `nvenc_`. For example, to use `h265` compression with a CUDA-enabled GPU, specify `nvenc_hevc` as your codec.
* `encoder_args`: any additional encoder options; run: `ffmpeg -h encoder=CODEC` to check all available options for your codec of choice
  > **Note**: this MUST be provided in the form of a list

  > **Tip**: use this to control the compression quality. For example, for `h265` compression, put `[-rc, constantqp, -qp, 18]` for this parameter to set the compression QP to 18. Note that compression quality decreases with QP: qp=0 is lossless, qp=12-18 maintains high quality (visually lossless), qp=23-28 is most common/average compression, up to a max of qp=51. We use a default of qp=18 for `nvenc_hevc` codec, but for most other codecs, FFMPEG will default to qp=23.
* `save_as_single_video` (only for `burst_video_saver`): compile all burst save events as a single video stream, otherwise each burst record command will save their own partial video [*default*: True]


# Example

> ***DEPRECATED: new examples are in progress***

To run the basic example, type:

    ros2 launch video_io example.launch.py

This will prompt you to pick an index for one of the examples. The examples provided are:

1. Basic playing/saving of video
2. Saving a (temporally) downsampled video, recording only ever nth frame
3. Play a video at lower resolution (downsampled pixels) and record the video
4. Play a video, and save the video only after receiving a command. This command contains a burst duration variable.

You can experiment with different parameters in the ./config/example.yaml file. Each example is contained in it's own namespace, as signified with it's YAML indent.

# License and reuse

Currently this repository is only accessible for maimon lab members. We plan to release this repository in the future under the LGPLv3 license.
