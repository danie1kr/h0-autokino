# h0-autokino
With [winston](https://github.com/danie1kr/winston), I also want to have a display on my model rail road.
Best option to hide a display is a drive in cinema. This project is the cinema part. Feel free to adapt it to your model rail road as well.

![H0 Autokino](./res/image.jpg?raw=true)

# Requirements
* Board:
  * [Makerfabs ESP32-S3 Parallel TFT with Touch](https://github.com/Makerfabs/Makerfabs-ESP32-S3-Parallel-TFT-with-Touch)

* Arduino Libraries:
  * SdFat
  * LovyanGFX
  * Wire
  * JPEGDEC
  * ArduinoJson

* For standalone:
  * USB-C male breakout
  * 5V power source
  * Some stands, see [floor](res/Autokino-Floor.dxf) and [side](res/Autokino-Side-x2.dxf) DXF files as an example.

# Usage
Run ffmpeg.ps1, put the files on the SD card and the SD card into the board and power it up.
The board will read the movies and display them in a random order. Touch the screen for about 1s to flip to the next random movie.
Usually, the code reports errors on the lcd. You can enable the define `WITH_SERIAL` to get more debug information on the serial output.

# SD card setup
The following structure is expected:

    SD \
       + loading.jpg
       + movies \
                + movies.json
                + movie1
                + movie2

`loading.jpeg` is  a 480x320 size JPEG to be shown on screen. If it is not available, nothing is displayed instead.

# movies.json
The file holds information from the JPEG generation as getting all the necessary frames and sizes during boot takes too long. This is its structure:

     {
	     "maxFileSize": 24000, #largest JPEG in bytes
	     "movies": [
		    { "name": "movie1", "frameCount": 1200},
		    { "name": "movie2", "frameCount": 1858},
		    ... ]
	  }

# ffmpeg.ps1
The file creates the SD structure.
Put it in the same folder as your mp4 files and run it as a powershell script. It will create a directory `sd` which is to be used as the root of the SD card for the board.

# Have fun!
