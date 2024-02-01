/*
See https://github.com/danie1kr/h0-autokino for documentation
Supported Boards:
Makerfabs ESP32-S3 Parallel TFT with Touch (https://github.com/Makerfabs/Makerfabs-ESP32-S3-Parallel-TFT-with-Touch)
By danie1kr, 2024
*/

#include <vector>
#include <string>

#include "loading.h"

//#define BENCHMARK
#ifdef BENCHMARK
#define WITH_SERIAL
//#define BENCHMARK_MJPEG
// Balu 1150 frames in 58954 ms:  51.2643 ms/frame =  19.5067 frames/s, longest frame: 72
//#define BENCHMARK_JPEG
// Balu 1151 frames in 69513 ms:  60.3936 ms/frame =  16.5581 frames/s, longest frame: 72
//#define BENCHMARK_PACK
// Balu 1151 frames in 57657 ms:  50.0929 ms/frame = 19.9628 frames/s
//  JPEG DEC 1.40:
// Balu 1151 frames in 29732 ms:  25.8315ms/frame  = 38.7125 frames/s
// Balu 1151 frames in 34067 ms:  29.5977ms/frame  = 33.7864 frames/s
#endif

// SD card interface
#define SPI_DRIVER_SELECT 0
#include <SPI.h>
#include <SdFat.h>
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdSpiConfig(SD_CS, USER_SPI_BEGIN | DEDICATED_SPI, SPI_CLOCK)
SdFat SD;

// lcd and touch
#include <bb_captouch.h>
#include <bb_spi_lcd.h>
const unsigned int screenWidth = 480;
const unsigned int screenHeight = 320;

// display jpegs
#include <JPEGDEC.h>
static JPEGDEC jpeg;
uint32_t largestJPEGFileSize = 0;
uint8_t* jpegBuffer = nullptr;

// for movies file
#include <ArduinoJson.h>

// PIN configuration
#define SPI_MOSI 2
#define SPI_MISO 41
#define SPI_SCK 42
#define SD_CS 1
#define LCD_CS 37
#define LCD_BLK 45
#define I2C_SCL 39
#define I2C_SDA 38

BBCapTouch touch;
BB_SPI_LCD lcd;

const unsigned int movieFrameStart = 1;
struct Movie
{
    const std::string path;
    const unsigned int frames;

    Movie(const std::string path, const unsigned int frames) : path(path), frames(frames) { };
};
std::vector<Movie> movies;

// get Frame name
std::string frameToFilename(unsigned int frame)
{
  // ffmpeg created files with setting %04d.jpg
  std::string name;
  if (frame < 1000)
    name += "0";
  if (frame < 100)
    name += "0";
  if (frame < 10)
    name += "0";
  name += std::to_string(frame);
  name += ".jpg";
  return name;
}

// pack Frames into one file with size prefix so we can quickly jump
File packFile;
void packMovieJPEGs(Movie &movie, const std::string targetFileName, const size_t chunkSize)
{
  if(SD.exists(targetFileName.c_str()))
  {
    Serial.printf("deleting %s\n", targetFileName.c_str());
    SD.remove(targetFileName.c_str());
  }

  auto currentFrame = movieFrameStart;
  File target = SD.open(targetFileName.c_str(), O_RDWR | O_CREAT);

  lcd.setCursor(0, 20);
  lcd.print(targetFileName.c_str());

  lcd.setCursor(0, 32);
  lcd.print("Getting sizes");
  uint32_t largestFrameOfMovie = 0;
  for(currentFrame = movieFrameStart; currentFrame < movie.frames; ++currentFrame)
  {
    std::string jpegFileName = movie.path + "/" + frameToFilename(currentFrame);
    File f = SD.open(jpegFileName.c_str());
    if (f) 
    {
      uint32_t size = f.size();
      if(size < largestFrameOfMovie)
        largestFrameOfMovie = size;
      f.close();
    }
  }
  Serial.printf("largest file: %dkb\n", largestFrameOfMovie);
  lcd.setCursor(0, 44);
  lcd.print("Packing");

  uint32_t bufferSize = chunkSize;
  uint8_t *buffer = (uint8_t*)malloc(bufferSize);

  target.write(&largestFrameOfMovie, sizeof(largestFrameOfMovie));
  uint32_t overallSize = sizeof(uint32_t) * (movie.frames + 1); 
  for(currentFrame = movieFrameStart; currentFrame <= movie.frames; ++currentFrame)
  {
    std::string jpegFileName = movie.path + "/" + frameToFilename(currentFrame);
    File f = SD.open(jpegFileName.c_str());
    if (f)
    {
      uint32_t size = f.size();
      target.write(&size, sizeof(size));

      uint32_t read = 0;
      uint32_t left = size;
      while(read < size)
      {
        uint32_t toRead = min(left, bufferSize);
        f.read(buffer, toRead);
        target.write(buffer, toRead);
        read += toRead;
        if(toRead > left)
        {
          informAndHalt("issue with toRead left when packing a movie");
        }
        left -= toRead;
      }
      overallSize += size;
      f.close();
    }
    lcd.setCursor(0, 56);
    lcd.print(jpegFileName.c_str());
  }
  free(buffer);
  Serial.printf("File %s size on disk %d content %d", targetFileName.c_str(), target.size(), overallSize);
  target.close();
  lcd.setCursor(0, 68);
  lcd.print("done");
}

// load a packed file
bool packLoad(std::string path)
{
  packFile = SD.open(path.c_str());
  if(!packFile)
    informAndHalt("cannot open packfile");

  uint32_t largestFrameOfMovie;
  packFile.read(&largestFrameOfMovie, sizeof(largestFrameOfMovie));

  if(largestFrameOfMovie > largestJPEGFileSize)
  {
    largestJPEGFileSize = largestFrameOfMovie;
    if(jpegBuffer)
      free(jpegBuffer);

    Serial.printf("realloc %d for jpeg buffer\n", largestJPEGFileSize);

    jpegBuffer = (uint8_t*)malloc(sizeof(uint8_t) * largestJPEGFileSize);
    if (!jpegBuffer)
      informAndHalt("cannot alloc memory for movie frame JPEG :(");
  }

  return true;
}

// play next frame if avialable
bool packPlayNextFrame()
{
  if(packFile.available())
  {
    uint32_t frameSize;
    packFile.read(&frameSize, sizeof(frameSize));
    packFile.read(jpegBuffer, frameSize);

    jpeg.openRAM((uint8_t*)jpegBuffer, frameSize, jpegDraw);
    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    jpeg.decode(0, 0, 0);
    jpeg.close();

    return true;
  }
  else
    return false;
}

// stop and display error if something bad happened
void informAndHalt(const char* text)
{
  Serial.printf("Halt due to: %s\n", text);
  Serial.flush();
  lcd.setCursor(20, 20);
  lcd.print("Halt due to error:");
  lcd.setCursor(20, 42);
  lcd.print(text);
  while(true);
}

// read movie infos from file
bool collectMovies(SdFat& fs, const char* dirname)
{
#ifdef WITH_SERIAL
    Serial.printf("Listing directory: %s\n", dirname);
#endif

    File root = fs.open(dirname);
    if (!root)
    {
#ifdef WITH_SERIAL
        Serial.println("Failed to open directory");
#endif
        return false;
    }
    if (!root.isDirectory())
    {
#ifdef WITH_SERIAL
        Serial.println("Not a directory");
#endif
        return false;
    }

    std::string path = std::string(dirname) + std::string("/movies.json");
    File jsonFile = fs.open(path.c_str());
    if(jsonFile)
    {
      Serial.println("Using movies.json");
      size_t jsonFileSize = jsonFile.size();
      char* jsonBuffer = (char*)malloc(jsonFileSize + 1);
      memset(jsonBuffer, 0, jsonFileSize+1);
      jsonFile.read(jsonBuffer, jsonFileSize);
      JsonDocument json;
      deserializeJson(json, jsonBuffer, jsonFileSize + 1);
      largestJPEGFileSize = json["maxFileSize"].as<size_t>();
      auto jsonMovies = json["movies"].as<JsonArray>();
      for(auto movie : jsonMovies)
      {
        std::string path = std::string(dirname) + std::string("/") + movie["name"].as<std::string>();
        auto frames = movie["frameCount"].as<unsigned int>();
        if(frames > 0)
        {
#ifdef WITH_SERIAL
          Serial.print("Dir: "); Serial.print(path.c_str()); Serial.print("Frames: "); Serial.println(frames);
#endif
          movies.emplace_back(path, frames);
        }
      }
      jsonFile.close();
#ifdef WITH_SERIAL
      Serial.print("Maximum size: "); Serial.println(largestJPEGFileSize);
#endif
      free(jsonBuffer);
    }
    else
    {
#ifdef WITH_SERIAL
      Serial.println("missing /movies/movies.json");
#endif
      return false;
    }

    return true;
}

// draw JPEG portion to screen
int jpegDraw(JPEGDRAW* pDraw)
{
    lcd.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels, DRAW_TO_LCD);
    return 1;
}

// display loading image when waiting
void displayLoadingScreen()
{
  jpeg.openFLASH((uint8_t *)loading, sizeof(loading), jpegDraw);
  jpeg.setPixelType(RGB565_BIG_ENDIAN);
  jpeg.decode(0, 0, 0);
  jpeg.close();
}

// setup
void setup() {
    pinMode(LCD_CS, OUTPUT);
    pinMode(LCD_BLK, OUTPUT);

    digitalWrite(LCD_CS, LOW);
    digitalWrite(LCD_BLK, HIGH);

#ifdef WITH_SERIAL
    Serial.begin(115200);
    delay(200);
    Serial.println("hello from h0 autokino board");
    Serial.flush();
#endif

    lcd.begin(DISPLAY_MAKERFABS_S3);
    lcd.setRotation(3);
    displayLoadingScreen();

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    if (!SD.begin(SD_CONFIG))
    {
        informAndHalt("cannot init SD");
        SD.initErrorHalt(&Serial);
    }

    constexpr int I2C_PIN_SDA = 38;
    constexpr int I2C_PIN_SCL = 39;
    int error = touch.init(I2C_PIN_SDA, I2C_PIN_SCL);

    lcd.setCursor(0, 48);
    if (error != 0)
    {
#ifdef WITH_SERIAL
        Serial.print("Unknown error at address 0x");
        Serial.println(TOUCH_I2C_ADD, HEX);
        lcd.print("ERROR:   TOUCH");
#endif
    }

    if(!collectMovies(SD, "/movies"))
      informAndHalt("no movies found");

    for(auto &movie : movies)
    {
      std::string packedFileName = movie.path + ".pack";
      if(!SD.exists(packedFileName.c_str()))
      {
        packMovieJPEGs(movie, packedFileName, 32*1024);
        displayLoadingScreen();
      }
    }

#ifdef WITH_SERIAL
    Serial.print("Alloc largestJPEGFileSize: ");
    Serial.println(largestJPEGFileSize);
#endif
    jpegBuffer = (uint8_t*)malloc(sizeof(uint8_t) * largestJPEGFileSize);
    if (!jpegBuffer)
      informAndHalt("cannot alloc memory for movie frame JPEG");

    lcd.setCursor(0, 0);
}

bool movieRunning = false;
unsigned long lastFrameTime = 0;
const unsigned long FPSdelay = 1000 / 20;
unsigned int consecutiveTouches = 0;
Movie *currentMovie = nullptr;

#ifdef BENCHMARK
unsigned long benchmarkMovieStart = 0;
Movie balu("/movies/balu", 1151);
#endif

unsigned int cinema()
{
  unsigned int returnDelay = 0;
  if(!movieRunning)
  {
#ifdef BENCHMARK
    currentMovie = &balu;
#else
    currentMovie = &movies[random(movies.size())];
#endif
    packLoad(currentMovie->path + ".pack");
    movieRunning = true;
#ifdef BENCHMARK
    benchmarkMovieStart = millis();
#endif
  }

  if(movieRunning)
  {
    movieRunning = packPlayNextFrame();

    const unsigned long lastFrameDuration = millis() - lastFrameTime;
    if(lastFrameDuration < FPSdelay)
      returnDelay = FPSdelay - lastFrameDuration - 1;
    lastFrameTime = millis();
  }

  if(!movieRunning)
  {
    packFile.close();
#ifdef BENCHMARK
    auto duration = millis() - benchmarkMovieStart;
    float msf = ((float)duration)/((float)currentMovie->frames);
    float fps = 1.f / (msf / 1000.f);
    Serial.printf("movie %s: %d frames in %d ms. %7.4fms/frame, %7.4fFPS\n", currentMovie->path.c_str(), currentMovie->frames, duration, msf, fps);
#endif
  }

#ifdef BENCHMARK
  return 0;
#endif

  TOUCHINFO ti;
  if (touch.getSamples(&ti)) 
  {
    if(ti.count > 0)
    {
      ++consecutiveTouches;
      if (consecutiveTouches > 20) // about 1s touch
      {
        // will switch to the next movie
        consecutiveTouches = 0;
        movieRunning = false;
      }
    }
  }
  return returnDelay;
}

void loop() {
  auto waitForNextFrame = cinema();
  if(waitForNextFrame)
    delay(waitForNextFrame);
}
