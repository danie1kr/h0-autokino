/*
See https://github.com/danie1kr/h0-autokino for documentation
Supported Boards:
Makerfabs ESP32-S3 Parallel TFT with Touch (https://github.com/Makerfabs/Makerfabs-ESP32-S3-Parallel-TFT-with-Touch)
By danie1kr, 2024
*/

#include <vector>
#include <string>

//#define WITH_SERIAL

#define BENCHMARK
#ifdef BENCHMARK
#define WITH_SERIAL
//#define BENCHMARK_ZIP
//#define BENCHMARK_MJPEG
// Balu 1150 frames in 58954 ms:  51.2643 ms/frame =  19.5067 frames/s, longest frame: 72
//#define BENCHMARK_JPEG
// Balu 1151 frames in 69513 ms:  60.3936 ms/frame =  16.5581 frames/s, longest frame: 72
#define BENCHMARK_PACK

#endif
#define SPI_DRIVER_SELECT 0
#define LGFX_USE_V1

// SD card interface
#include <SPI.h>
#include <SdFat.h>
#define SPI_CLOCK SD_SCK_MHZ(50)
#define SD_CONFIG SdSpiConfig(SD_CS, USER_SPI_BEGIN | DEDICATED_SPI, SPI_CLOCK)
SdFat SD;

// display and touch
#include <LovyanGFX.hpp>
#include <driver/i2c.h>
#include <Wire.h>
const unsigned int screenWidth = 480;
const unsigned int screenHeight = 320;

// display jpegs
#include <JPEGDEC.h>
static JPEGDEC jpeg;
File globalFile; // loading image
uint32_t largestJPEGFileSize = 0;
uint8_t* jpegBuffer = nullptr;

// for movies file
#include <ArduinoJson.h>


#ifdef BENCHMARK_MJPEG
//#include "MjpegClass.h"

class MjpegClass
{
public:
  bool setup(
      File *file, uint8_t *buffer, const size_t bufferSize, JPEGDEC *jpeg, JPEG_DRAW_CALLBACK *pfnDraw, bool useBigEndian,
      int x, int y)
  {

  }

  bool loadNextFrame()
  {

  }

  bool drawFrame()
  {
    this->jpeg->openRAM(buffer, frameSize, this->jpegDraw);

    if (this->useBigEndian)
    {
      this->jpeg->setPixelType(RGB565_BIG_ENDIAN);
    }
    this->jpeg->decode(this->x, this->y, 0);
    this->jpeg->close();

    return true;
  }
private:
  int x, y;
  bool useBigEndian;
  File *file;
  uint8_t *buffer;
  const size_t bufferSize;
  JPEGDEC *jpeg;
  JPEG_DRAW_CALLBACK *jpegDraw;

  size_t frameSize;
}

#endif

#ifdef BENCHMARK_ZIP
#include <unzipLIB.h>
UNZIP zip;
#endif

// PIN configuration
#define SPI_MOSI 2
#define SPI_MISO 41
#define SPI_SCK 42
#define SD_CS 1
#define LCD_CS 37
#define LCD_BLK 45
#define I2C_SCL 39
#define I2C_SDA 38

#define TOUCH_I2C_ADD 0x38
#define TOUCH_REG_XL 0x04
#define TOUCH_REG_XH 0x03
#define TOUCH_REG_YL 0x06
#define TOUCH_REG_YH 0x05

// lgfx main class for Makerfabs ESP32-S3 Parallel TFT with TouchMakerfabs ESP32-S3 Parallel TFT with Touch
class LGFX : public lgfx::LGFX_Device
{
    static constexpr int I2C_PORT_NUM = I2C_NUM_0;
    static constexpr int I2C_PIN_SDA = 38;
    static constexpr int I2C_PIN_SCL = 39;
    static constexpr int I2C_PIN_INT = 40;

    lgfx::Bus_Parallel16 _bus_instance;
    lgfx::Panel_ILI9488 _panel_instance;
    lgfx::Light_PWM     _light_instance;
    lgfx::ITouch* _touch_instance_ptr = nullptr;

    // Configure the touch panel
    bool init_impl(bool use_reset, bool use_clear) override
    {
        if (_touch_instance_ptr == nullptr)
        {
            lgfx::ITouch::config_t cfg;
            lgfx::i2c::init(I2C_PORT_NUM, I2C_PIN_SDA, I2C_PIN_SCL);
            if (lgfx::i2c::beginTransaction(I2C_PORT_NUM, 0x38, 400000, false).has_value()
                && lgfx::i2c::endTransaction(I2C_PORT_NUM).has_value())
            {
                _touch_instance_ptr = new lgfx::Touch_FT5x06();
                cfg = _touch_instance_ptr->config();
                cfg.i2c_addr = 0x38;
                cfg.x_max = screenHeight;
                cfg.y_max = screenWidth;
            }
            else
                if (lgfx::i2c::beginTransaction(I2C_PORT_NUM, 0x48, 400000, false).has_value()
                    && lgfx::i2c::endTransaction(I2C_PORT_NUM).has_value())
                {
                    _touch_instance_ptr = new lgfx::Touch_NS2009();
                    cfg = _touch_instance_ptr->config();
                    cfg.i2c_addr = 0x48;
                    cfg.x_min = 368;
                    cfg.y_min = 212;
                    cfg.x_max = 3800;
                    cfg.y_max = 3800;
                }
            if (_touch_instance_ptr != nullptr)
            {
                cfg.i2c_port = I2C_PORT_NUM;
                cfg.pin_sda = I2C_PIN_SDA;
                cfg.pin_scl = I2C_PIN_SCL;
                cfg.pin_int = I2C_PIN_INT;
                cfg.freq = 400000;
                cfg.bus_shared = false;
                _touch_instance_ptr->config(cfg);
                _panel_instance.touch(_touch_instance_ptr);
            }
        }
        return lgfx::LGFX_Device::init_impl(use_reset, use_clear);
    }

public:

    // Configure Panel
    LGFX(void)
    {
        {
            auto cfg = _bus_instance.config();

            cfg.freq_write = 40000000;
            cfg.pin_wr = 35;
            cfg.pin_rd = 48;
            cfg.pin_rs = 36;

            cfg.pin_d0 = 47;
            cfg.pin_d1 = 21;
            cfg.pin_d2 = 14;
            cfg.pin_d3 = 13;
            cfg.pin_d4 = 12;
            cfg.pin_d5 = 11;
            cfg.pin_d6 = 10;
            cfg.pin_d7 = 9;
            cfg.pin_d8 = 3;
            cfg.pin_d9 = 8;
            cfg.pin_d10 = 16;
            cfg.pin_d11 = 15;
            cfg.pin_d12 = 7;
            cfg.pin_d13 = 6;
            cfg.pin_d14 = 5;
            cfg.pin_d15 = 4;
            _bus_instance.config(cfg);
            _panel_instance.bus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs = -1;
            cfg.pin_rst = -1;
            cfg.pin_busy = -1;
            cfg.offset_rotation = 0;
            cfg.readable = true;
            cfg.invert = false;
            cfg.rgb_order = false;
            cfg.dlen_16bit = true;
            cfg.memory_width = screenHeight;
            cfg.memory_height = screenWidth;
            cfg.panel_width = screenHeight;
            cfg.panel_height = screenWidth;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.bus_shared = true;

            _panel_instance.config(cfg);
        }

        {
            auto cfg = _light_instance.config();

            cfg.pin_bl = 45;
            cfg.invert = false;
            cfg.freq = 44100;
            cfg.pwm_channel = 7;

            _light_instance.config(cfg);
            _panel_instance.light(&_light_instance);
        }
        setPanel(&_panel_instance);
    }
};

static LGFX lcd;

const unsigned int movieFrameStart = 1;
struct Movie
{
    const std::string path;
    const unsigned int frames;

    Movie(const std::string path, const unsigned int frames) : path(path), frames(frames) { };
};
std::vector<Movie> movies;

#ifdef BENCHMARK_PACK

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

File packFile;
void pack(Movie *movie, std::string targetFileName)
{
  if(SD.exists(targetFileName.c_str()))
  {
    SD.remove(targetFileName.c_str());
  }

  auto currentFrame = movieFrameStart;
  File target = SD.open(targetFileName.c_str(), O_RDWR | O_CREAT);

  lcd.setCursor(0, 20);
  lcd.print("Getting sizes");
  for(currentFrame = movieFrameStart; currentFrame < movie->frames; ++currentFrame)
  {
    std::string jpegFileName = movie->path + "/" + frameToFilename(currentFrame);
    File f = SD.open(jpegFileName.c_str());
    if (f) 
    {
      uint32_t size = f.size();
      if(size < largestJPEGFileSize)
        largestJPEGFileSize = size;
      f.close();
    }
  }
  Serial.printf("largest file: %dkb\n", largestJPEGFileSize);
  lcd.setCursor(0, 32);
  lcd.print("Packing");

  uint32_t bufferSize = 32*1024;
  uint8_t *buffer = (uint8_t*)malloc(bufferSize);

  target.write(&largestJPEGFileSize, sizeof(largestJPEGFileSize));
  uint32_t overallSize = sizeof(uint32_t) * (movie->frames + 1); 
  for(currentFrame = movieFrameStart; currentFrame <= movie->frames; ++currentFrame)
  {
    std::string jpegFileName = movie->path + "/" + frameToFilename(currentFrame);
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
          Serial.printf("issue with toRead %d and left %d", toRead, left);
          while(1);
        }
        left -= toRead;
        // Serial.printf("Packing %d total: %d read: %d left: %d transfer: %d\n", currentFrame, size, read, left, toRead);
      }
      overallSize += size;
      f.close();
    }
    lcd.setCursor(0, 44);
    lcd.print(jpegFileName.c_str());
  }
  free(buffer);
  Serial.printf("File %s size on disk %d content %d", targetFileName.c_str(), target.size(), overallSize);
  target.close();
  lcd.setCursor(0, 56);
  lcd.print("done");
}

bool loadPacked(std::string path)
{
  packFile = SD.open(path.c_str());
  if(!packFile)
    informAndHalt("cannot open packfile");

  packFile.read(&largestJPEGFileSize, sizeof(largestJPEGFileSize));
  if(jpegBuffer)
    free(jpegBuffer);

  Serial.printf("alloc %d for jpeg buffer\n", largestJPEGFileSize);

  jpegBuffer = (uint8_t*)malloc(sizeof(uint8_t) * largestJPEGFileSize);
  if (!jpegBuffer)
    informAndHalt("cannot alloc memory for movie frame JPEG :(");

  return true;
}

bool playNextPacked()
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

#endif

// stop and display error if something bad happened
void informAndHalt(const char* text)
{
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
    lcd.pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
    return 1;
}

// JPEG helpers for loading image
void * jpegOpen(const char *filename, int32_t *size) {
  globalFile = SD.open(filename);
  *size = globalFile.size();
  return &globalFile;
}

void jpegClose(void *handle) {
  if (globalFile) globalFile.close();
}

int32_t jpegRead(JPEGFILE *handle, uint8_t *buffer, int32_t length) {
  if (!globalFile) return 0;
  return globalFile.read(buffer, length);
}

int32_t jpegSeek(JPEGFILE *handle, int32_t position) {
  if (!globalFile) return 0;
  return globalFile.seek(position);
}

#ifdef BENCHMARK
unsigned long startBenchmark, finishBenchmark, framesBenchmark;
unsigned int frameStart = 0, maxFrameDuration = 0;
Movie balu("/movies/balu", 1151);
#endif

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

    lcd.init();
    lcd.setRotation(3);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    if (!SD.begin(SD_CONFIG))
    {
        informAndHalt("cannot init SD");
        SD.initErrorHalt(&Serial);
    }

    const char loadingScreenJPEG[] = "/loading.jpg";
    if(SD.exists(loadingScreenJPEG))
    {
      jpeg.open(loadingScreenJPEG, jpegOpen, jpegClose, jpegRead, jpegSeek, jpegDraw);
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      jpeg.decode(0, 0, 0);
      jpeg.close();
    }
    Wire.begin(I2C_SDA, I2C_SCL);
    byte error;
    Wire.beginTransmission(TOUCH_I2C_ADD);
    error = Wire.endTransmission();

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

#ifdef BENCHMARK_PACK
    //pack(&balu, "/movies/balu.pack");
#endif

#ifndef BENCHMARK_PACK
#ifdef WITH_SERIAL
    Serial.printf("Alloc largestJPEGFileSize: %d\n", largestJPEGFileSize);
#endif

    jpegBuffer = (uint8_t*)malloc(sizeof(uint8_t) * largestJPEGFileSize);
    if (!jpegBuffer)
      informAndHalt("cannot alloc memory for movie frame JPEG");
#endif
    lcd.setCursor(0, 0);
}

bool movieRunning = false;
Movie* currentMovie;
File movieFile;
unsigned int currentFrame = movieFrameStart;
unsigned long lastFrame = 0;
const unsigned long FPSdelay = 1000 / 20;
unsigned int consecutiveTouches = 0;

#ifdef BENCHMARK_JPEG
#endif
#ifdef BENCHMARK_MJPEG
MjpegClass mjpeg;
const size_t mjpegBufferSize = 20 * 1024; // we know
uint8_t mjpegBuffer[mjpegBufferSize];
#endif
#ifdef BENCHMARK_ZIP
const size_t zipBufferSize = 20 * 1024; // we know
uint8_t zipBuffer[zipBufferSize];

static File zipFile;
// JPEG helpers for loading image
void * zipOpen(const char *filename, int32_t *size) {
  zipFile = SD.open(filename);
  *size = globalFile.size();
  return &zipFile;
}

void zipClose(void *p) {
  ZIPFILE *pzf = (ZIPFILE *)p;
  File *f = (File *)pzf->fHandle;
  if (f) f->close();
}

int32_t zipRead(void *p, uint8_t *buffer, int32_t length) {
  ZIPFILE *pzf = (ZIPFILE *)p;
  File *f = (File *)pzf->fHandle;
  return f->read(buffer, length);
}

int32_t zipSeek(void *p, int32_t position, int iType) {
  ZIPFILE *pzf = (ZIPFILE *)p;
  File *f = (File *)pzf->fHandle;
  if (iType == SEEK_SET)
    return f->seek(position);
  else if (iType == SEEK_END) {
    return f->seek(position + pzf->iSize); 
  } else { // SEEK_CUR
    long l = f->position();
    return f->seek(l + position);
  }
}

#endif

unsigned int cinema()
{
#if defined(BENCHMARK_MJPEG)

  if(!movieRunning)
  {
    movieFile = SD.open("/movies/balu.mjpeg");
    mjpeg.setup(&movieFile, mjpegBuffer, jpegDraw, true, 0, 0, screenWidth, screenHeight );
    startBenchmark = millis();
    framesBenchmark = 0;
  }
 
  //if(mjpeg.readMjpegBuf())
  while(movieFile.available())
  {
    auto frameStart = millis();
    mjpeg.readMjpegBuf();
    mjpeg.drawJpg();
    ++framesBenchmark;
    auto duration = millis() - frameStart;
    if(duration > maxFrameDuration)
      maxFrameDuration = duration;
  }
  //else
  {
    movieRunning = false;
  }

#elif defined(BENCHMARK_ZIP)
  int rc;
  if(!movieRunning)
  {
    rc = zip.openZIP("/movies/balu.zip", zipOpen, zipClose, zipRead, zipSeek);
    if (rc == UNZ_OK) {
      zip.gotoFirstFile();
      movieRunning = true;
    }
    else
    {
      Serial.printf("cannot open zip /movies/balu.zip: %d\n", rc);
      movieRunning = false;
    }
  }

  if(movieRunning)
  {
    zip.openCurrentFile();
    char szComment[256], szName[256];
    unz_file_info fi;
    rc = zip.getFileInfo(&fi, szName, sizeof(szName), NULL, 0, szComment, sizeof(szComment));
    if(fi.uncompressed_size < zipBufferSize)
    {
      rc = zip.readCurrentFile(zipBuffer, fi.uncompressed_size); // we know the uncompressed size of these BMP images
      jpeg.openRAM((uint8_t*)zipBuffer, fi.uncompressed_size, jpegDraw);
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      jpeg.decode(0, 0, 0);
      jpeg.close();
    }

    rc = zip.gotoNextFile();
    if(rc != UNZ_OK)
    {
      Serial.printf("no more nextfile in zip\n");
      movieRunning = false;
    }
  }

  if(!movieRunning)
  {
    zip.closeZIP();
  }
#elif defined(BENCHMARK_PACK)

  if(!movieRunning)
  {
    loadPacked("/movies/balu.pack");
    movieRunning = true;
    startBenchmark = millis();
    framesBenchmark = 0;
  }

  if(movieRunning)
  {
#ifdef BENCHMARK
    auto frameStart = millis();
#endif

    movieRunning = playNextPacked();

#ifdef BENCHMARK
    ++framesBenchmark;
    auto duration = millis() - frameStart;
    if(duration > maxFrameDuration)
      maxFrameDuration = duration;
#endif
  }

  if(!movieRunning)
  {
    packFile.close();
    Serial.printf("done after %d frames\n", framesBenchmark);
  }

#else//if defined()
    unsigned int returnDelay = 0;
    if (!movieRunning)
    {
      #ifdef BENCHMARK
        currentMovie = &balu;
        startBenchmark = millis();
        framesBenchmark = 0;
      #else
        currentMovie = &movies[random(movies.size())];
      #endif
        movieRunning = true;
        currentFrame = movieFrameStart;
    }

    if (movieRunning)
    {
#ifdef BENCHMARK
        auto frameStart = millis();
#endif
        std::string jpegFileName = currentMovie->path + "/";
        // ffmpeg created files with setting %04d.jpg
        if (currentFrame < 1000)
            jpegFileName += "0";
        if (currentFrame < 100)
            jpegFileName += "0";
        if (currentFrame < 10)
            jpegFileName += "0";
        jpegFileName += std::to_string(currentFrame);
        jpegFileName += ".jpg";
        File f = SD.open(jpegFileName.c_str());
        if (f)
        {
            size_t size = f.size();
            if (size < largestJPEGFileSize)
            {
                f.read(jpegBuffer, size);

                jpeg.openRAM((uint8_t*)jpegBuffer, size, jpegDraw);
                jpeg.setPixelType(RGB565_BIG_ENDIAN);
                jpeg.decode(0, 0, 0);
                jpeg.close();

                f.close();

                const unsigned long lastFrameDuration = millis() - lastFrame;
                if(lastFrameDuration < FPSdelay)
                {
                  lastFrame = millis();
                  returnDelay = FPSdelay - lastFrameDuration - 1;
                }
            }

            ++currentFrame;
            if (currentFrame > currentMovie->frames)
            {
              Serial.println("last frame played");
                movieRunning = false;
            }
            #ifdef BENCHMARK
            ++framesBenchmark;
            auto duration = millis() - frameStart;
            if(duration > maxFrameDuration)
              maxFrameDuration = duration;
            #endif
        }
        else
        {
            Serial.printf("%s not found\n", jpegFileName.c_str());
            movieRunning = false;
        }
    }

#ifndef BENCHMARK
    uint16_t touchX, touchY;
    if (lcd.getTouch(&touchX, &touchY))
    {
      ++consecutiveTouches;
      if (consecutiveTouches > 20) // about 1s touch
      {
        // will switch to the next movie
        consecutiveTouches = 0;
        movieRunning = false;
      }
    }

    return returnDelay;
#endif
#endif

#ifdef BENCHMARK
    if(!movieRunning)
    {
      Serial.println("Statistics:");
      finishBenchmark = millis();
      Serial.printf("%d frames in %d ms\n", framesBenchmark, finishBenchmark - startBenchmark);
      Serial.flush();
      
      /*auto time = finishBenchmark - startBenchmark;
      float msf = (float)time / (float)framesBenchmark;
      float fps = 1.f / (msf / 1000.f);
      Serial.printf("%d frames in %d ms: %8.4f ms/frame = %8.4f frames/s, longest frame: %d\n", framesBenchmark, time, msf, fps, maxFrameDuration);
      Serial.flush();
      int y = 20;
      lcd.setCursor(20, y);
      lcd.print(framesBenchmark);
      lcd.setCursor(20, y += 16);
      lcd.print(time);
      lcd.setCursor(20, y += 16);
      lcd.print(msf);
      lcd.setCursor(20, y += 16);
      lcd.print(fps);
      lcd.setCursor(20, y += 16);
      lcd.print(maxFrameDuration);
      lcd.setCursor(20, y += 16);
      lcd.print("halt");*/
      while(1);
    }
  return 0;
#endif
}

void loop() {

  auto waitForNextFrame = cinema();
  if(waitForNextFrame)
    delay(waitForNextFrame);

}
