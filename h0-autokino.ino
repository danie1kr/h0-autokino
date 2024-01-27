/*
See https://github.com/danie1kr/h0-autokino for documentation
Supported Boards:
Makerfabs ESP32-S3 Parallel TFT with Touch (https://github.com/Makerfabs/Makerfabs-ESP32-S3-Parallel-TFT-with-Touch)
By danie1kr, 2024
*/

#include <vector>
#include <string>

//#define WITH_SERIAL

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
size_t largestJPEGFileSize = 0;
uint8_t* jpegBuffer;

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
Movie* currentMovie;
File movieFile;
unsigned int currentFrame = movieFrameStart;
unsigned long lastFrame = 0;
const unsigned long FPSdelay = 1000 / 20;
unsigned int consecutiveTouches = 0;

unsigned int cinema()
{
    unsigned int returnDelay = 0;
    if (!movieRunning)
    {
        currentMovie = &movies[random(movies.size())];
        movieRunning = true;
        currentFrame = movieFrameStart;
    }

    if (movieRunning)
    {
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
                movieRunning = false;
        }
        else
            movieRunning = false;
    }

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
}

void loop() {

  auto waitForNextFrame = cinema();
  if(waitForNextFrame)
    delay(waitForNextFrame);

}
