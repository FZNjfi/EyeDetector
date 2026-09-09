#include "esp_camera.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_heap_caps.h>
#include <mbedtls/base64.h>
#include <math.h>


// ============================================================
// CAMERA - AI THINKER ESP32-CAM
// ============================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// ============================================================
// IMAGE
// ============================================================

#define IMG_W 320
#define IMG_H 240

#define SMALL_W 160
#define SMALL_H 120

uint8_t *grayImage = nullptr;
uint8_t *workImage = nullptr;


// ============================================================
// ESP-NOW
// ============================================================

uint8_t carAddress[] =
{
    0xA4, 0xCF, 0x12, 0xFF, 0x0D, 0xFE
};


struct ControlPacket
{
    uint8_t command;
    float x;
    float y;
};


// ============================================================
// COMMANDS
// هماهنگ با ESP8266
// ============================================================

enum Command : uint8_t
{
    CMD_NONE      = 0,

    CMD_LEFT_1    = 1,
    CMD_LEFT_2    = 2,
    CMD_LEFT_3    = 3,

    CMD_RIGHT_1   = 4,
    CMD_RIGHT_2   = 5,
    CMD_RIGHT_3   = 6,

    CMD_FORWARD   = 7
};


Command lastCommand = CMD_NONE;


// ============================================================
// MOTOR COMMAND CONTROL
// ============================================================

// برای FORWARD می‌توانیم فرمان را دوره‌ای بفرستیم.
// ولی فرمان‌های چرخش فقط یک بار ارسال می‌شوند.

#define FORWARD_COMMAND_INTERVAL 100

unsigned long lastCommandTime = 0;


// ============================================================
// LOST TRACKING
// ============================================================

#define TRACK_MAX_MISSES 4

int trackMisses = 0;


// ============================================================
// GAZE CALIBRATION
// ============================================================

// تعداد نمونه برای کالیبراسیون مرکز
#define CALIBRATION_SAMPLES 30

// محدوده‌ای که بعد از calibration به عنوان مرکز
// برای حرکت Forward در نظر گرفته می‌شود.
#define GAZE_CENTER_RANGE 0.10f

bool gazeCalibrated = false;

int calibrationCount = 0;
float calibrationSum = 0.0f;

float calibratedCenterX = 0.50f;


// ============================================================
// ADAPTIVE CENTER
// ============================================================

// مرکز را فقط وقتی آرام تغییر می‌دهیم که gaze نزدیک مرکز باشد.
// این کار باعث می‌شود اگر هدبند/صورت کمی جابه‌جا شد، سیستم
// دوباره با مرکز جدید سازگار شود.

#define ADAPTIVE_CENTER_ENABLED true
#define ADAPTIVE_CENTER_ALPHA 0.015f

// حداکثر سرعت تغییر مرکز
#define CENTER_MIN 0.20f
#define CENTER_MAX 0.80f


// ============================================================
// GAZE COMMAND THRESHOLDS
// ============================================================

// فاصله gaze از مرکز تعیین‌کننده شدت چرخش است.
//
// مثال اگر center = 0.35:
//
// LEFT_1  : 0.25 تا 0.35
// LEFT_2  : 0.15 تا 0.25
// LEFT_3  : کمتر از 0.15
//
// RIGHT_1 : 0.35 تا 0.45
// RIGHT_2 : 0.45 تا 0.55
// RIGHT_3 : بیشتر از 0.55
//
// اما این‌ها بر اساس center محاسبه می‌شوند.

#define TURN_LEVEL_1 0.10f
#define TURN_LEVEL_2 0.20f


// ============================================================
// DETECTION PARAMETERS
// ============================================================

#define FACE_X_MIN 20
#define FACE_X_MAX 300

#define FACE_Y_MIN 25
#define FACE_Y_MAX 190

#define EYE_BAND_Y_MIN 15
#define EYE_BAND_Y_MAX 145

#define MIN_EYE_W 15
#define MAX_EYE_W 75

#define MIN_EYE_H 5
#define MAX_EYE_H 35

#define MIN_EYE_AREA 50
#define MAX_EYE_AREA 1500

#define MIN_EYE_DISTANCE 15
#define MAX_EYE_DISTANCE 190

#define MAX_EYE_DY 10

#define MAX_SIZE_RATIO 2.2f

#define MIN_PAIR_SCORE 45.0f


// ============================================================
// TRACKING
// ============================================================

#define TRACK_SEARCH_X 35
#define TRACK_SEARCH_Y 25

#define TRACK_STEP 2

#define TEMPLATE_W 32
#define TEMPLATE_H 20

#define TEMPLATE_HALF_W 16
#define TEMPLATE_HALF_H 10

// template فقط بعد از چند فریم معتبر update می‌شود
#define TEMPLATE_UPDATE_RATE 5

#define MAX_TRACK_ERROR 75.0f

// حداکثر حرکت عمودی tracker
// این مقدار عمداً سخت‌تر از قبل است.
#define MAX_TRACK_VERTICAL_JUMP 12

// حداکثر حرکت افقی
#define MAX_TRACK_HORIZONTAL_JUMP 35


bool tracking = false;
int trackFrameCounter = 0;


// ============================================================
// TEMPLATE
// ============================================================

uint8_t *leftTemplate = nullptr;
uint8_t *rightTemplate = nullptr;

bool templatesInitialized = false;


// ============================================================
// CONTINUITY
// ============================================================

#define CONTINUITY_RADIUS_X 45.0f
#define CONTINUITY_RADIUS_Y 25.0f

#define CONTINUITY_WEIGHT 0.8f


// ============================================================
// TEMPORAL FILTER
// ============================================================

#define FILTER_ALPHA 0.6f

float filteredLeftX = 0;
float filteredLeftY = 0;

float filteredRightX = 0;
float filteredRightY = 0;

bool filterInitialized = false;


// ============================================================
// EYE STRUCT
// ============================================================

struct EyeCandidate
{
    int x;
    int y;

    int w;
    int h;

    int area;

    float density;
    float compactness;

    bool valid;
};


EyeCandidate bestLeftEye;
EyeCandidate bestRightEye;


// ============================================================
// PUPIL
// ============================================================

struct Pupil
{
    float x;
    float y;

    bool valid;

    float confidence;
};


Pupil leftPupil;
Pupil rightPupil;


// ============================================================
// GAZE
// ============================================================

float lastGazeX = 0.5f;
float lastGazeY = 0.5f;


// ============================================================
// FREE RTOS
// ============================================================

TaskHandle_t leftTaskHandle;
TaskHandle_t rightTaskHandle;

SemaphoreHandle_t leftStartSemaphore;
SemaphoreHandle_t rightStartSemaphore;

SemaphoreHandle_t leftDoneSemaphore;
SemaphoreHandle_t rightDoneSemaphore;


// ============================================================
// TASK RESULT
// ============================================================

EyeCandidate leftCandidates[12];
EyeCandidate rightCandidates[12];

volatile int leftCandidateCount = 0;
volatile int rightCandidateCount = 0;


// ============================================================
// MEMORY
// ============================================================

void allocateMemory()
{
    grayImage = (uint8_t *)heap_caps_malloc(
        IMG_W * IMG_H,
        MALLOC_CAP_SPIRAM
    );

    workImage = (uint8_t *)heap_caps_malloc(
        IMG_W * IMG_H,
        MALLOC_CAP_SPIRAM
    );

    leftTemplate = (uint8_t *)heap_caps_malloc(
        TEMPLATE_W * TEMPLATE_H,
        MALLOC_CAP_SPIRAM
    );

    rightTemplate = (uint8_t *)heap_caps_malloc(
        TEMPLATE_W * TEMPLATE_H,
        MALLOC_CAP_SPIRAM
    );

    if (!grayImage ||
        !workImage ||
        !leftTemplate ||
        !rightTemplate)
    {
        Serial.println("MEMORY ERROR");

        while (1)
            delay(1000);
    }
}


// ============================================================
// SAVE TEMPLATE
// ============================================================

void saveEyeTemplate(
    EyeCandidate &eye,
    uint8_t *templ
)
{
    int startX = eye.x - TEMPLATE_HALF_W;
    int startY = eye.y - TEMPLATE_HALF_H;

    for (int y = 0; y < TEMPLATE_H; y++)
    {
        for (int x = 0; x < TEMPLATE_W; x++)
        {
            int sx = startX + x;
            int sy = startY + y;

            if (sx < 0 ||
                sx >= IMG_W ||
                sy < 0 ||
                sy >= IMG_H)
            {
                templ[y * TEMPLATE_W + x] = 128;
            }
            else
            {
                templ[y * TEMPLATE_W + x] =
                    grayImage[sy * IMG_W + sx];
            }
        }
    }
}


void initializeTemplates()
{
    saveEyeTemplate(
        bestLeftEye,
        leftTemplate
    );

    saveEyeTemplate(
        bestRightEye,
        rightTemplate
    );

    templatesInitialized = true;
}


// ============================================================
// TEMPLATE ERROR
// ============================================================

float templateError(
    int centerX,
    int centerY,
    uint8_t *templ
)
{
    long totalError = 0;
    int samples = 0;

    int startX = centerX - TEMPLATE_HALF_W;
    int startY = centerY - TEMPLATE_HALF_H;

    for (int y = 0; y < TEMPLATE_H; y += 2)
    {
        for (int x = 0; x < TEMPLATE_W; x += 2)
        {
            int sx = startX + x;
            int sy = startY + y;

            if (sx < 0 ||
                sx >= IMG_W ||
                sy < 0 ||
                sy >= IMG_H)
            {
                continue;
            }

            int current =
                grayImage[sy * IMG_W + sx];

            int reference =
                templ[y * TEMPLATE_W + x];

            totalError += abs(current - reference);

            samples++;
        }
    }

    if (samples == 0)
        return 999999.0f;

    return (float)totalError / samples;
}


// ============================================================
// TRACK ONE EYE
// ============================================================

bool trackOneEye(
    EyeCandidate &eye,
    uint8_t *templ
)
{
    int oldX = eye.x;
    int oldY = eye.y;

    float bestError = 999999.0f;

    int bestX = oldX;
    int bestY = oldY;

    for (
        int dy = -TRACK_SEARCH_Y;
        dy <= TRACK_SEARCH_Y;
        dy += TRACK_STEP
    )
    {
        for (
            int dx = -TRACK_SEARCH_X;
            dx <= TRACK_SEARCH_X;
            dx += TRACK_STEP
        )
        {
            int x = oldX + dx;
            int y = oldY + dy;

            if (abs(dy) > MAX_TRACK_VERTICAL_JUMP)
                continue;

            if (abs(dx) > MAX_TRACK_HORIZONTAL_JUMP)
                continue;

            if (x < TEMPLATE_HALF_W ||
                x >= IMG_W - TEMPLATE_HALF_W)
                continue;

            if (y < TEMPLATE_HALF_H ||
                y >= IMG_H - TEMPLATE_HALF_H)
                continue;

            float error =
                templateError(
                    x,
                    y,
                    templ
                );

            if (error < bestError)
            {
                bestError = error;

                bestX = x;
                bestY = y;
            }
        }
    }

    if (bestError > MAX_TRACK_ERROR)
        return false;

    eye.x = bestX;
    eye.y = bestY;

    return true;
}


// ============================================================
// UPDATE TEMPLATE
// ============================================================

void updateTemplate(
    EyeCandidate &eye,
    uint8_t *templ
)
{
    int startX = eye.x - TEMPLATE_HALF_W;
    int startY = eye.y - TEMPLATE_HALF_H;

    for (int y = 0; y < TEMPLATE_H; y++)
    {
        for (int x = 0; x < TEMPLATE_W; x++)
        {
            int sx = startX + x;
            int sy = startY + y;

            if (sx < 0 ||
                sx >= IMG_W ||
                sy < 0 ||
                sy >= IMG_H)
            {
                continue;
            }

            int current =
                grayImage[sy * IMG_W + sx];

            int old =
                templ[y * TEMPLATE_W + x];

            templ[y * TEMPLATE_W + x] =
                (old * 7 + current * 3) / 10;
        }
    }
}


// ============================================================
// CAMERA INIT
// ============================================================

bool initCamera()
{
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;

    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;

    config.jpeg_quality = 12;
    config.fb_count = 2;

    config.grab_mode = CAMERA_GRAB_LATEST;

    if (psramFound())
    {
        config.fb_location = CAMERA_FB_IN_PSRAM;
    }
    else
    {
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK)
    {
        Serial.printf(
            "Camera init failed: 0x%x\n",
            err
        );

        return false;
    }

    sensor_t *s =
        esp_camera_sensor_get();

    s->set_framesize(
        s,
        FRAMESIZE_QVGA
    );

    s->set_pixformat(
        s,
        PIXFORMAT_GRAYSCALE
    );

    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);

    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);

    return true;
}


// ============================================================
// COPY FRAME
// ============================================================

void copyFrame(camera_fb_t *fb)
{
    if (fb->width != IMG_W ||
        fb->height != IMG_H)
        return;

    memcpy(
        grayImage,
        fb->buf,
        IMG_W * IMG_H
    );
}


// ============================================================
// LIGHT BLUR
// ============================================================

uint8_t blurPixel(int x, int y)
{
    int sum = 0;
    int count = 0;

    for (int dy = -1; dy <= 1; dy++)
    {
        int yy = y + dy;

        if (yy < 0 ||
            yy >= IMG_H)
            continue;

        for (int dx = -1; dx <= 1; dx++)
        {
            int xx = x + dx;

            if (xx < 0 ||
                xx >= IMG_W)
                continue;

            sum +=
                grayImage[
                    yy * IMG_W + xx
                ];

            count++;
        }
    }

    return sum / count;
}


// ============================================================
// PREPROCESS
// ============================================================

void preprocess()
{
    for (int y = 1;
         y < IMG_H - 1;
         y++)
    {
        for (int x = 1;
             x < IMG_W - 1;
             x++)
        {
            workImage[
                y * IMG_W + x
            ] =
                blurPixel(x, y);
        }
    }
}


// ============================================================
// LOCAL THRESHOLD
// ============================================================

int calculateLocalThreshold(
    int x1,
    int y1,
    int x2,
    int y2
)
{
    long sum = 0;
    long sum2 = 0;

    int count = 0;

    for (int y = y1;
         y < y2;
         y += 2)
    {
        for (int x = x1;
             x < x2;
             x += 2)
        {
            int v =
                workImage[
                    y * IMG_W + x
                ];

            sum += v;
            sum2 += v * v;

            count++;
        }
    }

    if (count == 0)
        return 60;

    float mean =
        (float)sum / count;

    float variance =
        ((float)sum2 / count)
        - mean * mean;

    if (variance < 0)
        variance = 0;

    float stddev =
        sqrtf(variance);

    float threshold =
        mean - 0.65f * stddev;

    if (threshold < 25)
        threshold = 25;

    if (threshold > 120)
        threshold = 120;

    return (int)threshold;
}


// ============================================================
// FLOOD FILL
// ============================================================

#define QUEUE_SIZE 1000

struct Pixel
{
    int x;
    int y;
};

Pixel queueBuffer[QUEUE_SIZE];


bool darkPixel(
    int x,
    int y,
    int threshold
)
{
    return
        workImage[
            y * IMG_W + x
        ] < threshold;
}


EyeCandidate floodFill(
    int sx,
    int sy,
    int threshold
)
{
    EyeCandidate result;

    result.valid = false;

    int head = 0;
    int tail = 0;

    queueBuffer[tail++] =
        {sx, sy};

    workImage[
        sy * IMG_W + sx
    ] = 255;

    int minX = sx;
    int maxX = sx;

    int minY = sy;
    int maxY = sy;

    int area = 0;

    while (head < tail)
    {
        Pixel p =
            queueBuffer[head++];

        area++;

        if (p.x < minX)
            minX = p.x;

        if (p.x > maxX)
            maxX = p.x;

        if (p.y < minY)
            minY = p.y;

        if (p.y > maxY)
            maxY = p.y;

        const int nx[4] =
        {
            p.x + 1,
            p.x - 1,
            p.x,
            p.x
        };

        const int ny[4] =
        {
            p.y,
            p.y,
            p.y + 1,
            p.y - 1
        };

        for (int i = 0; i < 4; i++)
        {
            int xx = nx[i];
            int yy = ny[i];

            if (
                xx < 1 ||
                xx >= IMG_W - 1 ||
                yy < 1 ||
                yy >= IMG_H - 1
            )
                continue;

            if (!darkPixel(
                    xx,
                    yy,
                    threshold))
                continue;

            if (tail >= QUEUE_SIZE)
            {
                workImage[
                    yy * IMG_W + xx
                ] = 255;

                continue;
            }

            workImage[
                yy * IMG_W + xx
            ] = 255;

            queueBuffer[
                tail++
            ] =
                {xx, yy};
        }
    }

    int w =
        maxX - minX + 1;

    int h =
        maxY - minY + 1;


    if (area < MIN_EYE_AREA)
        return result;

    if (area > MAX_EYE_AREA)
        return result;

    if (w < MIN_EYE_W ||
        w > MAX_EYE_W)
        return result;

    if (h < MIN_EYE_H ||
        h > MAX_EYE_H)
        return result;


    float density =
        (float)area /
        (float)(w * h);

    if (density < 0.12f ||
        density > 0.90f)
        return result;


    float aspect =
        (float)w / h;

    if (aspect < 0.8f ||
        aspect > 8.0f)
        return result;


    result.x =
        (minX + maxX) / 2;

    result.y =
        (minY + maxY) / 2;

    result.w = w;
    result.h = h;

    result.area = area;

    result.density =
        density;

    result.compactness =
        density;

    result.valid = true;

    return result;
}


// ============================================================
// DETECT CANDIDATES
// ============================================================

void detectCandidates(
    int xMin,
    int xMax,
    EyeCandidate *output,
    volatile int *count
)
{
    *count = 0;

    int threshold =
        calculateLocalThreshold(
            xMin,
            EYE_BAND_Y_MIN,
            xMax,
            EYE_BAND_Y_MAX
        );


    for (int y = EYE_BAND_Y_MIN;
         y < EYE_BAND_Y_MAX;
         y++)
    {
        for (int x = xMin;
             x < xMax;
             x++)
        {
            if (
                grayImage[
                    y * IMG_W + x
                ] < threshold
            )
            {
                workImage[
                    y * IMG_W + x
                ] =
                    grayImage[
                        y * IMG_W + x
                    ];
            }
            else
            {
                workImage[
                    y * IMG_W + x
                ] = 255;
            }
        }
    }


    for (int y =
             EYE_BAND_Y_MIN + 1;
         y <
             EYE_BAND_Y_MAX - 1;
         y++)
    {
        for (int x =
                 xMin + 1;
             x <
                 xMax - 1;
             x++)
        {
            if (*count >= 12)
                return;

            if (!darkPixel(
                    x,
                    y,
                    threshold))
                continue;

            EyeCandidate c =
                floodFill(
                    x,
                    y,
                    threshold
                );

            if (!c.valid)
                continue;

            if (
                c.y < EYE_BAND_Y_MIN ||
                c.y > EYE_BAND_Y_MAX
            )
                continue;

            output[
                *count
            ] = c;

            (*count)++;
        }
    }
}


void detectLeft()
{
    detectCandidates(
        FACE_X_MIN,
        160,
        leftCandidates,
        &leftCandidateCount
    );
}


void detectRight()
{
    detectCandidates(
        160,
        FACE_X_MAX,
        rightCandidates,
        &rightCandidateCount
    );
}


// ============================================================
// PAIR SCORE
// ============================================================

float pairScore(
    EyeCandidate &l,
    EyeCandidate &r
)
{
    int dx =
        r.x - l.x;

    int dy =
        abs(r.y - l.y);


    if (dx < MIN_EYE_DISTANCE)
        return -100000;

    if (dx > MAX_EYE_DISTANCE)
        return -100000;

    if (dy > MAX_EYE_DY)
        return -100000;

    if (dx < dy * 4)
        return -100000;


    float areaRatio =
        (float)max(
            l.area,
            r.area
        ) /
        (float)min(
            l.area,
            r.area
        );

    if (areaRatio >
        MAX_SIZE_RATIO)
        return -100000;


    float widthRatio =
        (float)max(
            l.w,
            r.w
        ) /
        (float)min(
            l.w,
            r.w
        );

    if (widthRatio > 2.0f)
        return -100000;


    float score = 0;

    float idealDistance = 125;

    score +=
        40.0f -
        fabsf(
            dx - idealDistance
        ) * 0.25f;

    score +=
        50.0f -
        dy * 4.0f;

    score +=
        30.0f -
        fabsf(
            areaRatio - 1.0f
        ) * 25.0f;

    score +=
        20.0f -
        fabsf(
            widthRatio - 1.0f
        ) * 15.0f;

    score +=
        20.0f -
        fabsf(
            l.density -
            r.density
        ) * 30.0f;

    return score;
}


// ============================================================
// TRACK BOTH EYES
// ============================================================

bool trackEyes()
{
    if (!templatesInitialized)
        return false;


    EyeCandidate newLeft =
        bestLeftEye;

    EyeCandidate newRight =
        bestRightEye;


    bool leftOK =
        trackOneEye(
            newLeft,
            leftTemplate
        );

    bool rightOK =
        trackOneEye(
            newRight,
            rightTemplate
        );


    if (!leftOK ||
        !rightOK)
        return false;


    int dx =
        newRight.x -
        newLeft.x;

    int dy =
        abs(
            newRight.y -
            newLeft.y
        );


    if (dx <
        MIN_EYE_DISTANCE)
        return false;

    if (dx >
        MAX_EYE_DISTANCE)
        return false;

    if (dy >
        MAX_EYE_DY)
        return false;

    if (dx <
        dy * 3)
        return false;


    if (
        abs(
            newLeft.x -
            bestLeftEye.x
        ) >
        TRACK_SEARCH_X
    )
        return false;


    if (
        abs(
            newRight.x -
            bestRightEye.x
        ) >
        TRACK_SEARCH_X
    )
        return false;


    bestLeftEye =
        newLeft;

    bestRightEye =
        newRight;

    return true;
}


// ============================================================
// TRACKING FILTER
// ============================================================

void updateTrackingFilter()
{
    float lx =
        bestLeftEye.x;

    float ly =
        bestLeftEye.y;

    float rx =
        bestRightEye.x;

    float ry =
        bestRightEye.y;


    if (!filterInitialized)
    {
        filteredLeftX = lx;
        filteredLeftY = ly;

        filteredRightX = rx;
        filteredRightY = ry;

        filterInitialized = true;

        return;
    }


    filteredLeftX =
        FILTER_ALPHA *
        filteredLeftX +
        (1.0f - FILTER_ALPHA) *
        lx;

    filteredLeftY =
        FILTER_ALPHA *
        filteredLeftY +
        (1.0f - FILTER_ALPHA) *
        ly;


    filteredRightX =
        FILTER_ALPHA *
        filteredRightX +
        (1.0f - FILTER_ALPHA) *
        rx;

    filteredRightY =
        FILTER_ALPHA *
        filteredRightY +
        (1.0f - FILTER_ALPHA) *
        ry;


    bestLeftEye.x =
        (int)filteredLeftX;

    bestLeftEye.y =
        (int)filteredLeftY;

    bestRightEye.x =
        (int)filteredRightX;

    bestRightEye.y =
        (int)filteredRightY;
}


// ============================================================
// SELECT BEST PAIR
// ============================================================

bool selectBestEyePair(
    bool haveLast
)
{
    float bestTotal =
        -1e9;

    bool found = false;


    for (int i = 0;
         i < leftCandidateCount;
         i++)
    {
        for (int j = 0;
             j < rightCandidateCount;
             j++)
        {
            float quality =
                pairScore(
                    leftCandidates[i],
                    rightCandidates[j]
                );


            if (quality <= -1000.0f)
                continue;

            if (quality <
                MIN_PAIR_SCORE)
                continue;


            float total =
                quality;


            if (haveLast)
            {
                float dLx =
                    fabsf(
                        leftCandidates[i].x -
                        filteredLeftX
                    );

                float dLy =
                    fabsf(
                        leftCandidates[i].y -
                        filteredLeftY
                    );


                float dRx =
                    fabsf(
                        rightCandidates[j].x -
                        filteredRightX
                    );

                float dRy =
                    fabsf(
                        rightCandidates[j].y -
                        filteredRightY
                    );


                float bonusL = 0;
                float bonusR = 0;


                if (
                    dLx <=
                        CONTINUITY_RADIUS_X &&
                    dLy <=
                        CONTINUITY_RADIUS_Y
                )
                {
                    bonusL =
                        (CONTINUITY_RADIUS_X - dLx) +
                        (CONTINUITY_RADIUS_Y - dLy);
                }


                if (
                    dRx <=
                        CONTINUITY_RADIUS_X &&
                    dRy <=
                        CONTINUITY_RADIUS_Y
                )
                {
                    bonusR =
                        (CONTINUITY_RADIUS_X - dRx) +
                        (CONTINUITY_RADIUS_Y - dRy);
                }


                total +=
                    (bonusL + bonusR) *
                    CONTINUITY_WEIGHT;
            }


            if (total > bestTotal)
            {
                bestTotal = total;

                bestLeftEye =
                    leftCandidates[i];

                bestRightEye =
                    rightCandidates[j];

                found = true;
            }
        }
    }

    return found;
}


// ============================================================
// PUPIL DETECTION
// ============================================================

// سخت‌گیری اصلی برای ابرو:
//
// چشم را به این صورت فرض می‌کنیم:
//
// ┌───────────────────┐
// │  ممنوع - ابرو     │
// │  ممنوع - ابرو     │
// │                   │
// │      ● pupil      │
// │                   │
// └───────────────────┘
//
// بنابراین ناحیه بالایی اصلاً برای مردمک استفاده نمی‌شود.

#define PUPIL_HALF_W 28
#define PUPIL_TOP_OFFSET 5
#define PUPIL_BOTTOM_OFFSET 11

#define EYEBROW_REJECT_RATIO 0.22f

#define PUPIL_MIN_PIXELS 3
#define PUPIL_MAX_PIXELS 350

#define PUPIL_MIN_DENSITY 0.015f
#define PUPIL_MAX_DENSITY 0.55f

#define MAX_PUPIL_JUMP_X 18.0f
#define MAX_PUPIL_JUMP_Y 8.0f


Pupil findPupil(
    EyeCandidate &eye,
    Pupil &previousPupil
)
{
    Pupil result;

    result.valid = false;
    result.confidence = 0;


    // --------------------------------------------------------
    // ROI مردمک
    // --------------------------------------------------------

    int x1 =
        max(
            0,
            eye.x - PUPIL_HALF_W
        );

    int x2 =
        min(
            IMG_W - 1,
            eye.x + PUPIL_HALF_W
        );


    // عمداً قسمت بالایی ROI حذف شده
    int y1 =
        max(
            0,
            eye.y - PUPIL_TOP_OFFSET
        );

    int y2 =
        min(
            IMG_H - 1,
            eye.y + PUPIL_BOTTOM_OFFSET
        );


    if (y2 <= y1)
        return result;


    // --------------------------------------------------------
    // پیدا کردن تاریک‌ترین مقدار
    // --------------------------------------------------------

    int minValue = 255;

    for (int y = y1;
         y <= y2;
         y++)
    {
        for (int x = x1;
             x <= x2;
             x++)
        {
            int v =
                grayImage[
                    y * IMG_W + x
                ];

            if (v < minValue)
                minValue = v;
        }
    }


    // threshold سخت‌تر
    int threshold =
        minValue + 22;

    if (threshold > 85)
        threshold = 85;


    long sumX = 0;
    long sumY = 0;
    long count = 0;


    int minPX = IMG_W;
    int maxPX = 0;

    int minPY = IMG_H;
    int maxPY = 0;


    // --------------------------------------------------------
    // پیدا کردن dark pixels
    // --------------------------------------------------------

    for (int y = y1;
         y <= y2;
         y++)
    {
        for (int x = x1;
             x <= x2;
             x++)
        {
            int v =
                grayImage[
                    y * IMG_W + x
                ];


            if (v <= threshold)
            {
                sumX += x;
                sumY += y;

                count++;


                if (x < minPX)
                    minPX = x;

                if (x > maxPX)
                    maxPX = x;

                if (y < minPY)
                    minPY = y;

                if (y > maxPY)
                    maxPY = y;
            }
        }
    }


    // --------------------------------------------------------
    // تعداد پیکسل
    // --------------------------------------------------------

    if (
        count < PUPIL_MIN_PIXELS ||
        count > PUPIL_MAX_PIXELS
    )
        return result;


    float px =
        (float)sumX /
        count;

    float py =
        (float)sumY /
        count;


    // --------------------------------------------------------
    // بررسی اینکه candidate به لبه ROI نچسبیده باشد
    // --------------------------------------------------------

    if (
        px < x1 + 3 ||
        px > x2 - 3
    )
        return result;


    if (
        py < y1 + 2 ||
        py > y2 - 2
    )
        return result;


    // --------------------------------------------------------
    // جلوگیری از ابرو
    // --------------------------------------------------------

    float relativeY =
        (py - y1) /
        (float)(y2 - y1);


    if (
        relativeY <
        EYEBROW_REJECT_RATIO
    )
    {
        return result;
    }


    // --------------------------------------------------------
    // شکل candidate
    // --------------------------------------------------------

    int pw =
        maxPX - minPX + 1;

    int ph =
        maxPY - minPY + 1;


    if (pw <= 0 ||
        ph <= 0)
        return result;


    float density =
        (float)count /
        (float)(pw * ph);


    if (
        density <
            PUPIL_MIN_DENSITY ||
        density >
            PUPIL_MAX_DENSITY
    )
    {
        return result;
    }


    // ابرو معمولاً خیلی کشیده است
    float aspect =
        (float)pw / ph;


    if (
        aspect > 3.0f ||
        aspect < 0.25f
    )
    {
        return result;
    }


    // --------------------------------------------------------
    // بررسی فاصله با مردمک قبلی
    // --------------------------------------------------------

    if (previousPupil.valid)
    {
        float dx =
            fabsf(
                px -
                previousPupil.x
            );

        float dy =
            fabsf(
                py -
                previousPupil.y
            );


        if (
            dx > MAX_PUPIL_JUMP_X ||
            dy > MAX_PUPIL_JUMP_Y
        )
        {
            return result;
        }
    }


    // --------------------------------------------------------
    // Confidence
    // --------------------------------------------------------

    float confidence = 1.0f;


    if (relativeY < 0.30f)
        confidence -= 0.35f;


    if (aspect > 2.0f)
        confidence -= 0.25f;


    if (density < 0.03f)
        confidence -= 0.20f;


    if (confidence < 0)
        confidence = 0;


    result.x = px;
    result.y = py;

    result.confidence =
        confidence;

    result.valid = true;

    return result;
}


// ============================================================
// GAZE
// ============================================================

bool calculateGaze(
    float &gx,
    float &gy
)
{
    Pupil previousLeft =
        leftPupil;

    Pupil previousRight =
        rightPupil;


    leftPupil =
        findPupil(
            bestLeftEye,
            previousLeft
        );


    rightPupil =
        findPupil(
            bestRightEye,
            previousRight
        );


    if (
        !leftPupil.valid ||
        !rightPupil.valid
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Eye ROI
    // --------------------------------------------------------

    float leftEyeLeft =
        bestLeftEye.x - 32;

    float leftEyeRight =
        bestLeftEye.x + 32;


    float rightEyeLeft =
        bestRightEye.x - 32;

    float rightEyeRight =
        bestRightEye.x + 32;


    float lx =
        (
            leftPupil.x -
            leftEyeLeft
        ) /
        (
            leftEyeRight -
            leftEyeLeft
        );


    float rx =
        (
            rightPupil.x -
            rightEyeLeft
        ) /
        (
            rightEyeRight -
            rightEyeLeft
        );


    float ly =
        (
            leftPupil.y -
            (bestLeftEye.y - 14)
        ) / 28.0f;


    float ry =
        (
            rightPupil.y -
            (bestRightEye.y - 14)
        ) / 28.0f;


    gx =
        (lx + rx) * 0.5f;

    gy =
        (ly + ry) * 0.5f;


    gx =
        constrain(
            gx,
            0.0f,
            1.0f
        );

    gy =
        constrain(
            gy,
            0.0f,
            1.0f
        );


    lastGazeX = gx;
    lastGazeY = gy;


    return true;
}


// ============================================================
// GAZE CALIBRATION
// ============================================================

void resetCalibration()
{
    gazeCalibrated = false;

    calibrationCount = 0;

    calibrationSum = 0;

    calibratedCenterX =
        0.50f;
}


void updateCalibration(
    float gazeX
)
{
    if (gazeCalibrated)
        return;


    // فقط gaze های معقول
    if (
        gazeX < 0.15f ||
        gazeX > 0.85f
    )
        return;


    calibrationSum += gazeX;

    calibrationCount++;


    if (
        calibrationCount >=
        CALIBRATION_SAMPLES
    )
    {
        calibratedCenterX =
            calibrationSum /
            calibrationCount;


        calibratedCenterX =
            constrain(
                calibratedCenterX,
                CENTER_MIN,
                CENTER_MAX
            );


        gazeCalibrated = true;


        Serial.printf(
            ">>> GAZE CALIBRATED: %.3f <<<\n",
            calibratedCenterX
        );
    }
}


// ============================================================
// ADAPTIVE CENTER
// ============================================================

void updateAdaptiveCenter(
    float gazeX
)
{
    if (!gazeCalibrated)
        return;

#if ADAPTIVE_CENTER_ENABLED

    float difference =
        fabsf(
            gazeX -
            calibratedCenterX
        );


    // فقط وقتی gaze نزدیک مرکز است
    if (
        difference <=
        GAZE_CENTER_RANGE
    )
    {
        calibratedCenterX =
            calibratedCenterX *
            (1.0f -
             ADAPTIVE_CENTER_ALPHA)
            +
            gazeX *
            ADAPTIVE_CENTER_ALPHA;


        calibratedCenterX =
            constrain(
                calibratedCenterX,
                CENTER_MIN,
                CENTER_MAX
            );
    }

#endif
}


// ============================================================
// COMMAND
// ============================================================

Command getCommand(
    float gazeX
)
{
    if (!gazeCalibrated)
        return CMD_NONE;


    float diff =
        gazeX -
        calibratedCenterX;


    // --------------------------------------------------------
    // نگاه چپ
    // --------------------------------------------------------

    if (diff < 0)
    {
        float amount =
            -diff;


        if (
            amount <
            TURN_LEVEL_1
        )
        {
            return CMD_FORWARD;
        }


        if (
            amount <
            TURN_LEVEL_2
        )
        {
            return CMD_LEFT_1;
        }


        if (
            amount <
            TURN_LEVEL_2 +
            TURN_LEVEL_1
        )
        {
            return CMD_LEFT_2;
        }


        return CMD_LEFT_3;
    }


    // --------------------------------------------------------
    // نگاه راست
    // --------------------------------------------------------

    else
    {
        float amount =
            diff;


        if (
            amount <
            TURN_LEVEL_1
        )
        {
            return CMD_FORWARD;
        }


        if (
            amount <
            TURN_LEVEL_2
        )
        {
            return CMD_RIGHT_1;
        }


        if (
            amount <
            TURN_LEVEL_2 +
            TURN_LEVEL_1
        )
        {
            return CMD_RIGHT_2;
        }


        return CMD_RIGHT_3;
    }
}


// ============================================================
// CHECK ROTATION COMMAND
// ============================================================

bool isRotationCommand(
    Command c
)
{
    return
        c == CMD_LEFT_1 ||
        c == CMD_LEFT_2 ||
        c == CMD_LEFT_3 ||
        c == CMD_RIGHT_1 ||
        c == CMD_RIGHT_2 ||
        c == CMD_RIGHT_3;
}


// ============================================================
// ESP-NOW SEND
// ============================================================

void sendCommand(
    Command command,
    float x,
    float y
)
{
    ControlPacket packet;

    packet.command =
        (uint8_t)command;

    packet.x = x;
    packet.y = y;


    esp_now_send(
        carAddress,
        (uint8_t *)&packet,
        sizeof(packet)
    );
}


// ============================================================
// MOTOR COMMAND UPDATE
// ============================================================

void updateMotorCommand(
    Command command,
    float gazeX,
    float gazeY
)
{
    unsigned long now =
        millis();


    // --------------------------------------------------------
    // اگر فرمان چرخش است:
    //
    // فقط هنگام تغییر ارسال شود.
    //
    // این خیلی مهم است چون ESP8266 بعد از دریافت
    // LEFT_2 مثلاً خودش حدود 200ms می‌چرخد و STOP می‌کند.
    // --------------------------------------------------------

    if (
        isRotationCommand(command)
    )
    {
        if (
            command != lastCommand
        )
        {
            sendCommand(
                command,
                gazeX,
                gazeY
            );

            lastCommand =
                command;

            lastCommandTime =
                now;
        }

        return;
    }


    // --------------------------------------------------------
    // FORWARD
    // --------------------------------------------------------

    if (
        command ==
        CMD_FORWARD
    )
    {
        if (
            lastCommand !=
                CMD_FORWARD ||
            now -
                lastCommandTime >=
                FORWARD_COMMAND_INTERVAL
        )
        {
            sendCommand(
                command,
                gazeX,
                gazeY
            );

            lastCommand =
                command;

            lastCommandTime =
                now;
        }

        return;
    }


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    if (
        command ==
        CMD_NONE
    )
    {
        if (
            lastCommand !=
            CMD_NONE
        )
        {
            sendCommand(
                CMD_NONE,
                gazeX,
                gazeY
            );

            lastCommand =
                CMD_NONE;

            lastCommandTime =
                now;
        }
    }
}


// ============================================================
// BASE64 IMAGE
// ============================================================

void sendImageBase64(
    camera_fb_t *fb,
    float gazeX,
    float gazeY,
    int lx,
    int ly,
    int rx,
    int ry,
    float plx,
    float ply,
    float prx,
    float pry,
    const char *cmd
)
{
    static uint8_t *smallImage =
        nullptr;


    if (!smallImage)
    {
        smallImage =
            (uint8_t *)
            heap_caps_malloc(
                SMALL_W *
                SMALL_H,
                MALLOC_CAP_SPIRAM
            );
    }


    for (int y = 0;
         y < SMALL_H;
         y++)
    {
        for (int x = 0;
             x < SMALL_W;
             x++)
        {
            smallImage[
                y * SMALL_W + x
            ] =
                fb->buf[
                    (y * 2) *
                    IMG_W +
                    (x * 2)
                ];
        }
    }


    size_t outputLength =
        4 *
        (
            (SMALL_W *
             SMALL_H +
             2) / 3
        );


    static unsigned char *encoded =
        nullptr;


    if (!encoded)
    {
        encoded =
            (unsigned char *)
            heap_caps_malloc(
                outputLength + 8,
                MALLOC_CAP_SPIRAM
            );
    }


    size_t actualLength = 0;


    int ret =
        mbedtls_base64_encode(
            encoded,
            outputLength + 1,
            &actualLength,
            smallImage,
            SMALL_W * SMALL_H
        );


    if (ret != 0)
    {
        Serial.println(
            "BASE64 ERROR"
        );

        return;
    }


    Serial.println("FRAME");

    Serial.printf(
        "%d %d\n",
        SMALL_W,
        SMALL_H
    );


    const int chunk = 256;


    for (
        size_t i = 0;
        i < actualLength;
        i += chunk
    )
    {
        int n =
            min(
                chunk,
                (int)(
                    actualLength -
                    i
                )
            );

        Serial.write(
            encoded + i,
            n
        );
    }


    Serial.println();

    Serial.println("DATA");


    Serial.printf(
        "L %d %d\n",
        lx,
        ly
    );

    Serial.printf(
        "R %d %d\n",
        rx,
        ry
    );

    Serial.printf(
        "PL %.1f %.1f\n",
        plx,
        ply
    );

    Serial.printf(
        "PR %.1f %.1f\n",
        prx,
        pry
    );

    Serial.printf(
        "GAZE %.3f %.3f\n",
        gazeX,
        gazeY
    );

    Serial.printf(
        "CENTER %.3f\n",
        calibratedCenterX
    );

    Serial.printf(
        "CAL %d/%d\n",
        calibrationCount,
        CALIBRATION_SAMPLES
    );

    Serial.printf(
        "CMD %s\n",
        cmd
    );

    Serial.println("END");
}


// ============================================================
// TASKS
// ============================================================

void leftTask(
    void *parameter
)
{
    while (true)
    {
        xSemaphoreTake(
            leftStartSemaphore,
            portMAX_DELAY
        );

        detectLeft();

        xSemaphoreGive(
            leftDoneSemaphore
        );
    }
}


void rightTask(
    void *parameter
)
{
    while (true)
    {
        xSemaphoreTake(
            rightStartSemaphore,
            portMAX_DELAY
        );

        detectRight();

        xSemaphoreGive(
            rightDoneSemaphore
        );
    }
}


// ============================================================
// COMMAND TEXT
// ============================================================

const char *commandText(
    Command c
)
{
    switch (c)
    {
        case CMD_LEFT_1:
            return "LEFT_1";

        case CMD_LEFT_2:
            return "LEFT_2";

        case CMD_LEFT_3:
            return "LEFT_3";

        case CMD_RIGHT_1:
            return "RIGHT_1";

        case CMD_RIGHT_2:
            return "RIGHT_2";

        case CMD_RIGHT_3:
            return "RIGHT_3";

        case CMD_FORWARD:
            return "FORWARD";

        default:
            return "STOP";
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(921600);

    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "ESP32 EYE TRACKER"
    );

    Serial.println(
        "CALIBRATED GAZE + HARD PUPIL FILTER"
    );

    Serial.println(
        "================================"
    );


    allocateMemory();


    if (!initCamera())
    {
        while (1)
            delay(1000);
    }


    WiFi.mode(
        WIFI_STA
    );

    WiFi.disconnect();


    if (
        esp_now_init() != ESP_OK
    )
    {
        Serial.println(
            "ESP-NOW INIT ERROR"
        );

        while (1)
            delay(1000);
    }


    esp_now_peer_info_t peerInfo =
        {};


    memcpy(
        peerInfo.peer_addr,
        carAddress,
        6
    );


    peerInfo.channel = 0;

    peerInfo.encrypt = false;


    if (
        esp_now_add_peer(
            &peerInfo
        ) != ESP_OK
    )
    {
        Serial.println(
            "ESP-NOW PEER ERROR"
        );
    }


    leftStartSemaphore =
        xSemaphoreCreateBinary();

    rightStartSemaphore =
        xSemaphoreCreateBinary();

    leftDoneSemaphore =
        xSemaphoreCreateBinary();

    rightDoneSemaphore =
        xSemaphoreCreateBinary();


    xTaskCreatePinnedToCore(
        leftTask,
        "LeftEye",
        8192,
        nullptr,
        1,
        &leftTaskHandle,
        0
    );


    xTaskCreatePinnedToCore(
        rightTask,
        "RightEye",
        8192,
        nullptr,
        1,
        &rightTaskHandle,
        1
    );


    resetCalibration();


    Serial.println(
        "SYSTEM READY"
    );

    Serial.println(
        "LOOK STRAIGHT FOR CALIBRATION"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    camera_fb_t *fb =
        esp_camera_fb_get();


    if (!fb)
    {
        Serial.println(
            "FRAME FAILED"
        );


        sendCommand(
            CMD_NONE,
            lastGazeX,
            lastGazeY
        );


        lastCommand =
            CMD_NONE;


        delay(10);

        return;
    }


    // ========================================================
    // PREPROCESS
    // ========================================================

    copyFrame(fb);

    preprocess();


    bool eyesFound = false;

    bool pupilValid = false;


    // ========================================================
    // INITIAL DETECTION
    // ========================================================

    if (!tracking)
    {
        xSemaphoreGive(
            leftStartSemaphore
        );

        xSemaphoreGive(
            rightStartSemaphore
        );


        xSemaphoreTake(
            leftDoneSemaphore,
            portMAX_DELAY
        );

        xSemaphoreTake(
            rightDoneSemaphore,
            portMAX_DELAY
        );


        bool found =
            selectBestEyePair(
                false
            );


        if (found)
        {
            filterInitialized =
                false;


            updateTrackingFilter();


            initializeTemplates();


            tracking = true;


            trackMisses = 0;

            trackFrameCounter = 0;


            eyesFound = true;


            // calibration از اول شروع می‌شود
            resetCalibration();


            Serial.println(
                ">>> EYES LOCKED <<<"
            );
        }
    }


    // ========================================================
    // TRACKING
    // ========================================================

    else
    {
        bool tracked =
            trackEyes();


        if (tracked)
        {
            trackMisses = 0;

            updateTrackingFilter();


            eyesFound = true;

            trackFrameCounter++;
        }
        else
        {
            trackMisses++;

            eyesFound = false;


            Serial.printf(
                "TRACK LOST %d/%d\n",
                trackMisses,
                TRACK_MAX_MISSES
            );


            if (
                trackMisses >=
                TRACK_MAX_MISSES
            )
            {
                Serial.println(
                    ">>> TRACKING LOST - STOP <<<"
                );


                sendCommand(
                    CMD_NONE,
                    lastGazeX,
                    lastGazeY
                );


                lastCommand =
                    CMD_NONE;

                lastCommandTime =
                    millis();


                tracking = false;

                templatesInitialized =
                    false;

                filterInitialized =
                    false;


                trackMisses = 0;

                trackFrameCounter = 0;


                resetCalibration();
            }
        }
    }


    // ========================================================
    // GAZE
    // ========================================================

    float gazeX =
        lastGazeX;

    float gazeY =
        lastGazeY;


    Command command =
        CMD_NONE;


    if (eyesFound)
    {
        pupilValid =
            calculateGaze(
                gazeX,
                gazeY
            );


        if (pupilValid)
        {
            // -----------------------------------------------
            // Calibration
            // -----------------------------------------------

            if (!gazeCalibrated)
            {
                updateCalibration(
                    gazeX
                );

                command =
                    CMD_NONE;
            }


            // -----------------------------------------------
            // Normal operation
            // -----------------------------------------------

            else
            {
                updateAdaptiveCenter(
                    gazeX
                );


                command =
                    getCommand(
                        gazeX
                    );
            }
        }
        else
        {
            command =
                CMD_NONE;
        }
    }
    else
    {
        command =
            CMD_NONE;
    }


    // ========================================================
    // TEMPLATE UPDATE
    //
    // خیلی مهم:
    //
    // template فقط وقتی update می‌شود که مردمک معتبر باشد.
    // بنابراین اگر tracker به ابرو برود، template با ابرو
    // آلوده نمی‌شود.
    // ========================================================

    if (
        eyesFound &&
        pupilValid &&
        gazeCalibrated
    )
    {
        if (
            trackFrameCounter %
            TEMPLATE_UPDATE_RATE == 0
        )
        {
            updateTemplate(
                bestLeftEye,
                leftTemplate
            );


            updateTemplate(
                bestRightEye,
                rightTemplate
            );
        }
    }


    // ========================================================
    // MOTOR COMMAND
    // ========================================================

    updateMotorCommand(
        command,
        gazeX,
        gazeY
    );


    // ========================================================
    // SERIAL DEBUG
    // ========================================================

    if (eyesFound)
    {
        Serial.printf(
            "TRACK L(%d,%d) R(%d,%d)",
            bestLeftEye.x,
            bestLeftEye.y,
            bestRightEye.x,
            bestRightEye.y
        );


        if (
            leftPupil.valid &&
            rightPupil.valid
        )
        {
            Serial.printf(
                " | PUPIL L(%.1f,%.1f) R(%.1f,%.1f)",
                leftPupil.x,
                leftPupil.y,
                rightPupil.x,
                rightPupil.y
            );


            Serial.printf(
                " | GAZE %.3f",
                gazeX
            );


            Serial.printf(
                " | CENTER %.3f",
                calibratedCenterX
            );


            Serial.printf(
                " | CMD %s",
                commandText(command)
            );


            Serial.printf(
                " | CAL %d/%d",
                calibrationCount,
                CALIBRATION_SAMPLES
            );
        }
        else
        {
            Serial.print(
                " | PUPIL INVALID"
            );

            Serial.print(
                " | CMD STOP"
            );
        }


        Serial.println();
    }
    else
    {
        Serial.println(
            "TRACKING LOST -> STOP / SEARCH"
        );
    }


    // ========================================================
    // BASE64 IMAGE
    // ========================================================

    sendImageBase64(
        fb,

        gazeX,
        gazeY,

        eyesFound ?
            bestLeftEye.x : -1,

        eyesFound ?
            bestLeftEye.y : -1,

        eyesFound ?
            bestRightEye.x : -1,

        eyesFound ?
            bestRightEye.y : -1,

        leftPupil.valid ?
            leftPupil.x : -1,

        leftPupil.valid ?
            leftPupil.y : -1,

        rightPupil.valid ?
            rightPupil.x : -1,

        rightPupil.valid ?
            rightPupil.y : -1,

        commandText(command)
    );


    // ========================================================
    // RETURN FRAME
    // ========================================================

    esp_camera_fb_return(fb);


    delay(5);
}