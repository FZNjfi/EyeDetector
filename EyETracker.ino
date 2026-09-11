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
// COMMANDS to ESP8266
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

#define FORWARD_COMMAND_INTERVAL 100

unsigned long lastCommandTime = 0;


// ============================================================
// LOST TRACKING
// ============================================================

#define TRACK_MAX_MISSES 14

int trackMisses = 0;


// ============================================================
// FLASH ASSIST - PUPIL RECOVERY
// ============================================================

#define FLASH_GPIO 4

#define FLASH_PULSE_MS 60

#define PUPIL_LOST_BEFORE_FLASH 2

#define FLASH_COOLDOWN_MS 500

int pupilMisses = 0;
#define MAX_INVALID_PUPIL_FRAMES 3

int leftInvalidPupilFrames = 0;
int rightInvalidPupilFrames = 0;


// ============================================================
// GAZE CALIBRATION
// ============================================================

#define CALIBRATION_SAMPLES 0

#define GAZE_CENTER_RANGE 0.10f

bool gazeCalibrated = false;

int calibrationCount = 0;
float calibrationSum = 0.0f;

float calibratedCenterX = 0.50f;


// ============================================================
// ADAPTIVE CENTER
// ============================================================

#define ADAPTIVE_CENTER_ENABLED true
#define ADAPTIVE_CENTER_ALPHA 0.015f

#define CENTER_MIN 0.20f
#define CENTER_MAX 0.80f


// ============================================================
// GAZE COMMAND THRESHOLDS
// ============================================================

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

#define MIN_EYE_DISTANCE 40
#define MAX_EYE_DISTANCE 190

#define MAX_EYE_DY 10

#define MAX_SIZE_RATIO 2.2f

#define MIN_PAIR_SCORE 45.0f


// ============================================================
// TRACKING
// ============================================================

#define TRACK_SEARCH_X 35
#define TRACK_SEARCH_Y 12

#define TRACK_COARSE_STEP 4
#define TRACK_FINE_RADIUS 3
#define TRACK_FINE_STEP 1

#define TEMPLATE_W 32
#define TEMPLATE_H 20

#define TEMPLATE_HALF_W 16
#define TEMPLATE_HALF_H 10

#define TEMPLATE_UPDATE_RATE 5

#define MAX_TRACK_ERROR 75.0f

#define MAX_TRACK_VERTICAL_JUMP 12

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

    char rejectReason[40];
};


Pupil leftPupil;
Pupil rightPupil;

// ============================================================
// GAZE
// ============================================================

float lastGazeX = 0.5f;
float lastGazeY = 0.5f;

// ============================================================
// PARALLEL TRACKING
// ============================================================

EyeCandidate leftTrackInput;
EyeCandidate rightTrackInput;

EyeCandidate leftTrackResult;
EyeCandidate rightTrackResult;

volatile bool leftTrackOK = false;
volatile bool rightTrackOK = false;
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
// IRIS GEOMETRY CHECK
// ============================================================

#define IRIS_RAYS 8

#define IRIS_RADIUS_MIN 7
#define IRIS_RADIUS_MAX 24

#define IRIS_MIN_BRIGHTNESS_JUMP 15

#define IRIS_MIN_VALID_RAYS 5

#define IRIS_MAX_RADIUS_DEVIATION 0.38f

#define IRIS_MIN_GEOMETRY_SCORE 0.55f

// ============================================================
// TASK RESULT
// ============================================================

EyeCandidate leftCandidates[12];
EyeCandidate rightCandidates[12];

volatile int leftCandidateCount = 0;
volatile int rightCandidateCount = 0;

char pupilRejectReason[40] = "";
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
    uint8_t *templ,
    float cutoff
)
{
    long totalError = 0;
    int samples = 0;

    const int TOTAL_SAMPLES =
        (TEMPLATE_W / 2) * (TEMPLATE_H / 2);

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

            totalError +=
                abs(current - reference);

            samples++;

            // Early exit
            if (samples > 0)
            {
                float currentAverage =
                    (float)totalError /
                    (float)TOTAL_SAMPLES;

                if (currentAverage >= cutoff)
                {
                    return 999999.0f;
                }
            }
        }
    }

    if (samples == 0)
        return 999999.0f;

    return
        (float)totalError /
        (float)samples;
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


    // ========================================================
    // PHASE 1 - COARSE SEARCH
    // ========================================================

    for (
        int dy = -TRACK_SEARCH_Y;
        dy <= TRACK_SEARCH_Y;
        dy += TRACK_COARSE_STEP
    )
    {
        for (
            int dx = -TRACK_SEARCH_X;
            dx <= TRACK_SEARCH_X;
            dx += TRACK_COARSE_STEP
        )
        {
            if (
                abs(dy) >
                MAX_TRACK_VERTICAL_JUMP
            )
                continue;

            if (
                abs(dx) >
                MAX_TRACK_HORIZONTAL_JUMP
            )
                continue;


            int x =
                oldX + dx;

            int y =
                oldY + dy;


            if (
                x < TEMPLATE_HALF_W ||
                x >= IMG_W - TEMPLATE_HALF_W ||
                y < TEMPLATE_HALF_H ||
                y >= IMG_H - TEMPLATE_HALF_H
            )
                continue;


            float error =
                templateError(
                    x,
                    y,
                    templ,
                    bestError
                );


            if (
                error <
                bestError
            )
            {
                bestError = error;

                bestX = x;
                bestY = y;
            }
        }
    }


    // ========================================================
    // NO GOOD COARSE RESULT
    // ========================================================

    if (
        bestError >
        MAX_TRACK_ERROR
    )
    {
        return false;
    }


    // ========================================================
    // PHASE 2 - FINE SEARCH
    // ========================================================

    float coarseBestError =
        bestError;


    for (
        int dy = -TRACK_FINE_RADIUS;
        dy <= TRACK_FINE_RADIUS;
        dy += TRACK_FINE_STEP
    )
    {
        for (
            int dx = -TRACK_FINE_RADIUS;
            dx <= TRACK_FINE_RADIUS;
            dx += TRACK_FINE_STEP
        )
        {
            int x =
                bestX + dx;

            int y =
                bestY + dy;


            if (
                x < TEMPLATE_HALF_W ||
                x >= IMG_W - TEMPLATE_HALF_W ||
                y < TEMPLATE_HALF_H ||
                y >= IMG_H - TEMPLATE_HALF_H
            )
                continue;


            float error =
                templateError(
                    x,
                    y,
                    templ,
                    bestError
                );


            if (
                error <
                bestError
            )
            {
                bestError = error;

                bestX = x;
                bestY = y;
            }
        }
    }


    // ========================================================
    // FINAL VALIDATION
    // ========================================================

    if (
        bestError >
        MAX_TRACK_ERROR
    )
    {
        return false;
    }


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
    // Reject very thin horizontal objects
    // such as eyebrows
    if (h < 8)
        return result;
    
    int centerY = (minY + maxY) / 2;

    float bandHeight =
        (float)(EYE_BAND_Y_MAX - EYE_BAND_Y_MIN);

    float relativeBandY =
        (centerY - EYE_BAND_Y_MIN) / bandHeight;

    bool inEyebrowZone =
        relativeBandY < 0.32f; 

    bool eyebrowShaped =
        (aspect > 2.6f) &&
        ((float)h / w < 0.35f);

    if (inEyebrowZone && eyebrowShaped)
        return result;


    if (aspect > 2.2f && density > 0.55f)
        return result;


    // Reject very wide and thin objects
    if (aspect > 5.0f && h < 14)
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
    // ============================================================
    // REJECT OVERLAPPING EYE BOXES
    // ============================================================

    int leftBoxRight =
        l.x + l.w / 2;

    int rightBoxLeft =
        r.x - r.w / 2;

    if (leftBoxRight >= rightBoxLeft)
    {
        return -100000;
    }

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


    // ========================================================
    // COPY INPUT
    // ========================================================

    leftTrackInput =
        bestLeftEye;

    rightTrackInput =
        bestRightEye;


    leftTrackOK = false;
    rightTrackOK = false;


    // ========================================================
    // START BOTH CORES
    // ========================================================

    xSemaphoreGive(
        leftStartSemaphore
    );

    xSemaphoreGive(
        rightStartSemaphore
    );


    // ========================================================
    // WAIT FOR BOTH
    // ========================================================

    xSemaphoreTake(
        leftDoneSemaphore,
        portMAX_DELAY
    );

    xSemaphoreTake(
        rightDoneSemaphore,
        portMAX_DELAY
    );


    if (
        !leftTrackOK ||
        !rightTrackOK
    )
    {
        return false;
    }


    EyeCandidate newLeft =
        leftTrackResult;

    EyeCandidate newRight =
        rightTrackResult;


    // ========================================================
    // PAIR VALIDATION
    // ========================================================

    int dx =
        newRight.x -
        newLeft.x;

    int dy =
        abs(
            newRight.y -
            newLeft.y
        );


    if (
        dx < MIN_EYE_DISTANCE
    )
        return false;


    if (
        dx > MAX_EYE_DISTANCE
    )
        return false;


    if (
        dy > MAX_EYE_DY
    )
        return false;


    if (
        dx < dy * 3
    )
        return false;


    // ========================================================
    // HORIZONTAL CONTINUITY
    // ========================================================

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


    // ========================================================
    // ACCEPT
    // ========================================================

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

#define PUPIL_HALF_W 28
#define PUPIL_TOP_OFFSET 5
#define PUPIL_BOTTOM_OFFSET 11

#define SCLERA_MIN_BRIGHTNESS 90.0f
#define SCLERA_MIN_CONTRAST   45.0f

#define Eyelash_REJECT_RATIO 0.12f

#define PUPIL_MIN_PIXELS 2
#define PUPIL_MAX_PIXELS 350

#define PUPIL_MIN_DENSITY 0.015f
#define PUPIL_MAX_DENSITY 0.55f

#define MAX_PUPIL_JUMP_X 18.0f
#define MAX_PUPIL_JUMP_Y 8.0f

#define PUPIL_MIN_CIRCULARITY 0.40f
#define PUPIL_MAX_CIRCULARITY 1.05f
// ============================================================
// LIGHTWEIGHT IRIS GEOMETRY CHECK
// ============================================================

bool checkIrisGeometry(
    float cx,
    float cy,
    float &geometryScore
)
{
    geometryScore = 0.0f;

    if (
        cx < IRIS_RADIUS_MAX ||
        cx >= IMG_W - IRIS_RADIUS_MAX ||
        cy < IRIS_RADIUS_MAX ||
        cy >= IMG_H - IRIS_RADIUS_MAX
    )
    {
        return false;
    }


    const float angles[IRIS_RAYS] =
    {
        0.0f,
        0.7854f,
        1.5708f,
        2.3562f,
        3.1416f,
        3.9270f,
        4.7124f,
        5.4978f
    };


    int validRays = 0;

    float radiusSum = 0.0f;

    float minRadius = 999.0f;
    float maxRadius = 0.0f;


    for (int i = 0; i < IRIS_RAYS; i++)
    {
        float dx =
            cosf(angles[i]);

        float dy =
            sinf(angles[i]);


        float bestJump = 0.0f;
        int bestRadius = 0;

        for (
            int r = IRIS_RADIUS_MIN;
            r <= IRIS_RADIUS_MAX;
            r += 2
        )
        {
            int x1 =
                (int)(cx + dx * r);

            int y1 =
                (int)(cy + dy * r);

            int x2 =
                (int)(cx + dx * (r + 2));

            int y2 =
                (int)(cy + dy * (r + 2));


            if (
                x1 < 1 ||
                x1 >= IMG_W - 1 ||
                y1 < 1 ||
                y1 >= IMG_H - 1 ||
                x2 < 1 ||
                x2 >= IMG_W - 1 ||
                y2 < 1 ||
                y2 >= IMG_H - 1
            )
            {
                continue;
            }


            int inside =
                workImage[
                    y1 * IMG_W + x1
                ];


            int outside =
                workImage[
                    y2 * IMG_W + x2
                ];


            float jump =
                (float)outside -
                (float)inside;


            if (jump > bestJump)
            {
                bestJump = jump;

                bestRadius = r;
            }
        }

        if (
            bestRadius > 0 &&
            bestJump >=
                IRIS_MIN_BRIGHTNESS_JUMP
        )
        {
            validRays++;

            radiusSum += bestRadius;


            if (bestRadius < minRadius)
                minRadius = bestRadius;

            if (bestRadius > maxRadius)
                maxRadius = bestRadius;
        }
    }


    if (
        validRays <
        IRIS_MIN_VALID_RAYS
    )
    {
        return false;
    }

    float meanRadius =
        radiusSum /
        validRays;


    if (meanRadius <= 0)
        return false;

    float deviation =
        (maxRadius - minRadius) /
        meanRadius;


    if (
        deviation >
        IRIS_MAX_RADIUS_DEVIATION
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Geometry score
    // --------------------------------------------------------

    float rayScore =
        (float)validRays /
        (float)IRIS_RAYS;


    float circleScore =
        1.0f -
        deviation /
        IRIS_MAX_RADIUS_DEVIATION;


    if (circleScore < 0)
        circleScore = 0;


    geometryScore =
        rayScore *
        circleScore;


    if (
        geometryScore <
        IRIS_MIN_GEOMETRY_SCORE
    )
    {
        return false;
    }


    return true;
}
Pupil findPupil(
    EyeCandidate &eye,
    Pupil &previousPupil
)
{
    Pupil result;

    result.x = -1;
    result.y = -1;
    result.valid = false;
    result.confidence = 0;
    strcpy(result.rejectReason, "UNKNOWN");


    // --------------------------------------------------------
    // ROI Pupil
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
    {
        strcpy(result.rejectReason, "BAD_ROI");
        return result;
    }


    // --------------------------------------------------------
    // Darkest Pixel
    // --------------------------------------------------------

    int minValue = 255;

    for (int y = y1; y <= y2; y++)
    {
        for (int x = x1; x <= x2; x++)
        {
            int v =
                grayImage[y * IMG_W + x];

            if (v < minValue)
                minValue = v;
        }
    }


    // --------------------------------------------------------
    // Threshold
    // --------------------------------------------------------

    int threshold = minValue + 22;

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
    // Find dark pixels
    // --------------------------------------------------------

    for (int y = y1; y <= y2; y++)
    {
        for (int x = x1; x <= x2; x++)
        {
            int v =
                grayImage[y * IMG_W + x];

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
    // Number of dark pixels
    // --------------------------------------------------------

    if (
        count < PUPIL_MIN_PIXELS ||
        count > PUPIL_MAX_PIXELS
    )
    {
        snprintf(
            result.rejectReason,
            sizeof(result.rejectReason),
            "COUNT %ld",
            count
        );

        return result;
    }


    // --------------------------------------------------------
    // Pupil center
    // --------------------------------------------------------

    float px =
        (float)sumX / count;

    float py =
        (float)sumY / count;


    // --------------------------------------------------------
    // Relative Y
    // --------------------------------------------------------

    float relativeY =
        (py - y1) /
        (float)(y2 - y1);


    if (
        relativeY <
        Eyelash_REJECT_RATIO
    )
    {
        snprintf(
            result.rejectReason,
            sizeof(result.rejectReason),
            "RELATIVE_Y %.2f",
            relativeY
        );

        return result;
    }


    // --------------------------------------------------------
    // Candidate shape
    // --------------------------------------------------------

    int pw =
        maxPX - minPX + 1;

    int ph =
        maxPY - minPY + 1;


    if (pw <= 0 || ph <= 0)
    {
        strcpy(
            result.rejectReason,
            "BAD_SHAPE"
        );

        return result;
    }


    // --------------------------------------------------------
    // Density
    // --------------------------------------------------------

    float density =
        (float)count /
        (float)(pw * ph);


    if (
        density < PUPIL_MIN_DENSITY ||
        density > PUPIL_MAX_DENSITY
    )
    {
        snprintf(
            result.rejectReason,
            sizeof(result.rejectReason),
            "DENSITY %.3f",
            density
        );

        return result;
    }


    // --------------------------------------------------------
    // Aspect ratio
    // --------------------------------------------------------

    float aspect =
        (float)pw / ph;


    if (
        aspect > 3.4f ||
        aspect < 0.5f
    )
    {
        snprintf(
            result.rejectReason,
            sizeof(result.rejectReason),
            "ASPECT %.2f",
            aspect
        );

        return result;
    }


    // --------------------------------------------------------
    // Circularity
    // --------------------------------------------------------

    float expectedEllipseArea =
        3.14159f *
        (pw / 2.0f) *
        (ph / 2.0f);


    float circularity =
        (float)count /
        expectedEllipseArea;


    if (
        circularity < PUPIL_MIN_CIRCULARITY ||
        circularity > PUPIL_MAX_CIRCULARITY
    )
    {
        snprintf(
            result.rejectReason,
            sizeof(result.rejectReason),
            "CIRCULARITY %.2f",
            circularity
        );

        return result;
    }

    // --------------------------------------------------------
    // Pupillary movement
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
            snprintf(
                result.rejectReason,
                sizeof(result.rejectReason),
                "JUMP %.1f %.1f",
                dx,
                dy
            );

            return result;
        }
    }


    // --------------------------------------------------------
    // Valid pupil
    // --------------------------------------------------------

    result.x = px;
    result.y = py;

    result.valid = true;

    result.confidence = 1.0f;

    strcpy(
        result.rejectReason,
        "OK"
    );


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

// ============================================================
// TEMPLATE VALIDATION BY PUPIL
// ============================================================

bool updateTemplateValidity()
{
    bool resetTracking = false;


    // ========================================================
    // LEFT PUPIL
    // ========================================================

    if (leftPupil.valid)
    {
        leftInvalidPupilFrames = 0;
    }
    else
    {
        leftInvalidPupilFrames++;

        if (
            leftInvalidPupilFrames >=
            MAX_INVALID_PUPIL_FRAMES
        )
        {
            Serial.println(
                ">>> LEFT TEMPLATE INVALID <<<"
            );

            resetTracking = true;
        }
    }


    // ========================================================
    // RIGHT PUPIL
    // ========================================================

    if (rightPupil.valid)
    {
        rightInvalidPupilFrames = 0;
    }
    else
    {
        rightInvalidPupilFrames++;

        if (
            rightInvalidPupilFrames >=
            MAX_INVALID_PUPIL_FRAMES
        )
        {
            Serial.println(
                ">>> RIGHT TEMPLATE INVALID <<<"
            );

            resetTracking = true;
        }
    }


    // ========================================================
    // RESET
    // ========================================================

    if (resetTracking)
    {
        tracking = false;

        templatesInitialized =
            false;

        filterInitialized =
            false;

        trackMisses = 0;

        trackFrameCounter = 0;

        leftInvalidPupilFrames = 0;
        rightInvalidPupilFrames = 0;

        return true;
    }


    return false;
}

void updateCalibration(
    float gazeX
)
{
    if (gazeCalibrated)
        return;

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

    if (
        isRotationCommand(command)
    )
    {
//        if (
//            command != lastCommand
//        )
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
    const char *cmd,
    const char *leftReason,
    const char *rightReason
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
        "PERR_L %s\n",
        leftReason
    );

    Serial.printf(
        "PERR_R %s\n",
        rightReason
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


        if (tracking)
        {
            EyeCandidate localEye =
                leftTrackInput;

            leftTrackOK =
                trackOneEye(
                    localEye,
                    leftTemplate
                );

            leftTrackResult =
                localEye;
        }
        else
        {
            detectLeft();
        }


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


        if (tracking)
        {
            EyeCandidate localEye =
                rightTrackInput;

            rightTrackOK =
                trackOneEye(
                    localEye,
                    rightTemplate
                );

            rightTrackResult =
                localEye;
        }
        else
        {
            detectRight();
        }


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

    // ========================================================
    // FLASH INIT
    // ========================================================
    ledcSetup(0, 5000, 8);
    ledcAttachPin(FLASH_GPIO, 0);

    flashOn(0);
    
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
// FLASH CONTROL
// ============================================================

void flashOn(uint8_t brightness)
{
    ledcWrite(0, brightness);

    Serial.println(">>> FLASH ON - PUPIL RECOVERY <<<");
}


void flashOff()
{
    digitalWrite(FLASH_GPIO, LOW);

    Serial.println(">>> FLASH OFF <<<");
}


// ============================================================
// FLASH RECOVERY
// ============================================================

void updateFlashRecovery(bool pupilValid)
{
    unsigned long now = millis();

    if (pupilValid)
    {
        pupilMisses = 0;
        return;
    }

    pupilMisses++;


    if (
        pupilMisses <
        PUPIL_LOST_BEFORE_FLASH
    )
    {
        return;
    }

    pupilMisses = 0;

    flashOn(60);
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // VARIABLES
    // ========================================================

    bool eyesFound = false;
    bool pupilValid = false;
    bool templateReset = false;

    float gazeX = lastGazeX;
    float gazeY = lastGazeY;

    Command command = CMD_NONE;


    // ========================================================
    // GET FRAME
    // ========================================================

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


    // ========================================================
    // INITIAL DETECTION
    // ========================================================

    if (!tracking)
    {
        // ----------------------------------------------------
        // Ask left/right tasks to perform full eye detection
        // ----------------------------------------------------

        xSemaphoreGive(
            leftStartSemaphore
        );

        xSemaphoreGive(
            rightStartSemaphore
        );


        // ----------------------------------------------------
        // Wait for both eyes
        // ----------------------------------------------------

        xSemaphoreTake(
            leftDoneSemaphore,
            portMAX_DELAY
        );

        xSemaphoreTake(
            rightDoneSemaphore,
            portMAX_DELAY
        );


        // ----------------------------------------------------
        // Select valid left/right eye pair
        // ----------------------------------------------------

        bool found =
            selectBestEyePair(
                false
            );


        if (found)
        {
            // ------------------------------------------------
            // New eye lock
            // ------------------------------------------------

            filterInitialized =
                false;


            updateTrackingFilter();


            initializeTemplates();


            tracking =
                true;


            trackMisses =
                0;


            trackFrameCounter =
                0;


            eyesFound =
                true;


            resetCalibration();


            // Reset invalid-pupil counters
            leftInvalidPupilFrames =
                0;

            rightInvalidPupilFrames =
                0;


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
            // ------------------------------------------------
            // Tracking successful
            // ------------------------------------------------

            trackMisses =
                0;


            updateTrackingFilter();


            eyesFound =
                true;


            trackFrameCounter++;
        }
        else
        {
            // ------------------------------------------------
            // Tracking failed
            // ------------------------------------------------

            trackMisses++;


            eyesFound =
                false;


            Serial.printf(
                "TRACK LOST %d/%d\n",
                trackMisses,
                TRACK_MAX_MISSES
            );


            // ------------------------------------------------
            // Too many tracking failures
            // ------------------------------------------------

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


                tracking =
                    false;


                templatesInitialized =
                    false;


                filterInitialized =
                    false;


                trackMisses =
                    0;


                trackFrameCounter =
                    0;


                resetCalibration();


                // Reset pupil invalid counters
                leftInvalidPupilFrames =
                    0;

                rightInvalidPupilFrames =
                    0;
            }
        }
    }


    // ========================================================
    // GAZE / PUPIL
    // ========================================================

    if (eyesFound)
    {
        // ----------------------------------------------------
        // Calculate pupil + gaze
        // ----------------------------------------------------

        pupilValid =
            calculateGaze(
                gazeX,
                gazeY
            );


        // ----------------------------------------------------
        // Check repeated invalid pupil
        //
        // If one eye is invalid for 3 consecutive frames:
        // tracking + templates are completely reset.
        // ----------------------------------------------------

        templateReset =
            updateTemplateValidity();


        // ----------------------------------------------------
        // If template tracking became invalid, abandon
        // this frame and return to full eye detection.
        // ----------------------------------------------------

        if (templateReset)
        {
            eyesFound =
                false;


            pupilValid =
                false;


            command =
                CMD_NONE;


            Serial.println(
                ">>> PUPIL INVALID 3 FRAMES - RESET TRACKING <<<"
            );
        }


        // ====================================================
        // FLASH RECOVERY
        // ====================================================

        if (!templateReset)
        {
            updateFlashRecovery(
                pupilValid
            );
        }


        // ====================================================
        // NORMAL GAZE PROCESSING
        // ====================================================

        if (
            !templateReset &&
            pupilValid
        )
        {
            // ------------------------------------------------
            // Calibration
            // ------------------------------------------------

            if (!gazeCalibrated)
            {
                updateCalibration(
                    gazeX
                );


                command =
                    CMD_NONE;
            }


            // ------------------------------------------------
            // Normal operation
            // ------------------------------------------------

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
    // ========================================================

    if (
        eyesFound &&
        pupilValid &&
        gazeCalibrated &&
        !templateReset &&
        templatesInitialized
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

        commandText(command),

        leftPupil.rejectReason,

        rightPupil.rejectReason
    );


    // ========================================================
    // RETURN FRAME
    // ========================================================

    esp_camera_fb_return(
        fb
    );


    delay(5);
}
