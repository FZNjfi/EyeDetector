import serial
import base64
import numpy as np

from PIL import Image, ImageTk, ImageDraw, ImageFont

import tkinter as tk


# ============================================================
# SERIAL
# ============================================================

SERIAL_PORT = "COM7"
BAUD_RATE = 921600


ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=2
)


print("Connected:", SERIAL_PORT)


# ============================================================
# IMAGE
# ============================================================

WIDTH = 160
HEIGHT = 120

SCALE = 5

DISPLAY_WIDTH = WIDTH * SCALE
DISPLAY_HEIGHT = HEIGHT * SCALE


# ============================================================
# GUI
# ============================================================

root = tk.Tk()

root.title("ESP32 Eye Tracker")

root.geometry(
    f"{DISPLAY_WIDTH}x{DISPLAY_HEIGHT}"
)

root.resizable(
    False,
    False
)


label = tk.Label(root)

label.pack()


# ============================================================
# FONT
# ============================================================

try:
    font = ImageFont.truetype(
        "arial.ttf",
        8
    )
except:
    font = ImageFont.load_default()


# ============================================================
# ESP32 -> PYTHON
# ============================================================

def ex(x):
    return int(x / 2)


def ey(y):
    return int(y / 2)


# ============================================================
# DRAW SEARCH REGION
# ============================================================

def draw_search_regions(draw):

    # --------------------------------------------------------
    # LEFT
    # ESP32:
    # 20..160
    # 45..145
    # --------------------------------------------------------

    draw.rectangle(
        (
            ex(20),
            ey(45),
            ex(160),
            ey(145)
        ),
        outline=(255, 255, 0),
        width=1
    )


    # --------------------------------------------------------
    # RIGHT
    # --------------------------------------------------------

    draw.rectangle(
        (
            ex(160),
            ey(45),
            ex(300),
            ey(145)
        ),
        outline=(255, 255, 0),
        width=1
    )


# ============================================================
# DRAW EYE BOX
# ============================================================

def draw_eye_box(
    draw,
    x,
    y,
    name
):

    cx = ex(x)
    cy = ey(y)


    # همان محدوده findPupil()
    half_w = ex(32)
    half_h = ey(14)


    x1 = cx - half_w
    y1 = cy - half_h

    x2 = cx + half_w
    y2 = cy + half_h


    draw.rectangle(
        (
            x1,
            y1,
            x2,
            y2
        ),
        outline=(0, 255, 0),
        width=2
    )


    # center

    draw.line(
        (
            cx - 3,
            cy,
            cx + 3,
            cy
        ),
        fill=(0, 255, 255),
        width=1
    )


    draw.line(
        (
            cx,
            cy - 3,
            cx,
            cy + 3
        ),
        fill=(0, 255, 255),
        width=1
    )


    draw.text(
        (
            x1 + 2,
            y1 - 9
        ),
        name,
        fill=(0, 255, 0),
        font=font
    )


# ============================================================
# DRAW PUPIL
# ============================================================

def draw_pupil(
    draw,
    x,
    y
):

    cx = ex(x)
    cy = ey(y)

    r = 3


    draw.ellipse(
        (
            cx-r,
            cy-r,
            cx+r,
            cy+r
        ),
        outline=(255, 0, 0),
        width=2
    )


    draw.line(
        (
            cx-r-2,
            cy,
            cx+r+2,
            cy
        ),
        fill=(255, 0, 0),
        width=1
    )


    draw.line(
        (
            cx,
            cy-r-2,
            cx,
            cy+r+2
        ),
        fill=(255, 0, 0),
        width=1
    )


# ============================================================
# READ FRAME
# ============================================================

def read_frame():

    while True:

        line = ser.readline()

        if not line:
            return None

        line = line.decode(
            "ascii",
            errors="ignore"
        ).strip()


        if line == "FRAME":
            break


    # --------------------------------------------------------
    # size
    # --------------------------------------------------------

    line = ser.readline().decode(
        "ascii",
        errors="ignore"
    ).strip()


    try:

        width, height = map(
            int,
            line.split()
        )

    except:

        return None


    # --------------------------------------------------------
    # Base64
    # --------------------------------------------------------

    encoded = b""


    while True:

        line = ser.readline()

        if not line:
            return None


        if line.strip() == b"DATA":
            break


        encoded += line.strip()


    try:

        raw = base64.b64decode(
            encoded
        )

    except Exception as e:

        print(
            "Base64 error:",
            e
        )

        return None


    if len(raw) != width * height:

        print(
            "Invalid image:",
            len(raw),
            width * height
        )

        return None


    # --------------------------------------------------------
    # image
    # --------------------------------------------------------

    arr = np.frombuffer(
        raw,
        dtype=np.uint8
    )


    arr = arr.reshape(
        height,
        width
    )


    image = Image.fromarray(
        arr,
        mode="L"
    ).convert("RGB")


    # --------------------------------------------------------
    # variables
    # --------------------------------------------------------

    left_eye = None
    right_eye = None

    left_pupil = None
    right_pupil = None

    gaze_x = 0.5
    gaze_y = 0.5

    command = "STOP"


    # --------------------------------------------------------
    # DATA
    # --------------------------------------------------------

    while True:

        line = ser.readline().decode(
            "ascii",
            errors="ignore"
        ).strip()


        if line == "END":
            break


        p = line.split()


        if not p:
            continue


        if p[0] == "L":

            left_eye = (
                float(p[1]),
                float(p[2])
            )


        elif p[0] == "R":

            right_eye = (
                float(p[1]),
                float(p[2])
            )


        elif p[0] == "PL":

            x = float(p[1])
            y = float(p[2])

            if x >= 0 and y >= 0:

                left_pupil = (
                    x,
                    y
                )


        elif p[0] == "PR":

            x = float(p[1])
            y = float(p[2])

            if x >= 0 and y >= 0:

                right_pupil = (
                    x,
                    y
                )


        elif p[0] == "GAZE":

            gaze_x = float(p[1])
            gaze_y = float(p[2])


        elif p[0] == "CMD":

            command = p[1]


    return (
        image,
        left_eye,
        right_eye,
        left_pupil,
        right_pupil,
        gaze_x,
        gaze_y,
        command
    )


# ============================================================
# DRAW FRAME
# ============================================================

def draw_frame(data):

    (
        image,
        left_eye,
        right_eye,
        left_pupil,
        right_pupil,
        gaze_x,
        gaze_y,
        command
    ) = data


    draw = ImageDraw.Draw(
        image
    )


    # ========================================================
    # SEARCH AREAS
    # ========================================================

    draw_search_regions(
        draw
    )


    # ========================================================
    # EYE BOX
    # ========================================================

    if left_eye:

        draw_eye_box(
            draw,
            left_eye[0],
            left_eye[1],
            "LEFT"
        )


    if right_eye:

        draw_eye_box(
            draw,
            right_eye[0],
            right_eye[1],
            "RIGHT"
        )


    # ========================================================
    # EYE CONNECTION
    # ========================================================

    if left_eye and right_eye:

        draw.line(
            (
                ex(left_eye[0]),
                ey(left_eye[1]),
                ex(right_eye[0]),
                ey(right_eye[1])
            ),
            fill=(0, 255, 0),
            width=1
        )


    # ========================================================
    # PUPILS
    # ========================================================

    if left_pupil:

        draw_pupil(
            draw,
            left_pupil[0],
            left_pupil[1]
        )


    if right_pupil:

        draw_pupil(
            draw,
            right_pupil[0],
            right_pupil[1]
        )


    # ========================================================
    # INFO
    # ========================================================

    info = (
        f"X={gaze_x:.2f} "
        f"Y={gaze_y:.2f} "
        f"{command}"
    )


    draw.rectangle(
        (
            0,
            0,
            110,
            11
        ),
        fill=(0, 0, 0)
    )


    draw.text(
        (
            2,
            1
        ),
        info,
        fill=(255, 255, 255),
        font=font
    )


    # ========================================================
    # SCALE
    # ========================================================

    image = image.resize(
        (
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT
        ),
        Image.Resampling.NEAREST
    )


    return image


# ============================================================
# GUI UPDATE
# ============================================================

def update():

    try:

        data = read_frame()


        if data:

            image = draw_frame(
                data
            )


            photo = ImageTk.PhotoImage(
                image
            )


            label.configure(
                image=photo
            )


            label.image = photo


    except Exception as e:

        print(
            "ERROR:",
            e
        )


    root.after(
        10,
        update
    )


# ============================================================
# START
# ============================================================

print(
    "Waiting for ESP32..."
)

root.after(
    100,
    update
)


root.mainloop()


ser.close()