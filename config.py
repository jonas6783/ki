# config.py

CAMERA_WIDTH = 1280  # oder 1288, falls RealSense das akzeptiert (Standard HD ist 1280x720)
CAMERA_HEIGHT = 720  # oder 808
CAMERA_FPS = 30
AUTOEXPOSURE_FRAMES = 15

# Scanner Hardware
SERIAL_PORT = 'COM3'
BAUDRATE = 115200

# Marker Setup (WICHTIG!)
# Größe des schwarzen Quadrats in Meter
MARKER_SIZE = 0.1  # 100mm
ROTATION_STEP = 20.0
TOTAL_ANGLE = 360.0

# Die "Scan-Box" (Relativ zum Marker!)
# config.py
# Sticker ist 10cm (also +/- 0.05m vom Zentrum)
# Höhe soll 10cm sein (0.10m)

# config.py - RIESIGE BOX zum Testen
BOX_MIN = [-0.80, -0.80, -0.80]
BOX_MAX = [ 0.80,  0.80,  0.80]

# Verarbeitung
VOXEL_SIZE = 0.0
MESH_DEPTH = 11   #9=grob, 10,11 = gute qualität