# ==============================

CAMERA_WIDTH = 1280  #
CAMERA_HEIGHT = 720  #
CAMERA_FPS = 30
AUTOEXPOSURE_FRAMES = 15
# HARDWARE
# ==============================
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200


# ==============================
# MARKER
# ==============================
# Marker-Kantenlänge in Meter
MARKER_SIZE = 0.025   # 25 mm


# ==============================
# SCAN
# ==============================
ROTATION_STEP = 20.0
TOTAL_ANGLE = 360.0


# ==============================
# BOX (Marker = Mittelpunkt!)
# ==============================
# Halbausdehnungen der Box (± um Marker)
BOX_HALF_X = 0.1  # ±6 cm   0.05
BOX_HALF_Y = 0.1          #0.07
BOX_HALF_Z = 0.1          #      0.08


# ==============================
# FILTER FLAGS
# ==============================
USE_AI_MASK = True          # KI an/aus
USE_DBSCAN = False         # Cluster-Filter
USE_OUTLIER_REMOVAL = False# Statistik-Filter

# ==============================
# POINT CLOUD FILTERING
# ==============================

# --- DBSCAN ---
DBSCAN_EPS = 0.002          # 5 mm maximaler Abstand, den zwei Punkte haben dürfen, damit sie als Nachbarn gelten.

DBSCAN_MIN_POINTS = 10  #Ein Punkt ist nur dann Teil eines Clusters, wenn er mindestens 10 Nachbarn im eps-Radius hat.

# --- OUTLIER REMOVAL ---
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0



# ==============================
# DEPTH / SENSOR
# ==============================
DEPTH_TRUNC = 0.75   # Meter


# ==============================
# DEBUG
# ==============================
VERBOSE = True

# ==============================
# OUTPUT
# ==============================
OUTPUT_FOLDER = "scan_clean_pipeline"




