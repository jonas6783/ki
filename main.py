import config
from turntable import Turntable
from camera import RealSenseCamera
from processor import PointCloudProcessor
from calibration import Calibrator
import numpy as np
import open3d as o3d
import sys
import os
import copy
from inspection import GeometryInspector
from result_viewer import AlignmentViewer

# ==========================================
# KONFIGURATION INSPEKTION
# ==========================================
CAD_FILE = "kopfstueck.ply"      
CAD_SCALE = 0.001
VOXEL_SIZE = 0.002   
INSPECTION_POINT_COUNT = 250000 
KILLZONE_POINT_LIMIT = 500 

# ZONES Definition (unverändert)
ZONES = [
    {
        "name": "obere Seite (Killzone)",
        "center": [-0.02090, 0.00015, 0.00000],
        "extent": [0.00330, 0.07270, 0.01490],
        "R":      np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]), 
        "tolerance": 0.0005, 
    },
    {
        "name": "linke Zone",
        "center": [-0.03425, 0.03905, 0.00000],
        "extent": [0.02460, 0.00560, 0.02640],
        "R":      np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]), 
        "tolerance": 0.0005,
    },
    {
        "name": "rechte Zone",
        "center": [-0.04000, -0.03915, 0.00000],
        "extent": [0.03500, 0.00510, 0.03000],
        "R":      np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]), 
        "tolerance": 0.0005,
    },
    {
        "name": "Zone Mitte oben",
        "center": [-0.04165, 0.00000, 0.00000],
        "extent": [0.02350, 0.01080, 0.03550],
        "R":      np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]), 
        "tolerance": 0.0005,
    },
    {
        "name": "Zone rechts rotiert",
        "center": [-0.05430, 0.03315, 0.00000],
        "extent": [0.02000, 0.00500, 0.04500],
        "R":      np.array([[ 0.80385686, -0.59482279,  0.        ], [ 0.59482279,  0.80385686,  0.        ], [ 0.        ,  0.  ,  1.        ]]), 
        "tolerance": 0.0005,
    },
    {
        "name": "Zone links rotiert",
        "center": [-0.06340, -0.03550, 0.00000],
        "extent": [0.01500, 0.01000, 0.03500],
        "R":      np.array([[ 0.46947156,  0.88294759,  0.        ], [-0.88294759,  0.46947156,  0.        ], [ 0.        ,  0.  ,  1.        ]]), 
        "tolerance": 0.0005,
    },
    {
        "name": "Zone links unten rotiert",
        "center": [-0.06800, -0.02735, 0.00000],
        "extent": [0.00510, 0.00510, 0.01500],
        "R":      np.array([[ 0.9945219 ,  0.10452846,  0.        ], [-0.10452846,  0.9945219 ,  0.        ], [ 0.        ,  0.  ,  1.        ]]), 
        "tolerance": 0.0005,
    },
    {
        "name": "Zone rechts unten rotiert",
        "center": [-0.06800, 0.02700, 0.00000],
        "extent": [0.01000, 0.00500, 0.01500],
        "R":      np.array([[ 0.97029573, -0.2419219 ,  0.        ], [ 0.2419219 ,  0.97029573,  0.        ], [ 0.        ,  0.  ,  1.        ]]),
        "tolerance": 0.0005,
    }, 
    {
        "name": "Zone mitte rotiert1",
        "center": [-0.02475, 0.00920, 0.00000],
        "extent": [0.00510, 0.01500, 0.02700],
        "R":      np.array([[ 8.70349148e-01,  4.92434913e-01, -4.66016354e-04], [-4.92435133e-01,  8.70348794e-01, -7.85889605e-04], [ 1.85972925e-05,  9.13481174e-04,  9.99999583e-01]]),
        "tolerance": 0.0005,
    },
    {
        "name": "Zone mitte rotiert2",
        "center": [-0.02420, -0.01085, 0.00000],
        "extent": [0.00510, 0.01500, 0.02700],
        "R":      np.array([[ 8.70362243e-01, -4.92411797e-01,  4.33622706e-04], [ 4.92411987e-01,  8.70361872e-01, -8.04217050e-04], [ 1.85972925e-05,  9.13481174e-04,  9.99999583e-01]]),
        "tolerance": 0.0005,
    },
]

# ==========================================
# HILFSFUNKTIONEN
# ==========================================

def load_geometry(path, scale=1.0):
    if not os.path.exists(path): return None, False
    try:
        mesh = o3d.io.read_triangle_mesh(path)
        if len(mesh.triangles) > 0:
            mesh.scale(scale, center=(0,0,0))
            return mesh, True
    except: pass 
    pcd = o3d.io.read_point_cloud(path)
    pcd.scale(scale, center=(0,0,0))
    return pcd, False

def check_killzone(pcd, zones_list, limit):
    """Prüft nur die Killzone und gibt True zurück, wenn sie verletzt wurde."""
    for zone in zones_list:
        if "killzone" in zone["name"].lower():
            box = o3d.geometry.OrientedBoundingBox(
                center=np.array(zone["center"]), R=zone["R"], extent=np.array(zone["extent"])
            )
            indices = box.get_point_indices_within_bounding_box(pcd.points)
            count = len(indices)
            print(f"   -> Check Killzone: {count} Punkte (Limit: {limit})")
            if count > limit:
                return True, count
    return False, 0

# ==========================================
# TEIL 1: SCANNER PROZESS
# ==========================================

def run_scanning_process():
    print("\n" + "="*40)
    print("PHASE 1: D405 SCANNER & AUFNAHME")
    print("="*40)

    marker_pos = np.array([0.0, 0.0, 0.15])

    choice = input("Marker-Kalibrierung starten? (j/n): ").lower()
    if choice == 'j':
        calib = Calibrator(config.CAMERA_WIDTH, config.CAMERA_HEIGHT, config.CAMERA_FPS)
        found_pos = calib.find_marker_position(config.MARKER_SIZE)
        if found_pos is not None:
            marker_pos = found_pos
            print(f"--> Kalibrierung OK. Marker bei: {marker_pos}")
            print("Stelle jetzt das Objekt auf den Marker.")
            input("Drücke ENTER zum Starten...")
        else:
            print("Kalibrierung fehlgeschlagen! Nutze Standardwerte.")

    turntable = Turntable(config.SERIAL_PORT, config.BAUDRATE)
    camera = RealSenseCamera(config.CAMERA_WIDTH, config.CAMERA_HEIGHT, config.CAMERA_FPS)
    processor = PointCloudProcessor(marker_pos)

    camera.start()
    current_angle = 0.0

    try:
        while current_angle < config.TOTAL_ANGLE:
            print(f"Scan {current_angle}°...")
            frame_data = camera.get_frame_data()
            processor.process_frame(frame_data, current_angle)

            if current_angle + config.ROTATION_STEP < config.TOTAL_ANGLE:
                turntable.rotate(config.ROTATION_STEP)
            current_angle += config.ROTATION_STEP
    finally:
        camera.stop()
        turntable.close()

    # Wir erstellen die kombinierte Datei im Hintergrund
    base_filename = "scan_marker_mode.ply"
    processor.create_mesh(config.MESH_DEPTH, base_filename)
    
    # Rückgabe: Pfad zum Einzelscan für schnellere Inspektion
    target_scan_file = os.path.join("scan_ai_only", "scan_000.ply")
    print(f"-> Auswahl für Inspektion: {target_scan_file}")
    return target_scan_file

# ==========================================
# TEIL 2: INSPEKTION MIT AUTO-KORREKTUR
# ==========================================

def run_inspection_process(scan_file_path):
    print("\n" + "="*40)
    print("PHASE 2: GRAT-INSPEKTION MIT AUTO-FLIP KORREKTUR")
    print("="*40)
    
    # 1. Daten Laden
    print("\n[1/5] Lade Geometrie...")
    cad_raw, cad_is_mesh = load_geometry(CAD_FILE, scale=CAD_SCALE)
    scan_raw, scan_is_mesh = load_geometry(scan_file_path, scale=1.0) 
    
    if cad_raw is None or scan_raw is None:
        print("FEHLER: Dateien nicht gefunden.")
        return

    # Daten Vorbereitung für RANSAC
    if cad_is_mesh:
        cad_pcd_for_calc = cad_raw.sample_points_poisson_disk(20000)
    else:
        cad_pcd_for_calc = cad_raw

    if scan_is_mesh:
        scan_for_calc = scan_raw.sample_points_poisson_disk(50000)
    else:
        scan_for_calc = copy.deepcopy(scan_raw)

    # 2. BLIND-AUSRICHTUNG (RANSAC)
    inspector = GeometryInspector(voxel_size=VOXEL_SIZE)
    print(" -> Starte globale Suche (RANSAC)...")
    
    result = inspector.register_blindly(
        scan_for_calc, cad_pcd_for_calc, attempts=50, min_fitness_threshold=0.70 
    )
    
    if result is None:
        print("FEHLER: Suche komplett fehlgeschlagen.")
        return

    print(f" -> RANSAC Fitness: {result.fitness:.4f}")
    
    # Wende RANSAC Transformation auf die Hauptwolke an
    current_transform = result.transformation
    scan_raw.transform(current_transform)

    # 3. PRÜFUNG & AUTO-KORREKTUR
    print("\n[3/5] Prüfe auf falsche Ausrichtung (Killzone)...")
    
    # Temporäre Wolke für den Check (schneller)
    check_pcd = scan_raw.uniform_down_sample(10) # Nur jeden 10. Punkt für Speed
    is_flipped, pts_in_killzone = check_killzone(check_pcd, ZONES, KILLZONE_POINT_LIMIT / 10) 
    # Limit durch 10 teilen, da wir downgesamplet haben!

    if is_flipped:
        print("!"*60)
        print(f"WARNUNG: Falsche Ausrichtung erkannt! ({pts_in_killzone * 10} Punkte in Killzone)")
        print(" -> STARTE AUTOMATISCHE KORREKTUR (180° Rotation + ICP)...")
        print("!"*60)

        # A) 180 Grad Drehung um Y-Achse (typische Verwechslung bei flachen Teilen)
        # Wenn das Teil liegt, wäre es X-Achse. Wir probieren Y (Front <-> Back).
        # Rotation um das Zentrum (0,0,0) des CADs
        rotation_matrix = scan_raw.get_rotation_matrix_from_xyz((0, np.pi, 0)) # 180 Grad Y
        scan_raw.rotate(rotation_matrix, center=(0,0,0))
        
        # B) ICP FEIN-JUSTIERUNG (Damit es wieder einrastet)
        print(" -> Raste neu ein (ICP)...")
        threshold = 0.005 # 5mm Suchradius
        icp_sol = o3d.pipelines.registration.registration_icp(
            scan_raw, cad_pcd_for_calc, threshold, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        scan_raw.transform(icp_sol.transformation)
        
        # C) Erneute Prüfung
        check_pcd_2 = scan_raw.uniform_down_sample(10)
        still_flipped, pts_2 = check_killzone(check_pcd_2, ZONES, KILLZONE_POINT_LIMIT / 10)
        
        if still_flipped:
            print(f"ACHTUNG: Korrektur fehlgeschlagen. Immer noch {pts_2 * 10} Punkte in Killzone.")
        else:
            print("ERFOLG: Bauteil wurde korrekt gedreht und ausgerichtet.")

    else:
        print(" -> Ausrichtung sieht plausibel aus.")


    # 4. FINALE WOLKE & EINFÄRBEN
    print(f"\n[4/5] Berechne Abweichungen ({INSPECTION_POINT_COUNT} Punkte)...")
    
    if scan_is_mesh:
        final_vis_pcd = scan_raw.sample_points_poisson_disk(INSPECTION_POINT_COUNT)
    else:
        final_vis_pcd = copy.deepcopy(scan_raw) # Scan_raw ist jetzt korrekt gedreht

    final_vis_pcd.paint_uniform_color([0, 0.8, 0]) 
    colors_np = np.asarray(final_vis_pcd.colors)

    # Toleranz Messung
    scene = None
    if cad_is_mesh:
        scene = o3d.t.geometry.RaycastingScene()
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(cad_raw)
        scene.add_triangles(mesh_t)
    
    for zone in ZONES:
        try:
            box = o3d.geometry.OrientedBoundingBox(
                center=np.array(zone["center"]), R=zone["R"], extent=np.array(zone["extent"])
            )
            indices = box.get_point_indices_within_bounding_box(final_vis_pcd.points)
            if len(indices) == 0: continue
            
            # Färbe Killzone Rot, wenn Punkte drin sind (Visualisierung)
            if "killzone" in zone["name"].lower():
                for idx in indices: colors_np[idx] = [1, 0, 1] # Magenta für Killzone-Treffer
                continue

            if scene:
                zone_points = np.asarray(final_vis_pcd.points)[indices]
                query = o3d.core.Tensor.from_numpy(zone_points.astype(np.float32))
                signed_dist = scene.compute_signed_distance(query).numpy()
                
                for k, idx in enumerate(indices):
                    if abs(signed_dist[k]) > zone["tolerance"]:
                        colors_np[idx] = [1, 0, 0]
            else:
                for idx in indices: colors_np[idx] = [1, 0, 0]
        except: continue

    final_vis_pcd.colors = o3d.utility.Vector3dVector(colors_np)
    
    # 5. VIEWER
    print("\n[5/5] Starte Viewer.")
    viewer = AlignmentViewer(final_vis_pcd, cad_raw, window_name="Inspektion (Auto-Corrected)")
    viewer.run()

# ==========================================
# MAIN EXECUTION
# ==========================================

def main():
    generated_scan_file = run_scanning_process()
    
    if os.path.exists(generated_scan_file):
        run_inspection_process(generated_scan_file)
    else:
        print(f"Kritischer Fehler: Datei {generated_scan_file} wurde nicht gefunden!")
        print("Stelle sicher, dass der Ordner 'scan_ai_only' existiert.")

if __name__ == "__main__":
    main()
