import open3d as o3d
import os

def zeige_ply(dateiname):
    # 1. Prüfen, ob die Datei existiert
    if not os.path.exists(dateiname):
        print(f"FEHLER: Die Datei '{dateiname}' wurde nicht gefunden!")
        return

    print(f"Lade '{dateiname}' ...")
    
    # 2. Punktwolke laden
    pcd = o3d.io.read_point_cloud(dateiname)
    
    # Prüfen, ob Punkte geladen wurden
    if pcd.is_empty():
        print("Datei ist leer oder konnte nicht gelesen werden.")
        return

    print(f"Erfolgreich geladen: {pcd}")

    # 3. Ein Koordinatensystem zur Orientierung erstellen
    # Rot = X, Grün = Y, Blau = Z
    # size bestimmt die Größe der Pfeile (z.B. 0.1 Meter)
    koordinaten_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    # 4. Visualisierung starten
    print("Öffne Viewer... (Drücke 'Q' zum Schließen)")
    
    # Hier öffnet sich das Fenster
    o3d.visualization.draw_geometries([pcd, koordinaten_frame], 
                                      window_name="Mein 3D Viewer",
                                      width=800,
                                      height=600)

if __name__ == "__main__":
    # HIER DEN DATEINAMEN EINTRAGEN
    meine_datei = "/home/kilabor/Ki_Segmentation/scan_clean_pipeline/scan_100.ply" 
    
    zeige_ply(meine_datei)
