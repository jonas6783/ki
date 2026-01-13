import open3d as o3d
import numpy as np
import os
from rembg import remove, new_session

class PointCloudProcessor:
    def __init__(self, marker_position_vector):
        self.marker_pos = marker_position_vector
        self.main_pcd = o3d.geometry.PointCloud()

        # Ordner erstellen
        self.save_folder = "scan_ai_only"
        os.makedirs(self.save_folder, exist_ok=True)
        print(f"[Processor] MODE: KI + Drehung + Smart Filter. Speichere in '{self.save_folder}/'")

        print("Lade KI Modell (ISNET)...")
        self.session = new_session("isnet-general-use")

    def clean_point_cloud(self, pcd):
        """
        Entfernt schwebende Artefakte und Rauschen, ohne das Bauteil zu glätten.
        """
        if len(pcd.points) < 100:
            return pcd

        # 1. STATISTICAL OUTLIER REMOVAL (Ganz sanft)
        # Entfernt einzelne Pixel, die wild in der Luft hängen (Rauschen)
        # nb_neighbors=20, std_ratio=2.0 -> Sehr konservativ, löscht nur extremen Müll
        pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # 2. DBSCAN CLUSTERING (Der "Insel-Filter")
        # Das ist der wichtigste Schritt gegen Drehteller-Reste!
        # Wir suchen zusammenhängende Punkt-Gruppen.
        # eps=0.005 (5mm Abstand max zwischen Punkten), min_points=10
        labels = np.array(pcd.cluster_dbscan(eps=0.005, min_points=10, print_progress=False))

        if len(labels) > 0:
            max_label = labels.max()
            if max_label >= 0:
                # Zähle, welcher Cluster die meisten Punkte hat (das ist unser Bauteil)
                counts = np.bincount(labels[labels >= 0])
                largest_cluster_idx = np.argmax(counts)

                # Behalte NUR den größten Cluster
                indices = np.where(labels == largest_cluster_idx)[0]
                pcd = pcd.select_by_index(indices)

        return pcd

    def process_frame(self, data, angle):
        print(f"[Processor] Winkel {angle}° -> KI -> Filter -> Speichern...")

        color_raw = data['color']
        depth_raw = data['depth']

        # --- 1. KI HINTERGRUND ENTFERNUNG ---
        try:
            # KI schneidet das Objekt aus
            result_image = remove(color_raw, session=self.session, alpha_matting=False)
            mask = result_image[:, :, 3] > 0

            clean_depth = depth_raw.copy()
            clean_depth[~mask] = 0  # Alles was nicht Maske ist, wird Tiefe 0 (entfernt)
        except Exception as e:
            print(f"KI Fehler: {e}")
            clean_depth = depth_raw

        # --- 2. PUNKTWOLKE ERSTELLEN ---
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_raw),
            o3d.geometry.Image(clean_depth),
            depth_scale=data['scale_factor'],
            convert_rgb_to_intensity=False,
            depth_trunc=1.0 # Alles was weiter weg ist als 1 Meter wird ignoriert
        )

        intr = data['intrinsics']
        intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

        source = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

        # --- 3. DREHUNG & POSITIONIERUNG ---
        # Zum Marker schieben (Zentrieren)
        if self.marker_pos is not None:
            source.translate(-self.marker_pos)
        else:
            source.translate((0, 0, -0.15))

        # Rotieren
        rad = -angle * np.pi / 180.0
        R = source.get_rotation_matrix_from_xyz((0, rad, 0))
        source.rotate(R, center=(0, 0, 0))

        # --- NEU: 3.5 SMART CLEANING ---
        # Hier entfernen wir die schwebenden Punkte VOR dem Speichern
        source = self.clean_point_cloud(source)

        # --- 4. SPEICHERN ---
        # Speichert diesen einen Winkel als Datei
        filename = f"{self.save_folder}/scan_{int(angle):03d}.ply"
        o3d.io.write_point_cloud(filename, source)

        # Auch zum Gesamtspeicher hinzufügen (für die End-Datei)
        self.main_pcd += source

    def create_mesh(self, depth, output_filename):
        # Hier wird KEIN Mesh erzeugt, nur die gesammelten Punkte gespeichert
        print("Speichere komplette Punktwolke (Smart Filtered)...")
        
        # Optional: Auch die gesamte Wolke am Ende nochmal kurz reinigen,
        # falls beim Zusammensetzen Rauschen entstanden ist
        self.main_pcd = self.clean_point_cloud(self.main_pcd)

        final_name = output_filename.replace(".ply", "_combined_points.ply")
        o3d.io.write_point_cloud(final_name, self.main_pcd)

        print(f"Fertig! Alles gespeichert in: {final_name}")
