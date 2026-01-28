import open3d as o3d

from alignment import align_scans
from inspection import inspect

# =================================================
# DATEIEN
# =================================================
CAD_FILE = r"Cad/kopfstueck (1).ply"

SCAN_FILES =[


r"C:\PythonProject7\Project\Scanner\scan_clean_pipeline\scan_000.ply",
r"C:\PythonProject7\Project\Scanner\scan_clean_pipeline\scan_020.ply",


]

SCAN_NAMES = [
    "scan_000",
"scan_020",

]

REPORT_CSV = "inspection_report.csv"
REWORK_CSV = "inspection_rework_points.csv"

# =================================================
# MAIN
# =================================================
def main():
    print("====================================")
    print("START: ALIGNMENT")
    print("====================================")

    cad_pcd, aligned_scans, metrics = align_scans(
        CAD_FILE, SCAN_FILES
    )

    print("\nALIGNMENT-QUALITÄT PRO SCAN")
    for m in metrics:
        print(
            f"{m['scan']}: "
            f"fitness={m['fitness']:.3f}, "
            f"rmse={m['rmse']:.3f} mm"
        )

    print("\nViewer: Alignment")
    o3d.visualization.draw_geometries(
        [cad_pcd] + aligned_scans,
        window_name="Alignment – Per Scan Farbe",
    )

    print("\n====================================")
    print("START: QS-PRÜFUNG")
    print("====================================")

    colored_scans = inspect(
        cad_pcd=cad_pcd,
        scan_pcds=aligned_scans,
        scan_names=SCAN_NAMES,
        report_csv=REPORT_CSV,
        rework_csv=REWORK_CSV,
    )

    print("\nViewer: QS-Ergebnis")
    print("🟢 Grün = OK")
    print("🔴 Rot  = Grat XXX FAIL")
    print("🔵 Blau = Anguss FAIL")
    print("🟡 Gelb = OOO FAIL")

    o3d.visualization.draw_geometries(
        [cad_pcd] + colored_scans,
        window_name="QS Ergebnis – Zonenfarben",
    )

    print("\n====================================")
    print("FERTIG")
    print("====================================")


# =================================================
# ENTRY POINT
# =================================================
if __name__ == "__main__":
    main()


