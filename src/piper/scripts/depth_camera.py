import pyrealsense2 as rs
import numpy as np
import cv2

# === Filtros manuales ===
def remove_impulse_noise(depth_img, threshold=200):
    median_local = cv2.medianBlur(depth_img, 3)
    diff = cv2.absdiff(depth_img, median_local)
    mask = diff > threshold
    filtered = depth_img.copy()
    filtered[mask] = median_local[mask]
    return filtered

def median_smooth(depth_img):
    return cv2.medianBlur(depth_img, 5)

# === Configurar pipeline ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)
print("✅ RealSense D405 detectada. Presiona ESC para salir.")

align = rs.align(rs.stream.color)

try:
    while True:
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convertir a numpy
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # --- Filtros manuales ---
        depth_filtered = remove_impulse_noise(depth_image, threshold=200)
        depth_filtered = median_smooth(depth_filtered)

        # Normalizar y colorear
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_filtered, alpha=0.03),
            cv2.COLORMAP_TURBO
        )

        # Redimensionar para coincidir
        depth_colormap = cv2.resize(depth_colormap, (color_image.shape[1], color_image.shape[0]))

        # Concatenar lado a lado
        images = np.hstack((color_image, depth_colormap))

        cv2.imshow('RGB (izq) | Depth (der, filtrado manual)', images)

        if cv2.waitKey(1) == 27:  # ESC
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
