"""
Raspberry Pi 4 + Intel RealSense D435 + YOLO
- 컬러 영상에서 객체 탐지
- depth를 컬러에 정렬(align)해서 각 객체까지의 거리(m) 표시

실행: python3 rs_yolo.py            (모니터 연결 시)
      python3 rs_yolo.py --headless (SSH 등 화면 없을 때)
"""
import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ---------------- 설정 ----------------
WIDTH, HEIGHT, FPS = 640, 480, 15   # Pi4에선 15fps가 안정적
MODEL_PATH = "yolo11n.pt"           # 속도 원하면 NCNN 변환 모델 사용 (아래 설명)
IMGSZ = 320                         # 추론 해상도 (작을수록 빠름)
CONF = 0.4


def get_distance(depth_frame, x1, y1, x2, y2):
    """박스 중앙 영역의 depth 중앙값(m). 노이즈/0값에 강함."""
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    w, h = max((x2 - x1) // 6, 2), max((y2 - y1) // 6, 2)
    depth = np.asanyarray(depth_frame.get_data())
    roi = depth[max(cy - h, 0):cy + h, max(cx - w, 0):cx + w]
    valid = roi[roi > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) * depth_frame.get_units()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    model = YOLO(MODEL_PATH)

    # RealSense 파이프라인
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # 오토 노출 안정화용으로 초반 프레임 버리기
    for _ in range(30):
        pipeline.wait_for_frames()

    prev = time.time()
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            results = model(img, imgsz=IMGSZ, conf=CONF, verbose=False)[0]

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                name = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                dist = get_distance(depth_frame, x1, y1, x2, y2)
                dist_txt = f"{dist:.2f}m" if dist else "N/A"

                if args.headless:
                    print(f"{name} {conf:.2f} {dist_txt}")
                else:
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img, f"{name} {dist_txt}", (x1, max(y1 - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            now = time.time()
            fps = 1 / (now - prev)
            prev = now

            if args.headless:
                print(f"--- FPS {fps:.1f}")
            else:
                cv2.putText(img, f"FPS {fps:.1f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("RealSense YOLO", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
