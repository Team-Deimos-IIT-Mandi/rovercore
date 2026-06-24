import argparse
import json
import sys

import cv2
import numpy as np

DEFAULT_CONFIG = {
    "ilmenite_v_threshold": 45,
    "min_rock_area_px": 500,
    "max_rock_area_px": 100000,
    "gauss_blur_size": 5,
    "adaptive_block_size": 51,
    "adaptive_c": 10,
    "clahe_clip_limit": 2.0,
    "clahe_grid_size": 8,
    "use_clahe": True,
}

LABEL_MAP = {
    "illemenite_rich_basalt": "Ilmenite Basalt",
    "normal_rock": "Normal Rock",
}

COLOR_MAP = {
    "illemenite_rich_basalt": (0, 0, 255),
    "normal_rock": (0, 255, 255),
}


class IlmeniteBasaltClassifier:
    def __init__(self, config: dict | None = None):
        self.config = {**DEFAULT_CONFIG, **(config or {})}
        self._clahe = None
        self._build_clahe()

    def _build_clahe(self):
        c = self.config
        if c.get("use_clahe", True):
            self._clahe = cv2.createCLAHE(
                clipLimit=c.get("clahe_clip_limit", 2.0),
                tileGridSize=(c.get("clahe_grid_size", 8),) * 2,
            )
        else:
            self._clahe = None

    def classify_rocks(self, image: np.ndarray) -> list[dict]:
        if image is None or image.size == 0:
            return [], np.zeros((1, 1), dtype=np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, s, v = cv2.split(hsv)
        if self._clahe:
            v = self._clahe.apply(v)
        blur_sz = self.config["gauss_blur_size"]
        if blur_sz % 2 == 0:
            blur_sz += 1
        blurred = cv2.GaussianBlur(v, (blur_sz,) * 2, 0)
        block_sz = self.config["adaptive_block_size"]
        if block_sz % 2 == 0:
            block_sz += 1
        c_val = self.config["adaptive_c"]
        binary = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, block_sz, c_val,
        )
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.config["min_rock_area_px"]:
                continue
            if area > self.config["max_rock_area_px"]:
                continue
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.drawContours(mask, [cnt], -1, 255, -1)
            mean_v = cv2.mean(v, mask=mask)[0]
            mean_s = cv2.mean(s, mask=mask)[0]
            x, y, w, h = cv2.boundingRect(cnt)
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else x + w // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] > 0 else y + h // 2
            threshold = self.config["ilmenite_v_threshold"]
            is_illemenite = mean_v < threshold
            label = "illemenite_rich_basalt" if is_illemenite else "normal_rock"
            confidence = (
                min(1.0, (threshold - mean_v) / max(threshold, 1))
                if is_illemenite
                else min(1.0, (mean_v - threshold) / max(255 - threshold, 1))
            )
            results.append({
                "class": label,
                "label": LABEL_MAP[label],
                "bbox": (x, y, w, h),
                "centroid": (cx, cy),
                "area": int(area),
                "mean_v": round(mean_v, 1),
                "mean_s": round(mean_s, 1),
                "confidence": round(confidence, 3),
                "contour": cnt,
            })
        results.sort(key=lambda r: r["area"], reverse=True)
        return results, binary

    def annotate_image(self, image: np.ndarray, results: list[dict]) -> np.ndarray:
        out = image.copy()
        for r in results:
            x, y, w, h = r["bbox"]
            color = COLOR_MAP.get(r["class"], (255, 255, 255))
            cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
            cx, cy = r["centroid"]
            cv2.circle(out, (cx, cy), 4, color, -1)
            cv2.putText(out, f"{r['label']} ({r['confidence']:.2f})",
                        (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        ilmenite_count = sum(1 for r in results if r["class"] == "illemenite_rich_basalt")
        normal_count = sum(1 for r in results if r["class"] == "normal_rock")
        cv2.putText(out,
                    f"Ilmenite: {ilmenite_count}  Normal: {normal_count}  Total: {len(results)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        return out


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Detect dark ilmenite-rich basalt from normal rocks "
                    "using HSV colour-thresholding."
    )
    g = p.add_mutually_exclusive_group()
    g.add_argument("--image", "-i", type=str, default=None,
                    help="Path to a static image file.")
    g.add_argument("--camera", "-c", type=int, nargs="?", const=0,
                    default=None,
                    help="Camera device index (default 0). Omit flag for "
                         "live mode.")
    p.add_argument("--config", "-cfg", type=str, default=None,
                    help="Optional JSON config to override thresholds.")
    p.add_argument("--output", "-o", type=str, default="output_annotated.jpg",
                    help="Output path for annotated image (static mode).")
    return p.parse_args()


def _load_config(path: str | None) -> dict:
    if path is None:
        return {}
    with open(path) as f:
        return json.load(f)


def _print_results(results: list[dict]) -> None:
    ilmenite = [r for r in results if r["class"] == "illemenite_rich_basalt"]
    normal = [r for r in results if r["class"] == "normal_rock"]
    print(f"\n  Ilmenite-rich basalt: {len(ilmenite)}")
    print(f"  Normal rocks:         {len(normal)}")
    print(f"  Total:                {len(results)}")
    for r in results:
        x, y, w, h = r["bbox"]
        print(f"  [{r['class']:>24s}]  conf={r['confidence']:.2f}  "
              f"V={r['mean_v']:.0f}  S={r['mean_s']:.0f}  "
              f"area={r['area']}px  bbox=({x},{y},{w},{h})")


def run_static(args: argparse.Namespace,
               classifier: IlmeniteBasaltClassifier) -> None:
    image = cv2.imread(args.image)
    if image is None:
        print(f"Error: cannot load '{args.image}'", file=sys.stderr)
        sys.exit(1)
    print(f"Loaded '{args.image}'  ({image.shape[1]}×{image.shape[0]})")
    results, _ = classifier.classify_rocks(image)
    _print_results(results)
    annotated = classifier.annotate_image(image, results)
    cv2.imwrite(args.output, annotated)
    print(f"\nAnnotated image saved to '{args.output}'")
    cv2.imshow("Rock Detection", annotated)
    print("Press any key to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


TRACKBAR_WINDOW = "Controls"


def _nothing(_):
    pass


def _setup_trackbars(config: dict):
    cv2.namedWindow(TRACKBAR_WINDOW)
    cv2.createTrackbar("V threshold", TRACKBAR_WINDOW,
                        config["ilmenite_v_threshold"], 255, _nothing)
    cv2.createTrackbar("Block size", TRACKBAR_WINDOW,
                        config["adaptive_block_size"], 199, _nothing)
    cv2.createTrackbar("C value", TRACKBAR_WINDOW,
                        config["adaptive_c"] + 50, 100, _nothing)
    cv2.createTrackbar("Blur size", TRACKBAR_WINDOW,
                        config["gauss_blur_size"], 31, _nothing)
    cv2.createTrackbar("Min area", TRACKBAR_WINDOW,
                        config["min_rock_area_px"], 10000, _nothing)
    cv2.createTrackbar("Max area", TRACKBAR_WINDOW,
                        config["max_rock_area_px"] // 100, 2000, _nothing)


def _read_trackbars(config: dict) -> bool:
    changed = False
    val = cv2.getTrackbarPos("V threshold", TRACKBAR_WINDOW)
    if val != config["ilmenite_v_threshold"]:
        config["ilmenite_v_threshold"] = max(1, val)
        changed = True
    val = cv2.getTrackbarPos("Block size", TRACKBAR_WINDOW)
    if val != config["adaptive_block_size"]:
        config["adaptive_block_size"] = max(3, val | 1)
        changed = True
    val = cv2.getTrackbarPos("C value", TRACKBAR_WINDOW)
    adjusted = val - 50
    if adjusted != config["adaptive_c"]:
        config["adaptive_c"] = adjusted
        changed = True
    val = cv2.getTrackbarPos("Blur size", TRACKBAR_WINDOW)
    if val != config["gauss_blur_size"]:
        config["gauss_blur_size"] = max(1, val | 1)
        changed = True
    val = cv2.getTrackbarPos("Min area", TRACKBAR_WINDOW)
    if val != config["min_rock_area_px"]:
        config["min_rock_area_px"] = max(0, val)
        changed = True
    val = cv2.getTrackbarPos("Max area", TRACKBAR_WINDOW)
    scaled = val * 100
    if scaled != config["max_rock_area_px"]:
        config["max_rock_area_px"] = max(100, scaled)
        changed = True
    return changed


def run_live(args: argparse.Namespace,
             classifier: IlmeniteBasaltClassifier) -> None:
    cam_index = args.camera if args.camera is not None else 0
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"Error: cannot open camera {cam_index}", file=sys.stderr)
        sys.exit(1)
    _setup_trackbars(classifier.config)
    mask_window = "Segmentation Mask"
    cv2.namedWindow(mask_window, cv2.WINDOW_NORMAL)
    print(f"Live camera {cam_index} — press 'q' to quit, 's' to save frame.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            if _read_trackbars(classifier.config):
                classifier._build_clahe()
            results, binary = classifier.classify_rocks(frame)
            annotated = classifier.annotate_image(frame, results)
            mask_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            stacked = np.vstack([annotated, mask_bgr])
            cv2.imshow("Rock Detection — Live", stacked)
            cv2.imshow(mask_window, binary)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("Quit.")
                break
            elif key == ord("s"):
                path = "capture_annotated.jpg"
                cv2.imwrite(path, annotated)
                print(f"Saved {path}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    args = _parse_args()
    overrides = _load_config(args.config)
    config = {**DEFAULT_CONFIG, **overrides}
    classifier = IlmeniteBasaltClassifier(config)
    if args.image:
        run_static(args, classifier)
    else:
        run_live(args, classifier)


if __name__ == "__main__":
    main()
