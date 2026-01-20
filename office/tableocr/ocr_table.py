import os
import csv
from paddleocr import PaddleOCR

# 降低 Paddle 日志噪音（可选）
os.environ["FLAGS_log_level"] = "3"

# ===== OCR 初始化（新 pipeline 唯一安全写法）=====
ocr = PaddleOCR(
    lang='ch',
    use_textline_orientation=True
)

image_dir = "tables"
out_dir = "ocr_out"
os.makedirs(out_dir, exist_ok=True)


def y_center(poly):
    return sum(p[1] for p in poly) / len(poly)


def x_center(poly):
    return sum(p[0] for p in poly) / len(poly)


for fname in os.listdir(image_dir):
    if not fname.lower().endswith((".png", ".jpg", ".jpeg")):
        continue

    print(f"\n==== OCR {fname} ====")

    img_path = os.path.join(image_dir, fname)

    # ✅ 新 pipeline 接口
    result = ocr.predict(img_path)

    if not result:
        print("⚠️ no result")
        continue

    res = result[0]

    texts = res.get("rec_texts", [])
    scores = res.get("rec_scores", [])
    polys = res.get("dt_polys", [])

    rows = []
    for text, score, poly in zip(texts, scores, polys):
        rows.append({
            "text": text.strip(),
            "score": score,
            "x": x_center(poly),
            "y": y_center(poly)
        })

    # 表格排序：先 y 后 x
    rows.sort(key=lambda r: (round(r["y"] / 10), r["x"]))

    out_csv = os.path.join(out_dir, fname + ".csv")
    with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["text", "x", "y", "score"])
        for r in rows:
            writer.writerow([
                r["text"],
                f"{r['x']:.1f}",
                f"{r['y']:.1f}",
                f"{r['score']:.3f}"
            ])

    print(f"→ saved {out_csv}")
