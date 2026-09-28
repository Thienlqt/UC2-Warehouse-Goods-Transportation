"""Draw YOLO labels on a sample of dataset images and tile them into one contact sheet.

usage: preview_labels.py DATASET_DIR [--split train] [--count 16] [--out sheet.jpg]
"""

import argparse
import random
from pathlib import Path

import cv2
import numpy as np

COLOURS = [(56, 56, 255), (29, 178, 255), (10, 249, 72), (211, 188, 0)]  # BGR per class id


def draw(image_path, label_path, names):
    image = cv2.imread(str(image_path))
    h, w = image.shape[:2]
    lines = label_path.read_text().split('\n') if label_path.exists() else []
    for line in filter(None, lines):
        cls, cx, cy, bw, bh = line.split()
        cls = int(cls)
        cx, cy, bw, bh = float(cx) * w, float(cy) * h, float(bw) * w, float(bh) * h
        p1 = int(cx - bw / 2), int(cy - bh / 2)
        p2 = int(cx + bw / 2), int(cy + bh / 2)
        colour = COLOURS[cls % len(COLOURS)]
        cv2.rectangle(image, p1, p2, colour, 1)
        cv2.putText(image, names[cls], (p1[0], max(p1[1] - 3, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    colour, 1, cv2.LINE_AA)
    cv2.putText(image, image_path.name, (4, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--split', default='train')
    parser.add_argument('--count', type=int, default=16)
    parser.add_argument('--cols', type=int, default=4)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--out', type=Path, default=Path('label_preview.jpg'))
    args = parser.parse_args()

    names = (args.dataset / 'classes.txt').read_text().split()
    images = sorted((args.dataset / 'images' / args.split).glob('*.jpg'))
    random.Random(args.seed).shuffle(images)
    tiles = [draw(p, args.dataset / 'labels' / args.split / (p.stem + '.txt'), names)
             for p in images[:args.count]]
    while len(tiles) % args.cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + args.cols]) for i in range(0, len(tiles), args.cols)]
    cv2.imwrite(str(args.out), np.vstack(rows))
    print('wrote', args.out)


if __name__ == '__main__':
    main()
