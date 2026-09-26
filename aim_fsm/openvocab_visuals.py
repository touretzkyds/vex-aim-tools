"""Pure candidate filtering and image layout for open-vocabulary verification.

These functions neither call models nor publish detections or move the robot.
"""
import cv2
import numpy as np

DUPLICATE_IOU = 0.9             # equivalent outlines
DUPLICATE_PX = 2                # if no edge differs by more than this
CROP_PADDING_PX = 16            # minimum context around a verification crop
CROP_PADDING_FRACTION = 0.25    # of the box's larger side

def verification_candidates(records):
    """Remove equivalent outlines without merging different ground-contact edges."""
    kept = []
    for record in records:
        box = np.array([record['originx'], record['originy'],
                        record['originx'] + record['width'],
                        record['originy'] + record['height']])
        duplicate = False
        for other in kept:
            previous = np.array([other['originx'], other['originy'],
                                 other['originx'] + other['width'],
                                 other['originy'] + other['height']])
            intersection = np.maximum(0, np.minimum(box[2:], previous[2:]) -
                                      np.maximum(box[:2], previous[:2])).prod()
            union = record['width'] * record['height'] + other['width'] * other['height'] - intersection
            if (union > 0 and intersection / union >= DUPLICATE_IOU
                    and np.max(np.abs(box - previous)) <= DUPLICATE_PX):
                duplicate = True
                break
        if not duplicate:
            kept.append(record)
    return kept


def verification_panels(image, records, include_scene=True, base_tolerance_px=0):
    """Build padded candidate crops, optionally preceded by the full scene."""
    height, width = image.shape[:2]
    tile_width, tile_height = width // 2, height // 2
    header = 32
    offset = height + header if include_scene else 0
    panel_width = width if include_scene else tile_width * min(2, len(records))
    annotated = np.zeros((offset + ((len(records) + 1) // 2) *
                          (tile_height + header), panel_width, 3), dtype=image.dtype)
    if include_scene:
        annotated[header:header + height] = image
        cv2.putText(annotated, 'Full scene', (4, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    .7, (255, 255, 255), 2, cv2.LINE_AA)
    for i, r in enumerate(records):
        x1, y1 = int(r['originx']), int(r['originy'])
        x2, y2 = int(r['originx'] + r['width']), int(r['originy'] + r['height'])
        padding = max(CROP_PADDING_PX, int(max(r['width'], r['height']) * CROP_PADDING_FRACTION))
        left, top = max(0, x1 - padding), max(0, y1 - padding)
        right, bottom = min(width, x2 + padding), min(height, y2 + padding)
        crop = image[top:bottom, left:right].copy()
        if base_tolerance_px:
            # Draw boundary lines without obscuring the contact surface.
            for band_y in (y2 - base_tolerance_px, y2 + base_tolerance_px):
                cv2.line(crop, (x1 - left, band_y - top),
                         (x2 - left, band_y - top), (255, 200, 0), 1)
        cv2.rectangle(crop, (x1 - left, y1 - top), (x2 - left, y2 - top), (0, 255, 0), 1)
        scale = min(tile_width / crop.shape[1], tile_height / crop.shape[0])
        crop = cv2.resize(crop, (max(1, round(crop.shape[1] * scale)),
                                max(1, round(crop.shape[0] * scale))))
        row, column = divmod(i, 2)
        x, y = column * tile_width, offset + row * (tile_height + header)
        cv2.putText(annotated, f'Candidate {i + 1}', (x + 4, y + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2, cv2.LINE_AA)
        annotated[y + header:y + header + crop.shape[0], x:x + crop.shape[1]] = crop
    return annotated


