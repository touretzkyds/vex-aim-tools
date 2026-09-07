"""
Open-vocabulary object detection (YOLOE) for Celeste.
Detection is *on demand*, not per frame.
"""

import os
import threading
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from .utils import Pose

# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

DEFAULT_IMGSZ = 640
"""Inference resolution."""

DEFAULT_CONF = 0.2
""" confidence threshold for recognizing objects"""


MODEL_FILE = 'yoloe-26l-seg.pt'
TEXT_ENCODER_FILE = 'mobileclip2_b.ts'


# ---------------------------------------------------------------------------
# Asset (YoloE and textencoder) location
# ---------------------------------------------------------------------------

MODELS_DIR = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'models'))
""" The path to YoloE model and text encoder"""


def find_asset(filename):
    """Return the absolute path to filename in MODELS_DIR, or raise."""
    path = os.path.join(MODELS_DIR, filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f'Could not find {filename} in {MODELS_DIR}')
    return path


# ---------------------------------------------------------------------------
# The detector
# ---------------------------------------------------------------------------

class OpenVocabDetector():
    def __init__(self, robot=None, imgsz=DEFAULT_IMGSZ, conf=DEFAULT_CONF,
                 device='cpu'):
        self.robot = robot
        self.model_path = find_asset(MODEL_FILE)
        self.text_encoder_path = find_asset(TEXT_ENCODER_FILE)
        self.imgsz = imgsz
        self.conf = conf
        self.device = device
        self.model = None
        self.current_classes = None
        self._lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=1,
                                            thread_name_prefix='openvocab')

    def load(self):
        """Load the weights and the text encoder."""
        with self._lock:
            if self.model is not None:
                return self.model
            from ultralytics import YOLOE
            from ultralytics.nn.text_model import MobileCLIPTS
            import torch
            model = YOLOE(self.model_path)
            # get_text_pe() otherwise rebuilds the text encoder on every
            # set_classes(), resolving its name against the cwd.
            model.model.clip_model = MobileCLIPTS(torch.device(self.device),
                                                  weight=self.text_encoder_path)
            self.model = model
            return self.model

    def warm_up(self, label='object'):
        """Load the model and encode a placeholder label.
        Loading the text encoder at startup so that every later detection is faster
        """
        self.load()
        self.set_prompts([label])

    def set_prompts(self, classes):
        """Set the detector vocabulary, skipping the work when it is unchanged."""
        with self._lock:
            model = self.load()
            if self.current_classes == classes:
                return
            # cache_clip_model=True uses the encoder pinned in load().
            embeddings = model.model.get_text_pe(classes, cache_clip_model=True)
            model.set_classes(classes, embeddings)
            self.current_classes = classes

    def detect(self, image, label, variants=None, imgsz=None):
        """Run one inference on an RGB image and return detection records.

        All variants ride in one inference; NMS runs across them, so overlapping
        boxes collapse and the best-scoring variant wins."""
        classes = [label] + list(variants or [])
        with self._lock:
            self.set_prompts(classes)
            # ultralytics reads a numpy array as BGR; camera images are RGB.
            bgr = np.ascontiguousarray(image[:, :, ::-1])
            results = self.model.predict(
                bgr, conf=self.conf,
                imgsz=self.imgsz if imgsz is None else imgsz, verbose=False)
        return self._records(results[0], label)

    def _records(self, result, label):
        """ Coordinates are full resolution"""
        records = []
        for (x1, y1, x2, y2), score, cid in zip(result.boxes.xyxy.tolist(),
                                                result.boxes.conf.tolist(),
                                                result.boxes.cls.tolist()):
            records.append({
                'label': label,                             # identity in the map
                'matched_variant': result.names[int(cid)],  # phrasing that fired
                'originx': x1,
                'originy': y1,
                'width': x2 - x1,
                'height': y2 - y1,
                'score': score,
            })
        records.sort(key=lambda r: -r['score'])
        return records

    def detect_async(self, label, variants=None, imgsz=None):
        """Detect on the current camera image off the loop thread.

        The batch is appended to robot.openvocab_results on the loop thread, so
        world map ingestion stays single-threaded."""
        image = self.robot.camera_image.copy()
        # Snapshot the pose: the particle filter keeps updating robot.pose while
        # the worker runs, and the projection needs the pose at capture time.
        p = self.robot.pose
        batch = {'label': label,
                 'capture_pose': Pose(p.x, p.y, p.z, p.theta),
                 'frame_count': self.robot.frame_count,
                 'detections': []}

        def work():
            try:
                batch['detections'] = self.detect(image, label, variants, imgsz)
            except Exception as e:
                print(f'*** openvocab: detection for {label!r} failed: {e}')
            self.robot.loop.call_soon_threadsafe(self._post, batch)

        return self._executor.submit(work)

    def _post(self, batch):
        self.robot.openvocab_results.append(batch)
