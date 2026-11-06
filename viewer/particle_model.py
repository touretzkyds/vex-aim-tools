"""Qt list models projecting particle filter state for QML canvases."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any, Dict, Optional, Sequence

import math

import numpy as np
from PyQt6.QtCore import (
    QAbstractListModel,
    QModelIndex,
    QObject,
    Qt,
    pyqtProperty,
    pyqtSignal,
    pyqtSlot,
)

from aim_fsm.utils import Pose
from aim_fsm.worldmap import ArucoMarkerObj, WallObj


RoleMap = Dict[int, bytes]
Item = Dict[str, Any]


def _role(index: int) -> int:
    return int(Qt.ItemDataRole.UserRole) + index


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _flatten_vector(value: Any) -> Sequence[float]:
    try:
        arr = np.asarray(value, dtype=float).reshape(-1)
        return [float(v) for v in arr]
    except Exception:
        try:
            return [float(value)]
        except Exception:
            return []


def _covariance_components(matrix: Any) -> tuple[float, float, float]:
    try:
        arr = np.asarray(matrix, dtype=float)
        if arr.ndim == 2 and arr.shape[0] >= 2 and arr.shape[1] >= 2:
            return float(arr[0, 0]), float(arr[0, 1]), float(arr[1, 1])
        flat = arr.reshape(-1)
        if flat.size >= 4:
            mat = flat[:4].reshape(2, 2)
            return float(mat[0, 0]), float(mat[0, 1]), float(mat[1, 1])
    except Exception:
        pass
    return 0.0, 0.0, 0.0


def _ellipse_axes(matrix: Any) -> tuple[float, float, float]:
    try:
        arr = np.asarray(matrix, dtype=float).reshape(2, 2)
        eigenvalues, eigenvectors = np.linalg.eigh(arr)
        eigenvalues = np.clip(eigenvalues, a_min=0.0, a_max=None)
        major_index = int(np.argmax(eigenvalues))
        major = math.sqrt(float(eigenvalues[major_index]))
        minor = math.sqrt(float(np.min(eigenvalues)))
        vec = eigenvectors[:, major_index]
        angle = math.atan2(float(vec[1]), float(vec[0]))
        return major, minor, angle
    except Exception:
        return 0.0, 0.0, 0.0


def _pose_components(source: Any) -> tuple[float, float, float]:
    if source is None:
        return 0.0, 0.0, 0.0

    if isinstance(source, Pose):
        return (
            _to_float(source.x, 0.0),
            _to_float(source.y, 0.0),
            _to_float(source.theta, 0.0),
        )

    if hasattr(source, "pose"):
        return _pose_components(getattr(source, "pose"))

    if isinstance(source, Mapping):
        return (
            _to_float(source.get("x"), 0.0),
            _to_float(source.get("y"), 0.0),
            _to_float(source.get("theta"), 0.0),
        )

    return (
        _to_float(getattr(source, "x", None), 0.0),
        _to_float(getattr(source, "y", None), 0.0),
        _to_float(getattr(source, "theta", None), 0.0),
    )


def _kind_from_id(name: str, world_obj: Any) -> str:
    if isinstance(world_obj, WallObj):
        return "wall"
    if isinstance(world_obj, ArucoMarkerObj):
        return "aruco"
    if name.startswith("Wall-"):
        return "wall"
    if name.startswith("ArucoMarker-"):
        return "aruco"
    if name.lower().startswith("video"):
        return "video"
    return "unknown"


def _label_from(name: str, world_obj: Any) -> str:
    if world_obj is not None:
        label = getattr(world_obj, "name", None)
        if label:
            return str(label)
    if name.startswith("ArucoMarker-"):
        return name.split("-", 1)[-1]
    if name.startswith("Wall-"):
        return name
    return name


class ParticleLayerModel(QAbstractListModel):
    """Flat list of particle state suitable for Canvas rendering."""

    countChanged = pyqtSignal()
    revisionChanged = pyqtSignal()

    ROLE_NAMES: tuple[str, ...] = (
        "id",
        "x",
        "y",
        "theta",
        "weight",
        "isBest",
    )

    _ROLE_MAP: RoleMap = {
        _role(i + 1): name.encode("utf-8") for i, name in enumerate(ROLE_NAMES)
    }

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._items: list[Item] = []
        self._revision = 0

    @pyqtProperty(int, notify=countChanged)
    def count(self) -> int:
        return len(self._items)

    @pyqtProperty(int, notify=revisionChanged)
    def revision(self) -> int:
        return self._revision

    # Qt overrides --------------------------------------------------

    def roleNames(self) -> RoleMap:  # type: ignore[override]
        return self._ROLE_MAP

    def rowCount(self, parent: QModelIndex | None = None) -> int:  # type: ignore[override]
        if parent is not None and parent.isValid():
            return 0
        return len(self._items)

    def data(self, index: QModelIndex, role: int = 0) -> Any:  # type: ignore[override]
        if not index.isValid():
            return None
        row = index.row()
        if row < 0 or row >= len(self._items):
            return None
        name = self._ROLE_MAP.get(role)
        if name is None:
            return None
        key = name.decode("utf-8")
        return self._items[row].get(key)

    # Public API ----------------------------------------------------

    def sync_from(self, particle_filter: Any) -> None:
        entries: list[Item] = []
        if particle_filter is not None:
            particles = getattr(particle_filter, "particles", None)
            if particles is None:
                particles = []
            best_particle = getattr(particle_filter, "best_particle", None)
            for index, particle in enumerate(particles):
                entry = {
                    "id": f"particle#{getattr(particle, 'index', index)}",
                    "x": _to_float(getattr(particle, "x", None), 0.0),
                    "y": _to_float(getattr(particle, "y", None), 0.0),
                    "theta": _to_float(getattr(particle, "theta", None), 0.0),
                    "weight": _to_float(getattr(particle, "weight", None), 0.0),
                    "isBest": bool(best_particle is particle),
                }
                entries.append(entry)

        self.beginResetModel()
        self._items = entries
        self.endResetModel()
        self._revision += 1
        self.countChanged.emit()
        self.revisionChanged.emit()

    @pyqtProperty("QVariantList", constant=True)
    def roles(self) -> list[str]:
        """Expose role names to QML for dynamic bindings."""
        return list(self.ROLE_NAMES)

    @pyqtSlot(int, result="QVariant")
    def get(self, row: int) -> Optional[Item]:
        if row < 0 or row >= len(self._items):
            return None
        return dict(self._items[row])


class ParticleSummary(QObject):
    """Aggregate metrics derived from the particle filter pose/variance."""

    changed = pyqtSignal()

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_theta = 0.0
        self._theta_variance = 0.0
        self._ellipse_major = 0.0
        self._ellipse_minor = 0.0
        self._ellipse_angle = 0.0
        self._is_valid = False

    @pyqtProperty(bool, notify=changed)
    def isValid(self) -> bool:
        return self._is_valid

    @pyqtProperty(float, notify=changed)
    def poseX(self) -> float:
        return self._pose_x

    @pyqtProperty(float, notify=changed)
    def poseY(self) -> float:
        return self._pose_y

    @pyqtProperty(float, notify=changed)
    def poseTheta(self) -> float:
        return self._pose_theta

    @pyqtProperty(float, notify=changed)
    def thetaVariance(self) -> float:
        return self._theta_variance

    @pyqtProperty(float, notify=changed)
    def ellipseMajor(self) -> float:
        return self._ellipse_major

    @pyqtProperty(float, notify=changed)
    def ellipseMinor(self) -> float:
        return self._ellipse_minor

    @pyqtProperty(float, notify=changed)
    def ellipseAngle(self) -> float:
        return self._ellipse_angle

    def sync_from(self, particle_filter: Any) -> None:
        pose_x = 0.0
        pose_y = 0.0
        pose_theta = 0.0
        theta_variance = 0.0
        ellipse_major = 0.0
        ellipse_minor = 0.0
        ellipse_angle = 0.0
        is_valid = False

        if particle_filter is not None:
            pose = getattr(particle_filter, "pose", None)
            if pose is None:
                update_pose = getattr(particle_filter, "update_pose_estimate", None)
                if callable(update_pose):
                    try:
                        pose = update_pose()
                    except Exception:
                        pose = None
            if pose is not None:
                pose_x = _to_float(getattr(pose, "x", None), 0.0)
                pose_y = _to_float(getattr(pose, "y", None), 0.0)
                pose_theta = _to_float(getattr(pose, "theta", None), 0.0)
                is_valid = True

            variance = getattr(particle_filter, "variance", None)
            if isinstance(variance, (tuple, list)) and len(variance) >= 2:
                xy_var, theta_var = variance[0], variance[1]
                theta_variance = _to_float(theta_var, 0.0)
                ellipse_major, ellipse_minor, ellipse_angle = _ellipse_axes(xy_var)

        self._pose_x = pose_x
        self._pose_y = pose_y
        self._pose_theta = pose_theta
        self._theta_variance = theta_variance
        self._ellipse_major = ellipse_major
        self._ellipse_minor = ellipse_minor
        self._ellipse_angle = ellipse_angle
        self._is_valid = is_valid
        self.changed.emit()


class LandmarkModel(QAbstractListModel):
    """Landmark collection combining SLAM state and world map objects."""

    countChanged = pyqtSignal()
    revisionChanged = pyqtSignal()

    ROLE_NAMES: tuple[str, ...] = (
        "id",
        "kind",
        "x",
        "y",
        "theta",
        "label",
        "seen",
        "source",
        "sigma_xx",
        "sigma_xy",
        "sigma_yy",
        "length_mm",
        "width_mm",
        "marker_id",
    )

    _ROLE_MAP: RoleMap = {
        _role(i + 1): name.encode("utf-8") for i, name in enumerate(ROLE_NAMES)
    }

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._items: list[Item] = []
        self._revision = 0

    @pyqtProperty(int, notify=countChanged)
    def count(self) -> int:
        return len(self._items)

    @pyqtProperty(int, notify=revisionChanged)
    def revision(self) -> int:
        return self._revision

    # Qt overrides --------------------------------------------------

    def roleNames(self) -> RoleMap:  # type: ignore[override]
        return self._ROLE_MAP

    def rowCount(self, parent: QModelIndex | None = None) -> int:  # type: ignore[override]
        if parent is not None and parent.isValid():
            return 0
        return len(self._items)

    def data(self, index: QModelIndex, role: int = 0) -> Any:  # type: ignore[override]
        if not index.isValid():
            return None
        row = index.row()
        if row < 0 or row >= len(self._items):
            return None
        name = self._ROLE_MAP.get(role)
        if name is None:
            return None
        key = name.decode("utf-8")
        return self._items[row].get(key)

    # Public API ----------------------------------------------------

    def sync_from(self, particle_filter: Any, world_map: Any = None) -> None:
        entries: list[Item] = []
        seen_ids: set[str] = set()

        sensor_model = getattr(particle_filter, "sensor_model", None)
        pf_landmarks: Mapping[str, Any] | Iterable[tuple[str, Any]]
        if sensor_model is not None:
            pf_landmarks = getattr(sensor_model, "landmarks", {}) or {}
        else:
            pf_landmarks = {}

        world_objects_by_string = {}
        if world_map is not None:
            snapshot = getattr(world_map, "snapshot_objects", None)
            try:
                if callable(snapshot):
                    world_objects = snapshot() or {}
                else:
                    world_objects = dict(getattr(world_map, "objects", {}) or {})
            except Exception:
                world_objects = {}
        else:
            world_objects = {}

        for obj in world_objects.values():
            if isinstance(obj, ArucoMarkerObj):
                world_objects_by_string[obj.marker_string] = obj
            obj_name = getattr(obj, "name", None)
            if isinstance(obj_name, str) and obj_name:
                world_objects_by_string[obj_name] = obj
            obj_id = getattr(obj, "id", None)
            if isinstance(obj_id, str) and obj_id:
                world_objects_by_string[obj_id] = obj
                
        if hasattr(pf_landmarks, "items"):
            lm_iterable = pf_landmarks.items()
        else:
            lm_iterable = pf_landmarks  # type: ignore[assignment]

        for name, spec in lm_iterable:
            entry = self._build_entry(str(name), spec, world_objects_by_string.get(name), source="slam")
            if entry:
                entries.append(entry)
                seen_ids.add(entry["id"])

        """
        for name, world_obj in world_objects.items():
            if name in seen_ids:
                continue
            if not isinstance(world_obj, (ArucoMarkerObj, WallObj)):
                continue
            entry = self._build_entry(str(name), world_obj, world_obj, source="world")
            if entry:
                entries.append(entry)

        entries.sort(key=lambda item: item["id"])
        """

        self.beginResetModel()
        self._items = entries
        self.endResetModel()
        self._revision += 1
        self.countChanged.emit()
        self.revisionChanged.emit()

    @pyqtSlot(int, result="QVariant")
    def get(self, row: int) -> Optional[Item]:
        if row < 0 or row >= len(self._items):
            return None
        return dict(self._items[row])

    def _build_entry(self, name: str, spec: Any, world_obj: Any, source: str) -> Optional[Item]:
        label = _label_from(name, world_obj)
        seen = bool(getattr(world_obj, "is_visible", False)) if world_obj is not None else False
        marker_id = getattr(world_obj, "marker_id", None) if world_obj is not None else None
        length_mm = getattr(world_obj, "length", None) if world_obj is not None else None
        width_mm = getattr(world_obj, "height", None) if world_obj is not None else None

        x = y = theta = 0.0
        sigma_xx = sigma_xy = sigma_yy = 0.0

        if isinstance(spec, Pose):
            x, y, theta = _pose_components(spec)
        elif isinstance(spec, (tuple, list)) and len(spec) >= 3:
            mu, orient, sigma = spec[0], spec[1], spec[2]
            coords = _flatten_vector(mu)
            if len(coords) >= 2:
                x, y = coords[0], coords[1]
            orientation = _flatten_vector(orient)
            if orientation:
                theta = orientation[-1]
            sigma_xx, sigma_xy, sigma_yy = _covariance_components(sigma)
        else:
            x, y, theta = _pose_components(spec)

        if marker_id is None and name.startswith("ArucoMarker-"):
            marker_token = name.split("-", 1)[-1]
            marker_id = marker_token.split(".", 1)[0]

        entry: Item = {
            "id": name,
            "kind": _kind_from_id(name, world_obj),
            "x": _to_float(x, 0.0),
            "y": _to_float(y, 0.0),
            "theta": _to_float(theta, 0.0),
            "label": label,
            "seen": seen,
            "source": source,
            "sigma_xx": sigma_xx,
            "sigma_xy": sigma_xy,
            "sigma_yy": sigma_yy,
            "length_mm": _to_float(length_mm, 0.0) if length_mm is not None else None,
            "width_mm": _to_float(width_mm, 0.0) if width_mm is not None else None,
            "marker_id": marker_id,
        }
        return entry
