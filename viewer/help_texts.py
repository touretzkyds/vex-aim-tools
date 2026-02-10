"""Shared help text resources for Qt camera views."""

from __future__ import annotations

CAMERA_HELP_TEXT = (
    "Camera viewer help:\n"
    "    Type 'c' to toggle crosshairs.\n"
    "    Type 's' to take a snapshot of the raw camera image.\n"
    "    Type 'S' to take an annotated snapshot (includes bounding boxes and crosshairs).\n"
    "    Press Esc to close the viewer.\n"
    "    Type 'h' to toggle this help overlay."
)

WORLD_HELP_TEXT = (
    "World viewer keyboard commands:\n"
    "  w/a/s/d       Translate focus forward/left/back/right\n"
    "  q/e or PgUp/PgDn Raise/lower focus\n"
    "  </>           Zoom in/out (orbit radius)\n"
    "  ↑/↓ or i/k   Pitch camera up/down\n"
    "  ←/→ or j/l   Yaw camera left/right\n"
    "  x             Toggle axes\n"
    "  z             Reset camera and focus\n"
    "  h             Print this help text\n"
)

PARTICLE_HELP_TEXT = (
    "Particle viewer commands:\n"
    "  w/a/s/d    Drive robot +/- 10 mm or turn +/- 22.5 degrees\n"
    "  W/A/S/D    Drive robot +/- 40 mm or turn +/- 90 degrees\n"
    "  j/k/J/K    Strafe left/right by 10 or 40 mm\n"
    "  e/r        Evaluate particles / resample\n"
    "  m          Update occupancy grid from current camera frame\n"
    "  z/Z        Reset particles / jitter\n"
    "  c          Clear landmarks (SLAM)\n"
    "  l          Show landmarks\n"
    "  o          Show objects\n"
    "  p/P        Show pose / best particle\n"
    "  v/V        Toggle verbose mode / display weight variance\n"
    "  arrows     Translate the view\n"
    "  Home       Center the view (zero translation)\n"
    "  </>        Zoom view in/out\n"
    "  $          Toggle redisplay (auto refresh)\n"
    "  Space      Toggle auto-centering\n"
    "  h          Print this help text\n"
    "  Esc/Q      Close the viewer\n"
)

PATH_HELP_TEXT = (
    "Path viewer commands:\n"
    "  arrows   Translate the view up/down/left/right\n"
    "  Home     Center the view (zero translation)\n"
    "  </>      Zoom in/out\n"
    "  o        Show objects\n"
    "  b        Show obstacles\n"
    "  p        Show pose\n"
    "  space    Toggle redisplay (for debugging)\n"
    "  h        Print this help text\n"
    "  Esc/Q    Close the viewer\n"
)

DEPTH_HELP_TEXT = (
    "Depth viewer commands:\n"
    "  Space    Toggle live refresh on/off\n"
    "  g        Toggle gradient source (camera/depth)\n"
    "  r        Refresh one frame (manual)\n"
    "  Default gradient source: camera image (right/down shift differencing)\n"
    "  h        Toggle this help overlay\n"
    "  Esc/Q    Close the viewer\n"
)

__all__ = [
    "CAMERA_HELP_TEXT",
    "WORLD_HELP_TEXT",
    "PARTICLE_HELP_TEXT",
    "PATH_HELP_TEXT",
    "DEPTH_HELP_TEXT",
]
