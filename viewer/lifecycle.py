"""Shared viewer lifecycle helpers used by the CLI and program startup."""

from __future__ import annotations

from typing import Any, Callable


def viewer_is_visible(viewer: Any) -> bool:
    """Return True when the viewer exists and its window is currently visible."""

    if viewer is None or viewer is True:
        return False

    probe = getattr(viewer, "is_visible", None)
    if callable(probe):
        try:
            return bool(probe())
        except Exception:
            return False
    else:
        return False


def ensure_viewer(target: Any, attr: str, factory: Callable[[], Any]) -> Any:
    """Create or relaunch a viewer stored on ``target``."""

    if target is None:
        return None

    viewer = getattr(target, attr, None)
    if viewer is not None and viewer is not True:
        if viewer_is_visible(viewer):
            return viewer

        start = getattr(viewer, "start", None)
        if callable(start):
            start()
        return viewer

    viewer = factory()
    start = getattr(viewer, "start", None)
    if callable(start):
        start()
    setattr(target, attr, viewer)
    return viewer

def stop_timer_if_view_hidden(view: Any, timer: Any) -> None:
    """Stop ``timer`` when ``view`` is hidden, ignoring Qt teardown races."""

    try:
        visible = bool(view.isVisible())
    except RuntimeError:
        return
    except Exception:
        return

    if visible:
        return

    try:
        timer.stop()
    except RuntimeError:
        return
    except Exception:
        return


__all__ = ["ensure_viewer", "viewer_is_visible", "stop_timer_if_view_hidden"]
