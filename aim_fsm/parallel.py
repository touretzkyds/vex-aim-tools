"""Simple parallel composition for StateNode behaviors."""

from __future__ import annotations

from typing import Iterable, List

from .base import StateNode
from .events import CompletionEvent, FailureEvent


class Parallel(StateNode):
    """Run multiple child StateNodes concurrently.

    Marks completion when all children complete, or failure as soon as one fails.
    """

    def __init__(self, *children: StateNode):
        super().__init__()
        self.children_nodes: List[StateNode] = list(children)
        self._completed: set[StateNode] = set()
        self._failed: bool = False

    def set_children(self, children: Iterable[StateNode]) -> "Parallel":
        self.children_nodes = list(children)
        return self

    def set_robot(self, robot) -> "Parallel":
        self._robot = robot
        for child in self.children_nodes:
            child._robot = robot
        return self

    def start(self, event=None):
        if self.running:
            return self.running
        self._completed.clear()
        self._failed = False
        super().start(event)
        for child in self.children_nodes:
            # Keep parent links for introspection only; transitions are not used here.
            child.parent = self
            if getattr(child, "_robot", None) is None:
                child._robot = self.robot
            # Listen for completion/failure from each child
            self.robot.erouter.add_listener(self, CompletionEvent, child)
            self.robot.erouter.add_listener(self, FailureEvent, child)
            child.start(event)
        return self.running

    def handle_event(self, event):
        if isinstance(event, CompletionEvent):
            self._completed.add(event.source)
            if len(self._completed) == len(self.children_nodes):
                self.post_completion()
                for child in self.children_nodes:
                    try:
                        child.stop()
                    except Exception:
                        pass
        elif isinstance(event, FailureEvent):
            if not self._failed:
                self._failed = True
                self.post_failure()
                for child in self.children_nodes:
                    try:
                        child.stop()
                    except Exception:
                        pass

    def poll(self):
        # If still running children, stay in running state.
        return self.running

    def stop(self):
        for child in self.children_nodes:
            try:
                child.stop()
            except Exception:
                pass
        super().stop()
