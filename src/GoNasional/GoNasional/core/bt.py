"""Minimal reactive behavior tree. No ROS imports, no external dependency."""

from enum import Enum


class Status(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    RUNNING = 'running'


class Node:
    def __init__(self, name):
        self.name = name

    def tick(self, bb):
        raise NotImplementedError


class Condition(Node):
    """Leaf that succeeds when predicate(bb) is true."""

    def __init__(self, name, predicate):
        super().__init__(name)
        self.predicate = predicate

    def tick(self, bb):
        return Status.SUCCESS if self.predicate(bb) else Status.FAILURE


class Action(Node):
    """Leaf that runs fn(bb); fn returns a Status, or None meaning RUNNING."""

    def __init__(self, name, fn):
        super().__init__(name)
        self.fn = fn

    def tick(self, bb):
        result = self.fn(bb)
        bb.active_leaf = self.name
        return Status.RUNNING if result is None else result


class Sequence(Node):
    """Ticks children in order every time; stops at the first non-success."""

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status is not Status.SUCCESS:
                return status
        return Status.SUCCESS


class Selector(Node):
    """Ticks children in order every time; stops at the first non-failure.

    Re-evaluating from the left each tick is what makes the tree reactive:
    a higher-priority branch takes over as soon as its condition holds.
    """

    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status is not Status.FAILURE:
                return status
        return Status.FAILURE
