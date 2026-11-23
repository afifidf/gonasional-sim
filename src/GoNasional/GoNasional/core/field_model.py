"""Field geometry and the landmark map used for localization.

Frame: origin at the centre mark, +x toward the goal we attack, +y to the left
when looking along +x. Landmark type ids match aroc_interfaces/Landmark.
"""

from dataclasses import dataclass

import numpy as np

GOALPOST = 1
PENALTY_MARK = 2
CENTER_MARK = 3
FIELD_CORNER = 4


@dataclass
class FieldModel:
    length: float = 9.0
    width: float = 6.0
    goal_width: float = 2.6
    penalty_mark_distance: float = 1.5

    def __post_init__(self):
        hl, hw, hg = self.length / 2.0, self.width / 2.0, self.goal_width / 2.0
        pm = hl - self.penalty_mark_distance
        self.landmarks = {
            GOALPOST: np.array([[hl, hg], [hl, -hg], [-hl, hg], [-hl, -hg]]),
            PENALTY_MARK: np.array([[pm, 0.0], [-pm, 0.0]]),
            CENTER_MARK: np.array([[0.0, 0.0]]),
            FIELD_CORNER: np.array([[hl, hw], [hl, -hw], [-hl, hw], [-hl, -hw]]),
        }

    @property
    def opponent_goal(self):
        return (self.length / 2.0, 0.0)

    @property
    def own_goal(self):
        return (-self.length / 2.0, 0.0)

    def inside(self, x, y, margin=0.0):
        return (abs(x) <= self.length / 2.0 + margin
                and abs(y) <= self.width / 2.0 + margin)

    def aim_point(self, ball_y, post_margin=0.35):
        """Point on the opponent goal line to shoot at.

        The nearest point of the goal mouth to the ball's lateral position,
        kept `post_margin` inside the posts, so a ball already in front of the
        goal is shot straight and a ball out wide is aimed at the near side.
        """
        half = max(0.0, self.goal_width / 2.0 - post_margin)
        return (self.length / 2.0, float(np.clip(ball_y, -half, half)))
