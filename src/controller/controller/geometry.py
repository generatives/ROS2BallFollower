import math

def heading_diff(current_heading, goal_heading):
    diff = goal_heading - current_heading

    if diff > math.pi:
        return diff - (2 * math.pi)

    if diff < -math.pi:
        return diff - (-2 * math.pi)

    return diff