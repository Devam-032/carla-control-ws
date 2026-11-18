#!/usr/bin/env python3
"""
On-the-fly command parsing shared by the Stanley and LQR controllers.

Commands are published as std_msgs/String on <node>/command:

    go [trajectory_type] [L=<m>] [R=<m>] [v=<m/s>] [ds=<m>]
        Re-plan from the CURRENT ego pose and start tracking.
    stop
        Brake and hold (node stays alive, waiting for the next 'go').
    speed <m/s>
        Change v_ref while driving.
"""

TRAJECTORY_TYPES = ("straight", "right_turn", "s_curve")

# short key -> controller attribute
KEYS = {
    "L": "total_length",
    "R": "turn_radius",
    "v": "v_ref",
    "ds": "waypoint_spacing",
}


def parse_command(text):
    """
    Returns (verb, updates) where updates maps controller attribute -> value.
    Raises ValueError on malformed input.
    """
    tokens = text.strip().split()
    if not tokens:
        raise ValueError("empty command")

    verb = tokens[0].lower()
    updates = {}

    if verb == "stop":
        return verb, updates

    if verb == "speed":
        if len(tokens) != 2:
            raise ValueError("usage: speed <m/s>")
        updates["v_ref"] = float(tokens[1])
        return verb, updates

    if verb == "go":
        for tok in tokens[1:]:
            if "=" in tok:
                key, val = tok.split("=", 1)
                if key not in KEYS:
                    raise ValueError(f"unknown key '{key}' (use {', '.join(KEYS)})")
                updates[KEYS[key]] = float(val)
            elif tok.lower() in TRAJECTORY_TYPES:
                updates["trajectory_type"] = tok.lower()
            else:
                raise ValueError(f"unknown token '{tok}'")
        return verb, updates

    raise ValueError(f"unknown command '{verb}' (use go / stop / speed)")
