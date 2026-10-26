"""
settings.py

The settings the web page shows when you press Start mapping: values that used
to be fixed in the code (speed, acceleration, the tight-space slowdown, robot
size, Explore, the stuck check). Kept free of ROS so they can be tested anywhere.

The page fills the form from SETTINGS (each with its default, range and a
line of help), the person changes what they want, and Start mapping sends the
values back. web_ui then:
- checks them (clean),
- saves them next to the maps (settings.json), so the next Start shows them,
- writes them as a ROS 2 parameters file (params_yaml) that mapping.launch.py
  hands to every node it starts, after each node's own settings,
- and applies the few it uses itself (speed limit for the slider, robot size
  for drawing, the wait before Explore carries on after driving by hand).

ROS 2 parameters are typed: a node that declared 300.0 rejects 300 and doesn't
start. params_yaml writes each value as the type its node declared.
"""

import json
import math
import os

import yaml


class Setting:
    def __init__(self, key, group, label, kind, default, low=None, high=None, step=None, unit='', help=''):
        self.key, self.group, self.label, self.kind = key, group, label, kind
        self.default, self.low, self.high, self.step = default, low, high, step
        self.unit, self.help = unit, help

    def as_dict(self):
        return {'key': self.key, 'group': self.group, 'label': self.label, 'kind': self.kind,
                'default': self.default, 'min': self.low, 'max': self.high, 'step': self.step,
                'unit': self.unit, 'help': self.help}


SETTINGS = [
    # Driving
    Setting('top_speed', 'Driving', 'Speed limit', 'float', 0.30, 0.10, 0.50, 0.01, 'm/s',
            'The fastest it may go; the Top speed slider goes up to this. The Roomba itself tops out at 0.50.'),
    Setting('linear_accel', 'Driving', 'Speeding up', 'float', 0.3, 0.05, 1.0, 0.05, 'm/s²',
            'How quickly it gets going. 0.3: from a stop to 15 cm/s in half a second.'),
    Setting('linear_decel', 'Driving', 'Slowing down', 'float', 0.6, 0.1, 2.0, 0.05, 'm/s²',
            'How quickly it stops, STOP included. 0.6: from 15 cm/s in 0.3 s (about 2 cm).'),
    Setting('angular_accel', 'Driving', 'Turning, speeding up', 'float', 1.5, 0.2, 5.0, 0.1, 'rad/s²', ''),
    Setting('angular_decel', 'Driving', 'Turning, slowing down', 'float', 3.0, 0.5, 8.0, 0.1, 'rad/s²', ''),
    # Robot and sensors
    Setting('use_imu', 'Robot', 'Use the IMU', 'bool', True,
            help='Heading from the IMU (straighter maps). Off: wheels only, if the IMU is unplugged.'),
    Setting('robot_radius', 'Robot', 'Robot radius', 'float', 0.23, 0.15, 0.40, 0.01, 'm',
            'Center to its furthest edge (18 in across = 0.23). For planning, the tight-space slowdown and drawing.'),
    # Tight spaces (safety_gate)
    Setting('tight_space', 'Tight spaces', 'Slow down in tight spaces', 'bool', True,
            help='Full speed in the open, slower close to things (from the lidar). Never stops on its own.'),
    Setting('tight_min_speed', 'Tight spaces', 'Slowest speed', 'float', 0.10, 0.03, 0.30, 0.01, 'm/s',
            'How slow it gets at the closest.'),
    Setting('tight_near_fast', 'Tight spaces', 'Full speed with this much room around it', 'float',
            0.35, 0.10, 1.50, 0.05, 'm', 'Room beside and in front of it, measured from its edge.'),
    Setting('tight_near_slow', 'Tight spaces', 'Slowest with something this close', 'float',
            0.05, 0.0, 0.50, 0.01, 'm', ''),
    Setting('tight_ahead_fast', 'Tight spaces', 'Full speed with this much clear ahead', 'float',
            0.8, 0.2, 3.0, 0.05, 'm', 'Free run straight ahead, in a lane as wide as the robot.'),
    Setting('tight_ahead_slow', 'Tight spaces', 'Slowest with this little clear ahead', 'float',
            0.2, 0.0, 1.0, 0.05, 'm', ''),
    # Explore (mission)
    Setting('return_home', 'Explore', 'Go HOME when done', 'bool', True),
    Setting('goal_timeout', 'Explore', 'Give up on a frontier after', 'float', 90.0, 20.0, 600.0, 5.0, 's', ''),
    Setting('min_frontier', 'Explore', 'Ignore frontiers shorter than', 'float', 0.30, 0.10, 2.0, 0.05, 'm',
            'Shorter edges of the map are mostly speckle.'),
    # Stuck check (mission)
    Setting('stuck_check', 'Stuck check', 'Stuck check', 'bool', True,
            help='Notices when the wheels turn but the robot isn\'t moving, backs up and tries elsewhere.'),
    Setting('backup_m', 'Stuck check', 'Back up', 'float', 0.15, 0.05, 0.50, 0.05, 'm', ''),
    Setting('max_stuck', 'Stuck check', 'Ask for help after this many in a row', 'int', 3, 1, 10, 1, '',
            'Then it stops and plays a tune for help.'),
    # Driving by hand (web_ui)
    Setting('resume_after', 'Driving by hand', 'Carry on after the last key press', 'float', 3.0, 0.0, 30.0, 0.5, 's',
            'Explore / HOME pick up again this long after you stop driving by hand. 0 = never.'),
]

BY_KEY = {s.key: s for s in SETTINGS}

# Fixed parts of what the robot size feeds (nav2.yaml has the same margins)
INFLATION_MARGIN_M = 0.22       # paths get more expensive up to this far beyond the robot's edge
CLEARANCE_MARGIN_M = 0.02       # Explore keeps the robot's middle this far beyond its radius from walls


def defaults():
    return {s.key: s.default for s in SETTINGS}


def describe():
    """Everything the page needs to build the form."""
    return [s.as_dict() for s in SETTINGS]


def _value(setting, raw):
    """raw -> the setting's type, or ValueError with a message for the person."""
    if setting.kind == 'bool':
        if isinstance(raw, bool):
            return raw
        raise ValueError(f'{setting.label}: on or off')
    if isinstance(raw, bool):
        raise ValueError(f'{setting.label}: a number')
    try:
        number = float(raw)
    except (TypeError, ValueError):
        raise ValueError(f'{setting.label}: a number') from None
    if not math.isfinite(number):
        raise ValueError(f'{setting.label}: a number')
    if setting.kind == 'int':
        if number != int(number):
            raise ValueError(f'{setting.label}: a whole number')
        number = int(number)
    if not setting.low <= number <= setting.high:
        unit = f' {setting.unit}' if setting.unit else ''
        raise ValueError(f'{setting.label}: between {setting.low} and {setting.high}{unit}')
    return number


def clean(values):
    """
    (settings, errors): the values checked and typed, with defaults for any
    that are missing; unknown keys are dropped. errors is a list of messages;
    when it isn't empty, don't use the settings.
    """
    if not isinstance(values, dict):
        return defaults(), ['Settings must be a list of name: value']
    out, errors = defaults(), []
    for key, raw in values.items():
        setting = BY_KEY.get(key)
        if setting is None:
            continue
        try:
            out[key] = _value(setting, raw)
        except ValueError as exc:
            errors.append(str(exc))
    if not errors:
        if out['tight_min_speed'] > out['top_speed']:
            errors.append('Tight spaces: the slowest speed is above the speed limit')
        if out['tight_near_slow'] >= out['tight_near_fast']:
            errors.append('Tight spaces: "slowest" room must be less than "full speed" room')
        if out['tight_ahead_slow'] >= out['tight_ahead_fast']:
            errors.append('Tight spaces: "slowest" clear-ahead must be less than "full speed" clear-ahead')
    return out, errors


def load(path):
    """Saved settings, checked; defaults if there's no file or it doesn't check out."""
    try:
        with open(path) as f:
            values = json.load(f)
    except (OSError, ValueError):
        return defaults()
    settings, errors = clean(values)
    return defaults() if errors else settings


def save(path, settings):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(settings, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def ros_params(s):
    """
    The settings as ROS 2 parameters for the nodes mapping.launch.py starts,
    keyed by node name (Nav2's costmaps are nested: local_costmap/local_costmap).
    Each value has the type its node declared.
    """
    radius = float(s['robot_radius'])

    def costmap():      # a fresh dict each time: shared ones come out as YAML aliases
        return {'ros__parameters': {'robot_radius': radius,
                                    'inflation_layer': {'inflation_radius': radius + INFLATION_MARGIN_M}}}
    return {
        'roomba_bridge': {'ros__parameters': {
            'max_speed_mm_s': float(s['top_speed']) * 1000.0,
            'max_linear_accel': float(s['linear_accel']),
            'max_linear_decel': float(s['linear_decel']),
            'max_angular_accel': float(s['angular_accel']),
            'max_angular_decel': float(s['angular_decel']),
        }},
        'safety_gate': {'ros__parameters': {
            'tight_space': bool(s['tight_space']),
            'max_speed_mps': float(s['top_speed']),
            'min_speed_mps': float(s['tight_min_speed']),
            'near_slow_m': float(s['tight_near_slow']),
            'near_fast_m': float(s['tight_near_fast']),
            'ahead_slow_m': float(s['tight_ahead_slow']),
            'ahead_fast_m': float(s['tight_ahead_fast']),
            'robot_radius_m': radius,
        }},
        'mission': {'ros__parameters': {
            'robot_radius_m': radius,
            'clearance_m': radius + CLEARANCE_MARGIN_M,
            'return_home_when_done': bool(s['return_home']),
            'goal_timeout_s': float(s['goal_timeout']),
            'min_frontier_m': float(s['min_frontier']),
            'stuck_check': bool(s['stuck_check']),
            'backup_m': float(s['backup_m']),
            'max_stuck': int(s['max_stuck']),
        }},
        'local_costmap': {'local_costmap': costmap()},
        'global_costmap': {'global_costmap': costmap()},
    }


def params_yaml(s):
    """ros_params as the text of a ROS 2 parameters file."""
    return ('# Written by lucia_web when mapping starts, from the settings on the page.\n'
            + yaml.safe_dump(ros_params(s), default_flow_style=False, sort_keys=True))
