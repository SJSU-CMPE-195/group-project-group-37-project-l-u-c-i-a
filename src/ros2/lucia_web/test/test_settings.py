"""Tests for settings.py. Run from src/ros2/lucia_web:  python3 -m pytest test"""

import yaml

from lucia_web.settings import (BY_KEY, SETTINGS, clean, defaults, describe, load, params_yaml,
                                ros_params, save)


def test_defaults_check_out_and_match_the_code():
    settings, errors = clean(defaults())
    assert errors == []
    d = defaults()
    # The values the nodes had before the settings menu existed
    assert d['top_speed'] == 0.30 and d['tight_min_speed'] == 0.10 and d['robot_radius'] == 0.23
    assert (d['linear_accel'], d['linear_decel'], d['angular_accel'], d['angular_decel']) == (0.3, 0.6, 1.5, 3.0)
    assert d['max_stuck'] == 3 and d['use_imu'] is True


def test_every_setting_describes_itself():
    for s in describe():
        assert s['label'] and s['group'] and s['kind'] in ('float', 'int', 'bool')
        if s['kind'] != 'bool':
            assert s['min'] <= s['default'] <= s['max'], s['key']


def test_numbers_from_the_page_are_typed():
    settings, errors = clean({'top_speed': '0.4', 'max_stuck': 5.0, 'stuck_check': False})
    assert errors == []
    assert settings['top_speed'] == 0.4 and isinstance(settings['max_stuck'], int) and settings['max_stuck'] == 5
    assert settings['stuck_check'] is False


def test_bad_values_are_refused_with_a_reason():
    _, errors = clean({'top_speed': 0.9, 'max_stuck': 2.5, 'use_imu': 'yes', 'linear_accel': 'fast',
                       'goal_timeout': float('nan')})
    text = ' '.join(errors)
    assert 'Speed limit: between 0.1 and 0.5 m/s' in text
    assert 'whole number' in text and 'on or off' in text
    assert len(errors) == 5


def test_settings_that_must_agree():
    _, errors = clean({'top_speed': 0.15, 'tight_min_speed': 0.2})
    assert errors and 'speed limit' in errors[0]
    _, errors = clean({'tight_near_slow': 0.4, 'tight_near_fast': 0.3})
    assert errors
    _, errors = clean({'tight_ahead_slow': 0.8, 'tight_ahead_fast': 0.8})
    assert errors


def test_unknown_keys_dropped_missing_ones_default():
    settings, errors = clean({'nope': 1, 'top_speed': 0.25})
    assert errors == [] and 'nope' not in settings
    assert settings['top_speed'] == 0.25 and settings['robot_radius'] == 0.23


def test_save_and_load(tmp_path):
    path = str(tmp_path / 'settings.json')
    assert load(path) == defaults()                         # no file yet
    settings, _ = clean({'top_speed': 0.4})
    save(path, settings)
    assert load(path)['top_speed'] == 0.4
    (tmp_path / 'settings.json').write_text('{"top_speed": 99}')
    assert load(path) == defaults()                         # doesn't check out: defaults


def test_ros_params_have_the_types_the_nodes_declared():
    p = ros_params(defaults())
    bridge = p['roomba_bridge']['ros__parameters']
    assert bridge['max_speed_mm_s'] == 300.0 and isinstance(bridge['max_speed_mm_s'], float)
    gate = p['safety_gate']['ros__parameters']
    assert gate['tight_space'] is True and gate['min_speed_mps'] == 0.10 and gate['near_fast_m'] == 0.35
    mission = p['mission']['ros__parameters']
    assert isinstance(mission['max_stuck'], int) and isinstance(mission['goal_timeout_s'], float)
    assert mission['clearance_m'] == 0.25                   # robot radius + 2 cm, as before
    local = p['local_costmap']['local_costmap']['ros__parameters']
    assert local['robot_radius'] == 0.23 and local['inflation_layer']['inflation_radius'] == 0.45


def test_params_yaml_is_plain_yaml_without_aliases():
    text = params_yaml(defaults())
    assert '&' not in text and '*' not in text              # the ROS parameter parser wants plain YAML
    data = yaml.safe_load(text)
    assert data == ros_params(defaults())
    assert 'max_speed_mm_s: 300.0' in text                  # a float stays a float


def test_robot_radius_reaches_every_node():
    settings, _ = clean({'robot_radius': 0.26})
    p = ros_params(settings)
    assert p['safety_gate']['ros__parameters']['robot_radius_m'] == 0.26
    assert p['mission']['ros__parameters']['robot_radius_m'] == 0.26
    assert abs(p['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']['inflation_radius'] - 0.48) < 1e-9


def test_keys_are_unique():
    assert len(BY_KEY) == len(SETTINGS)
