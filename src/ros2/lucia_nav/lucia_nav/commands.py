"""
commands.py

The words the mission node accepts on /mission/command, parsed without ROS so
they can be tested anywhere:

    explore     frontier exploration until nothing is left, then home
    home        drive back to where mapping started
    goto X Y    drive to map point (X, Y), in meters
    stop        cancel whatever it's doing and stop the wheels
"""

import math

SIMPLE = ('explore', 'home', 'stop')


def parse_command(text):
    """
    ('explore',), ('home',), ('stop',) or ('goto', x, y); None if the text
    isn't a valid command. Case and extra spaces don't matter.
    """
    if not isinstance(text, str):
        return None
    words = text.strip().lower().split()
    if len(words) == 1 and words[0] in SIMPLE:
        return (words[0],)
    if len(words) == 3 and words[0] == 'goto':
        try:
            x, y = float(words[1]), float(words[2])
        except ValueError:
            return None
        if math.isfinite(x) and math.isfinite(y):
            return ('goto', x, y)
    return None
