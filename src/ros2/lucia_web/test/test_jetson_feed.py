"""Tests for logic.JetsonFeed: the Jetson's detection messages."""

import json

from lucia_web.logic import JetsonFeed


def msg(people, sign=None, seq=1):
    return json.dumps({'seq': seq, 'people': people, 'sign': sign}).encode()


def test_people_with_distances():
    f = JetsonFeed()
    assert f.feed(msg([{'id': 3, 'x': 0.3, 'y': -0.2, 'z': 1.4, 'conf': 0.91}]), 10.0)
    s = f.status(10.5)
    assert s['people'] == [{'id': 3, 'dist_m': 1.45, 'conf': 0.91}]
    assert s['age_s'] == 0.5 and s['sign'] is None


def test_a_sign_is_kept_after_the_message_that_carried_it():
    f = JetsonFeed()
    f.feed(msg([], sign={'id': 3, 'kind': 'follow'}), 10.0)
    f.feed(msg([]), 10.1)
    s = f.status(12.0)
    assert s['sign'] == {'id': 3, 'kind': 'follow'} and s['sign_age_s'] == 2.0


def test_rate():
    f = JetsonFeed()
    for k in range(20):
        f.feed(msg([]), 10.0 + k * 0.1)
    assert f.status(12.0)['hz'] == 10.0


def test_garbage_is_counted_not_kept():
    f = JetsonFeed()
    assert not f.feed(b'not json', 1.0)
    assert not f.feed(b'[1, 2]', 1.0)
    assert not f.feed(json.dumps({'people': 'nobody'}).encode(), 1.0)
    assert f.bad == 3 and f.status(2.0) is None
    # a broken person entry is skipped, the rest kept
    assert f.feed(msg([{'id': 1}, {'id': 2, 'x': 0, 'y': 0, 'z': 2}]), 3.0)
    assert [p['id'] for p in f.status(3.0)['people']] == [2]
