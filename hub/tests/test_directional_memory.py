from __future__ import annotations

from focus_hub.directional_memory import DirectionalMemory


def test_first_visit_creates_a_node_and_wedge():
    m = DirectionalMemory()
    idx = m.update((10, 10), 90.0, 1.5)
    assert idx == 0
    assert m.history_nodes == [(10, 10)]
    assert m.history_count == [1]
    state = m.history_states[0]
    # ±39deg wedge centered at 90 -> indices [51, 128] inclusive set, rest untouched.
    assert state[50] == 0.0
    assert state[51] == 1.5
    assert state[128] == 1.5
    assert state[129] == 0.0
    assert m.history_score[0] == sum(state)


def test_nearby_revisit_merges_into_same_node():
    m = DirectionalMemory()
    m.update((10, 10), 90.0, 1.0)
    idx = m.update((11, 11), 100.0, 2.0)  # distance ~1.41 < 25 -> merges
    assert idx == 0
    assert len(m.history_nodes) == 1
    assert m.history_count == [2]


def test_far_visit_creates_a_new_node():
    m = DirectionalMemory()
    m.update((10, 10), 90.0, 1.0)
    idx = m.update((200, 200), 45.0, 0.5)
    assert idx == 1
    assert len(m.history_nodes) == 2
    assert m.history_count == [1, 1]


def test_wedge_wraps_around_zero_on_first_visit():
    m = DirectionalMemory()
    m.update((0, 0), 10.0, 1.0)  # c_angle=10 < 39 -> wraps
    state = m.history_states[0]
    # front part [0:10+39) and wrapped tail [360-10-39:360)
    assert state[0] == 1.0
    assert state[48] == 1.0
    assert state[49] == 0.0
    assert state[310] == 0.0
    assert state[311] == 1.0
    assert state[359] == 1.0


def test_closest_index_picks_the_nearest_within_threshold():
    m = DirectionalMemory()
    m.update((0, 0), 0.0, 1.0)
    m.update((100, 100), 0.0, 1.0)
    idx = m.update((99, 99), 0.0, 1.0)  # closer to the second node
    assert idx == 1
    assert m.history_count == [1, 2]
