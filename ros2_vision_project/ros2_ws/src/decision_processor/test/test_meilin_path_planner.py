from decision_processor import meilin_path_planner as mpp


def test_entry_kfs_forces_matching_entry():
    path = mpp.plan_path(entry_block=2, real_kfs=[3], fake_kfs=[8])

    assert mpp.get_pre_entry_pickups([3]) == [3]
    assert mpp.resolve_entry_block(2, [3], [8]) == 3
    assert path
    assert path[0] == 3


def test_entry_kfs_is_not_picked_again_inside_merlin():
    path = mpp.plan_path(entry_block=2, real_kfs=[3], fake_kfs=[8])
    pre_entry = mpp.get_pre_entry_pickups([3])
    in_merlin_kfs = [kfs for kfs in [3] if kfs not in pre_entry]

    assert in_merlin_kfs == []
    assert mpp.get_pickup_info(path, in_merlin_kfs) == {}


def test_non_entry_kfs_keeps_neighbor_pickup_behavior():
    path = mpp.plan_path(entry_block=2, real_kfs=[5], fake_kfs=[8])

    assert mpp.get_pre_entry_pickups([5]) == []
    assert path
    assert mpp.get_pickup_info(path, [5]) == {5: 4}


def test_transition_height_diff_uses_block_heights_not_coordinates():
    assert mpp.get_transition_height_diff_mm(2, 5) == 200
    assert mpp.get_transition_height_diff_mm(5, 8) == 200
    assert mpp.get_transition_height_diff_mm(8, 11) == -200
