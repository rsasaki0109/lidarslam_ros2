"""Contracts for the ground-truth-free v40 GBA graph audit."""

import importlib.util
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'audit_v40_gba', ROOT / 'scripts/audit_v40_gba_graph_contract.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_keyframe_selection_replays_nonoverlapping_v21_windows():
    positions = [np.asarray([float(index), 0.0, 0.0])
                 for index in range(25)]
    assert MODULE.select_keyframe_scan_ids(
        positions, window_size=5, keyframe_stride=1,
        min_keyframe_distance_m=6.0) == [4, 14, 24]


def test_scan_id_contract_accepts_sparse_ids_and_rejects_unsafe_ids():
    contiguous = MODULE.validate_keyframe_scan_ids(5, [0, 1, 2])
    assert contiguous['valid_scan_index_namespace'] is True
    assert contiguous['dense_keyframe_ordinal_namespace'] is True

    dropped = MODULE.validate_keyframe_scan_ids(7, [0, 2, 6])
    assert dropped['valid_scan_index_namespace'] is True
    assert dropped['dense_keyframe_ordinal_namespace'] is False

    duplicate = MODULE.validate_keyframe_scan_ids(7, [0, 2, 2])
    assert duplicate['valid_scan_index_namespace'] is False
    assert duplicate['duplicates'] == [2]

    out_of_range = MODULE.validate_keyframe_scan_ids(7, [0, 7])
    assert out_of_range['valid_scan_index_namespace'] is False
    assert out_of_range['out_of_range'] == [7]


def test_sparse_v21_graph_exposes_missing_dense_keys():
    contract = MODULE.graph_contract(20, [4, 14])
    sparse = contract['v21_sparse_graph']
    assert sparse['initial_keys'] == 2
    assert sparse['declared_dense_writeback_keys'] == 15
    assert sparse['missing_initial_keys_in_declared_range'] == 13
    assert sparse['trailing_scans_outside_declared_range'] == 5
    assert sparse['structurally_valid'] is False
    full = contract['full_scan_graph']
    assert full['initial_keys'] == 20
    assert full['odometry_chain_edges'] == 19
    assert full['anchored_prior_present'] is True


def test_source_audit_distinguishes_scan_ids_from_sparse_graph_keys():
    base = '''
      smp->id = buf_base - 1;
      gba_edges.push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
      initial.insert(j, pose3);
      add_edge(j-1, j, old_pose, new_pose, graph, noise);
      graph.addPrior(i, pose3, fixd_noise);
      for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++) {}
    '''
    v21 = '''
      initial.insert(kf->id, pose3);
      stepsizes.push_back(max_kf_id + 1);
      topDownProcess(initial, graph, ids, stepsizes);
    '''
    assert all(MODULE.audit_source_contract(base, v21).values())
