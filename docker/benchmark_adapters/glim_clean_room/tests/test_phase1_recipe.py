#!/usr/bin/env python3
"""Adversarial offline checks for the additive Phase 1 r2 recipe."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ADAPTER_ROOT = Path(__file__).resolve().parents[1]
PHASE1_ROOT = ADAPTER_ROOT / "phase1"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError("cannot load " + str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


validator = _load_module("phase1_recipe_validator", PHASE1_ROOT / "validate_phase1_recipe.py")
tree_hash = _load_module("boost_tree_hash", PHASE1_ROOT / "boost_tree_hash.py")


class Phase1RecipeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = json.loads(
            (PHASE1_ROOT / "source_closure.json").read_text(encoding="utf-8"))

    def test_current_manifest_and_surface_pass(self) -> None:
        validator._check_manifest(self.manifest)
        validator._check_recipe_surface()

    def test_missing_boost_source_is_rejected(self) -> None:
        candidate = copy.deepcopy(self.manifest)
        del candidate["boost_source"]
        with self.assertRaises(ValueError):
            validator._check_manifest(candidate)

    def test_moving_boost_url_is_rejected(self) -> None:
        candidate = copy.deepcopy(self.manifest)
        candidate["boost_source"]["release_url"] = (
            "https://archives.boost.io/release/1.83.0/source/current.tar.bz2")
        with self.assertRaises(ValueError):
            validator._check_manifest(candidate)

    def test_boost_archive_or_tree_drift_is_rejected(self) -> None:
        for field in ("archive_sha256", "source_tree_sha256"):
            candidate = copy.deepcopy(self.manifest)
            candidate["boost_source"][field] = "0" * 64
            with self.subTest(field=field), self.assertRaises(ValueError):
                validator._check_manifest(candidate)

    def test_tree_hash_rejects_symlink_and_hardlink_ambiguity(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase1-tree-fixture-") as directory:
            root = Path(directory)
            regular = root / "regular.txt"
            regular.write_text("fixture\n", encoding="utf-8")
            link = root / "symlink.txt"
            link.symlink_to(regular.name)
            with self.assertRaises(ValueError):
                tree_hash._tree_hash(root)

        with tempfile.TemporaryDirectory(prefix="phase1-tree-fixture-") as directory:
            parent = Path(directory)
            real_root = parent / "real"
            real_root.mkdir()
            (real_root / "regular.txt").write_text("fixture\n", encoding="utf-8")
            linked_root = parent / "linked"
            linked_root.symlink_to(real_root, target_is_directory=True)
            with self.assertRaises(ValueError):
                tree_hash._tree_hash(linked_root)

        with tempfile.TemporaryDirectory(prefix="phase1-tree-fixture-") as directory:
            root = Path(directory)
            regular = root / "regular.txt"
            regular.write_text("fixture\n", encoding="utf-8")
            hardlink = root / "hardlink.txt"
            hardlink.hardlink_to(regular)
            with self.assertRaises(ValueError):
                tree_hash._tree_hash(root)


if __name__ == "__main__":
    unittest.main()
