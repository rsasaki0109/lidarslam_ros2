#!/usr/bin/env python3
"""Adversarial parser and receipt checks for the Phase 1 container evidence."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import unittest


ADAPTER_ROOT = Path(__file__).resolve().parents[1]
PHASE1_ROOT = ADAPTER_ROOT / "phase1"
EVIDENCE_PATH = PHASE1_ROOT / "container_evidence.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("phase1_container_evidence", EVIDENCE_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError("cannot load " + str(EVIDENCE_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


evidence = _load_module()


class Phase1ContainerEvidenceTest(unittest.TestCase):
    def test_old_set_x_grep_artifact_is_not_counted(self) -> None:
        text = """LDD_REPORT
### /opt/build/smoke
\tlibc.so.6 => /lib/libc.so.6
+ test 0 = 0
++ grep -c 'not found' /tmp/ldd-report.txt
LDD_UNRESOLVED=0
"""
        parsed = evidence.parse_ldd_report(text)
        self.assertEqual(parsed["unresolved_count"], 0)
        self.assertEqual(parsed["declared_unresolved"], 0)
        self.assertEqual(parsed["shell_trace_lines_excluded"], 2)
        self.assertTrue(parsed["marker_bounded"])

    def test_real_not_found_line_counts_once(self) -> None:
        text = """LDD_REPORT
### /opt/build/smoke
\tlibmissing.so => not found
LDD_UNRESOLVED=1
++ grep -c 'not found' /tmp/ldd-report.txt
"""
        parsed = evidence.parse_ldd_report(text)
        self.assertEqual(parsed["unresolved_count"], 1)
        self.assertEqual(parsed["unresolved_lines"], ["\tlibmissing.so => not found"])

    def test_missing_markers_fail_closed(self) -> None:
        with self.assertRaises(evidence.EvidenceValidationError):
            evidence.parse_ldd_report("### /opt/build/smoke\nlib.so => /lib/lib.so\n")
        with self.assertRaises(evidence.EvidenceValidationError):
            evidence.parse_ldd_report("LDD_REPORT\n### /opt/build/smoke\n")

    def test_corrected_receipt_validates(self) -> None:
        receipt = Path("/tmp/glim-clean-room-phase1-container-r2.EbkpQf/attempt.receipt.corrected.json")
        self.assertTrue(receipt.is_file(), "authoritative corrected receipt is missing")
        result = evidence.validate_receipt_file(receipt)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["receipt_status"], "CORRECTED_SEALED")
        self.assertEqual(result["ctest_all"], {"total": 3, "passed": 3, "failed": 0})
        self.assertEqual(result["ctest_link_smoke"], {"total": 1, "passed": 1, "failed": 0})
        self.assertEqual(result["installed_regular_file_count"], 16428)
        self.assertEqual(result["ldd_unresolved"], 0)

    def test_original_receipt_is_rejected_after_correction(self) -> None:
        receipt = Path("/tmp/glim-clean-room-phase1-container-r2.EbkpQf/attempt.receipt.json")
        self.assertTrue(receipt.is_file(), "preserved original receipt is missing")
        with self.assertRaises(evidence.EvidenceValidationError):
            evidence.validate_receipt_file(receipt)

    def test_corrected_status_requires_supersedes_identity(self) -> None:
        receipt = Path("/tmp/glim-clean-room-phase1-container-r2.EbkpQf/attempt.receipt.corrected.json")
        value = json.loads(receipt.read_text(encoding="utf-8"))
        value = copy.deepcopy(value)
        del value["supersedes_receipt"]
        with self.assertRaises(evidence.EvidenceValidationError):
            evidence.validate_receipt(value, receipt)

    def test_schema_contains_correction_condition(self) -> None:
        schema = json.loads((PHASE1_ROOT / "container_receipt.schema.json").read_text(encoding="utf-8"))
        self.assertIn("CORRECTED_SEALED", schema["properties"]["receipt_status"]["enum"])
        self.assertIn("supersedes_receipt", schema["allOf"][0]["then"]["required"])


if __name__ == "__main__":
    unittest.main()
