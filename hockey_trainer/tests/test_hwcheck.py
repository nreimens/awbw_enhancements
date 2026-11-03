import pytest

from trainer import hwcheck


def run_fake(*argv, answers=("y",) * 20, tmp=None):
    it = iter(answers)
    args = hwcheck.build_parser().parse_args(["--fake", "--out", str(tmp), *argv])
    lines = []
    code = hwcheck.run(args, ask=lambda _: next(it), out=lines.append)
    return code, "\n".join(lines)


def test_all_passes_on_simulator(tmp_path):
    code, text = run_fake("all", answers=["y", "y", "YES"] + ["y"] * 20, tmp=tmp_path)
    assert code == 0, text
    assert "laser dot detection" in text and "FAIL" not in text.replace("0 failed", "")
    assert (tmp_path / "dot.png").exists() and (tmp_path / "camera_0.png").exists()


def test_laser_checks_skipped_without_confirmation(tmp_path):
    code, text = run_fake("dot", answers=["no"], tmp=tmp_path)
    assert "SKIP" in text and code == 0


def test_wrong_direction_reports_invert_hint(tmp_path):
    code, text = run_fake("stepper", "pan", answers=["n"], tmp=tmp_path)
    assert code == 1 and "invert" in text


def test_yes_laser_flag_runs_non_interactively(tmp_path):
    args = hwcheck.build_parser().parse_args(["--fake", "--yes", "--yes-laser", "--out", str(tmp_path), "laser"])
    assert hwcheck.run(args, ask=lambda _: pytest.fail("should not prompt"), out=lambda s: None) == 0
