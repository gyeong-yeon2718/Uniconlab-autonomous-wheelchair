#!/usr/bin/env python3
"""What the app's four buttons reach, and whether they all reach the same place.

The Bluetooth bridge runs a fixed allowlist of four home scripts, one per
button. Those scripts were maintained by hand on the NUC, in no repository, and
they drifted: on 2026-08-28 [로컬 켜기] brought up one deploy while [주행 시작]
drove another. Nothing could see it, because none of the four was in git.

These tests are the part of that which can be checked away from the chair: that
the four are generated together from one file, that the one file cannot be
re-entered or fall back into a $HOME wrapper, that stopping never routes through
a script whose own fallback is the wrapper that called it, and that the app's
ordinary way of setting off runs the same preflight as its other one.

No ROS, no hardware. The end-to-end cases build a fake deploy in a tmpdir and
run the real scripts against it.
"""

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
BRIDGE = ROOT / "scripts" / "ros1_bluetooth_bridge.py"

requires_bash = pytest.mark.skipif(
    shutil.which("bash") is None or sys.platform == "win32",
    reason="needs a POSIX bash and pkill-shaped process names")


def tool(name):
    return (TOOLS / name).read_text(encoding="utf-8")


def bridge():
    return BRIDGE.read_text(encoding="utf-8")


def bridge_jobs():
    """The allowlist the phone can actually reach, read out of the bridge.

    Derived rather than listed: a hand-kept copy here would be edited in the
    same commit as JobRunner.JOBS and would therefore never catch anything.
    """
    block = re.search(r"JOBS = \{(.*?)\n    \}", bridge(), re.S)
    assert block, "cannot find JobRunner.JOBS in the bridge"
    return dict(re.findall(r'"([a-z_]+)": \("([^"]+)"', block.group(1)))


# ------------------------------------------------------------- the allowlist

def test_every_button_the_app_can_press_has_a_script_in_the_repo():
    """A job with no script is a permanently greyed-out button.

    stop_stack.sh was exactly this for every field session between the bridge
    being written and 2026-08-30: JobRunner.JOBS offered it, the app had the
    [스택 내리기] button wired to it, and no such file existed anywhere. The
    cost was not only the button. stack_stop is the bridge's only real abort
    for a bring-up in progress -- job_cancel signals the tracked process group
    and the bring-up detaches its nodes with setsid, so the sensors it already
    started never see that signal.
    """
    missing = sorted(script for script in bridge_jobs().values()
                     if not (TOOLS / script).is_file())
    assert not missing, (
        "the bridge offers %s but the repo has no such script, so the app "
        "button is dead and cannot be installed" % ", ".join(missing))


def test_the_installer_writes_every_job_the_bridge_offers_and_nothing_else():
    """One act, all four. The installer is the only thing that writes them, so
    if it and the bridge disagree about the set, one button keeps pointing at
    whatever was there before -- which is the drift, exactly."""
    block = re.search(r'JOBS="(.*?)"', tool("install_operator_entrypoints.sh"), re.S)
    assert block, "cannot find the installer's job table"
    installed = set(re.findall(r"([a-z_]+\.sh):[a-z-]+", block.group(1)))
    assert installed == set(bridge_jobs().values())


# ---------------------------------------------------------- the single entry

def test_stopping_never_routes_through_hybrid():
    """hybrid.sh's own `stop` resolves BASE_STOP:-$HOME/stop.sh, and once
    ~/stop.sh is a wrapper back into the entry point that is a loop whose every
    iteration is a stop that has not happened yet. Straight to the deploy's
    stop.sh instead."""
    entry = tool("wheelchair_entry.sh")
    branch = entry[entry.index("  drive-stop|estop)"):]
    assert "hybrid.sh" not in branch.split("esac")[0]


def test_the_entry_point_exports_the_bases_before_it_dispatches():
    """hybrid.sh, start_hybrid_avoidance.sh and go_hybrid.sh all fall back to
    $HOME/<name>.sh when these are unset -- and $HOME/<name>.sh is the wrapper
    that called them. LOCALIZATION_WS is the quieter half of the same bug: it
    defaults to the OLD single workspace, so a wrapper that forgets it sources
    one workspace while catkin's relay stubs exec another tree's source."""
    entry = tool("wheelchair_entry.sh")
    exports = entry[:entry.index("# --------------------------------------------------------------- commands")]
    for name in ("BASE_START", "BASE_GO", "BASE_STOP", "LOCALIZATION_WS"):
        assert re.search(r"^export .*%s=" % name, exports, re.M) or \
               re.search(r"^export %s=" % name, exports, re.M), \
            "%s is not exported before dispatch" % name


def test_re_entry_refuses_instead_of_recursing():
    entry = tool("wheelchair_entry.sh")
    assert "WHEELCHAIR_ENTRY_ACTIVE" in entry
    assert "exit 70" in entry


def test_the_deploy_is_decided_by_where_this_file_is_not_by_current():
    """~/wheelchair_deploys/current moved twice under one session on
    2026-08-30 and ended on a different branch's deploy. A chair that follows
    it changes what it runs while nobody is looking; `cat ~/go.sh` should say
    which tree drives, and switching should take an install."""
    entry = tool("wheelchair_entry.sh")
    self_located = entry.index('DEPLOY_SOURCE="the deploy this script lives in"')
    current_fallback = entry.index('UNPINNED')
    assert self_located < current_fallback


# ------------------------------------------------------------- the teardown

def test_the_teardown_stops_the_drive_before_it_sweeps():
    """Killing the follower while the base is still in auto leaves uart.py
    accepting nothing and the chair coasting on its last command until the
    0.6 s starvation watchdog notices. And uart.py is itself swept, so the
    mode command has to land while something is still there to carry it."""
    text = tool("stop_stack.sh")
    assert text.index("stopping the drive") < text.index("sweeping the stack")
    assert text.index("sweeping the stack") < text.index("taking the master down")


def test_the_teardown_reads_its_sweep_out_of_the_bringup():
    """A second hand-kept list would be edited in the same commit as the first
    and would catch nothing. A node missing from a TEARDOWN list is worse than
    one missing from a start-up list: it keeps its ROS name and its publishers
    with no new run on top of it to make the trouble visible."""
    text = tool("stop_stack.sh")
    assert "start_wheelchair_localization.sh" in text
    assert re.search(r"sed -n 's/\^for pattern in", text)
    # And the confirmation uses the same derived list, not a fresh one.
    assert 'SURVIVOR_RE="$(printf' in text


def test_the_teardown_leaves_the_chair_on_the_joystick():
    text = tool("stop_stack.sh")
    assert "manual" in text.lower()


# --------------------------------------------------------- the bridge's half

def test_arm_and_drive_takes_the_same_path_as_drive_start():
    """The app sends arm_and_drive from [주행 시작] too, whenever the base is
    resting in manual -- which is the normal state right after bring-up. So the
    ordinary way of setting off used to be the one way that skipped
    go_hybrid.sh: eight node pings, the CuPy/RTX DWA backend, PointPillars,
    hybrid_preflight.py and person_bypass_preflight.py."""
    text = bridge()
    body = text[text.index("def _arm_and_drive"):text.index("def _halt")]
    assert "self._drive(payload, True)" in body, (
        "arm_and_drive must go through _drive, which runs the operator's go.sh")
    assert "set_follower" not in body, (
        "arm_and_drive must not start the follower itself -- that is the "
        "shortcut that skipped every preflight")


def test_arming_waits_for_the_controller_to_agree_before_driving():
    """Rule 5: a command is effective when the echo agrees. uart.py transmits a
    stop frame on entering auto and only then accepts wheel_cmd, so a drive
    started inside that window runs with every command discarded -- follower
    reporting DRIVING, dashboard green, chair stationary."""
    text = bridge()
    body = text[text.index("def _arm_and_drive"):text.index("def _halt")]
    assert body.index("await_auto_echo") < body.index("self._drive(payload, True)")


def test_stopping_does_the_same_two_acts_however_it_is_reached():
    """[주행 정지] runs stop.sh, which publishes mode 77 AND pauses the
    follower. Its Python fallback used to pause the follower and stop there, so
    with --allow-scripts off the base stayed in auto and anything that
    published a wheel_cmd afterwards still moved the chair. Same button, two
    different amounts of stopping, chosen by a flag the rider cannot see."""
    text = bridge()
    body = text[text.index("def _halt"):text.index("def _stack_start")]
    assert "set_follower(False)" not in body
    assert body.count("engage_estop(mark_estop=False)") == 2


def test_an_ordinary_stop_is_not_reported_as_an_emergency():
    """estop_engaged greys out the drive controls and points the rider at
    [E-STOP 해제]. Calling a [주행 정지] an e-stop sends them looking for an
    emergency that never happened."""
    text = bridge()
    body = text[text.index("def engage_estop"):text.index("def release_estop")]
    assert "if mark_estop:" in body
    assert body.index("_publish_mode(MANUAL_MODE)") < body.index("if mark_estop:")


def test_the_stop_publishes_the_mode_before_it_calls_the_service():
    """Publishing is instant and cannot fail; a missing service must never be
    what delays a stop."""
    text = bridge()
    body = text[text.index("def engage_estop"):text.index("def release_estop")]
    assert body.index("_publish_mode(MANUAL_MODE)") < body.index("set_follower(False)")


# ------------------------------------------------------------- end to end

def _fake_deploy(tmp_path, name="deployA"):
    home = tmp_path / "home"
    deploy = home / "wheelchair_deploys" / name
    tools = deploy / "source-linux" / "tools"
    tools.mkdir(parents=True)
    (deploy / "source-linux" / "src" / "static_livox_localization").mkdir(parents=True)
    (deploy / "ws" / "src").mkdir(parents=True)
    os.symlink("../../source-linux/src/static_livox_localization",
               deploy / "ws" / "src" / "static_livox_localization")
    for script in ("wheelchair_entry.sh", "install_operator_entrypoints.sh",
                   "stop_stack.sh", "start_wheelchair_localization.sh"):
        shutil.copy(TOOLS / script, tools / script)
    for stub in ("hybrid.sh", "go.sh", "stop.sh"):
        (tools / stub).write_text(
            '#!/usr/bin/env bash\necho "RAN %s $*"\n'
            'echo "WS=$LOCALIZATION_WS"\necho "START=$BASE_START"\n'
            'echo "GO=$BASE_GO"\necho "STOP=$BASE_STOP"\n' % stub)
    for path in tools.iterdir():
        path.chmod(0o755)
    return home, deploy


def _run(home, *argv):
    env = dict(os.environ, HOME=str(home))
    # encoding is explicit: the scripts print Korean, and a non-UTF-8
    # locale turns that into a decode error rather than a test failure.
    return subprocess.run(["bash", *[str(a) for a in argv]], env=env,
                          capture_output=True, text=True, encoding="utf-8")


@requires_bash
def test_installing_points_all_four_buttons_at_one_deploy(tmp_path):
    home, deploy = _fake_deploy(tmp_path)
    entry = deploy / "source-linux" / "tools" / "wheelchair_entry.sh"

    (home / "go.sh").write_text('#!/usr/bin/env bash\necho hand-written\n')

    done = _run(home, deploy / "source-linux" / "tools" /
                "install_operator_entrypoints.sh")
    assert done.returncode == 0, done.stderr

    # The hand-written original is kept, not swallowed.
    assert "hand-written" in (home / "go.sh.pre-wheelchair-entry").read_text()

    for name in bridge_jobs().values():
        assert str(entry) in (home / name).read_text(), \
            "~/%s does not name the entry point that installed it" % name

    assert _run(home, entry, "check").returncode == 0


@requires_bash
def test_check_fails_when_one_button_is_re_pointed(tmp_path):
    """The 2026-08-28 failure, in one command. Every wrapper still mentions
    wheelchair_entry.sh; they no longer name the same one, and that is the
    difference that mattered."""
    home, deploy = _fake_deploy(tmp_path)
    _, other = _fake_deploy(tmp_path / "second", "deployB")
    entry = deploy / "source-linux" / "tools" / "wheelchair_entry.sh"
    _run(home, deploy / "source-linux" / "tools" /
         "install_operator_entrypoints.sh")

    (home / "go.sh").write_text(
        '#!/usr/bin/env bash\nexec bash "%s/source-linux/tools/'
        'wheelchair_entry.sh" drive-start "$@"\n' % other)

    done = _run(home, entry, "check")
    assert done.returncode == 1
    assert "DIVERGED" in done.stdout


@requires_bash
def test_a_workspace_symlink_into_another_deploy_refuses_to_drive(tmp_path):
    """The 2026-08-30 failure: a deploy whose name said one thing and whose
    ws/src symlink pointed into a different deploy entirely. catkin's relay
    stubs exec the source file, so the symlink is what drives."""
    home, deploy = _fake_deploy(tmp_path)
    _, other = _fake_deploy(tmp_path / "second", "deployB")
    entry = deploy / "source-linux" / "tools" / "wheelchair_entry.sh"

    link = deploy / "ws" / "src" / "static_livox_localization"
    link.unlink()
    os.symlink(other / "source-linux" / "src" / "static_livox_localization", link)

    done = _run(home, entry, "drive-start")
    assert done.returncode != 0
    assert "OUTSIDE itself" in done.stderr
    assert "RAN hybrid.sh" not in done.stdout


@requires_bash
def test_each_button_reaches_its_own_entry_with_the_bases_set(tmp_path):
    home, deploy = _fake_deploy(tmp_path)
    _run(home, deploy / "source-linux" / "tools" /
         "install_operator_entrypoints.sh")
    tools = deploy / "source-linux" / "tools"

    for name, expected in (("start_wheelchair_localization.sh", "RAN hybrid.sh start"),
                           ("go.sh", "RAN hybrid.sh go"),
                           ("stop.sh", "RAN stop.sh")):
        done = _run(home, home / name)
        assert expected in done.stdout, (name, done.stdout, done.stderr)
        assert "WS=%s" % (deploy / "ws") in done.stdout
        assert "START=%s" % (tools / "start_wheelchair_localization.sh") in done.stdout
        assert "GO=%s" % (tools / "go.sh") in done.stdout
        assert "STOP=%s" % (tools / "stop.sh") in done.stdout


@requires_bash
def test_re_entry_from_a_deploy_fallback_exits_rather_than_looping(tmp_path):
    home, deploy = _fake_deploy(tmp_path)
    entry = deploy / "source-linux" / "tools" / "wheelchair_entry.sh"
    env = dict(os.environ, HOME=str(home), WHEELCHAIR_ENTRY_ACTIVE=str(deploy))
    done = subprocess.run(["bash", str(entry), "drive-start"], env=env,
                          capture_output=True, text=True, encoding="utf-8")
    assert done.returncode == 70
    assert "RAN hybrid.sh" not in done.stdout
