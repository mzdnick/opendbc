"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The coast detent: while the plan is quiet the wire rests on the side of its last real
move, clamped to stock's dither band, instead of following the plan's sign noise across
zero. The detent must not snap recoveries onto the band edge, must not arm before the
first real move, and must stay out of the stopping grammar's way.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda.tests.conftest import LongCtrlState, step_long
from opendbc.car.mazda.values import CarControllerParams

STOPPING = LongCtrlState.stopping
CRUISE = dict(cruise_engaged=True, v_ego=20.)


def seconds(t):
  return int(t / DT_CTRL)


def settle(cc, cs, accel, t=2.0, **kwargs):
  for _ in range(seconds(t)):
    step_long(cc, cs, accel=accel, **{**CRUISE, **kwargs})


def test_quiet_plan_rests_on_the_side_of_the_last_real_move(cc, cs):
  # a real pull arms the accel side; the wire settles into the band and plan noise can
  # no longer pull it below zero
  settle(cc, cs, 0.4)
  assert cc.coast_side == 1
  settle(cc, cs, 0.05, t=1.0)
  wire = []
  for i in range(seconds(3.0)):
    step_long(cc, cs, accel=0.05 if i % 2 == 0 else -0.05, **CRUISE)
    wire.append(cc.accel_last)
  assert all(0. <= w <= CarControllerParams.ACCEL_COAST_MAX + 1e-9 for w in wire), \
    f"wire left the accel-side band: {min(wire):.3f}..{max(wire):.3f}"

  # a real decel arms the decel side; noise can no longer lift the wire above zero, and
  # a quiet plan deeper than the floor does not dig past it either
  settle(cc, cs, -0.4)
  assert cc.coast_side == -1
  settle(cc, cs, -0.05, t=1.0)
  wire = []
  for i in range(seconds(3.0)):
    step_long(cc, cs, accel=0.05 if i % 2 == 0 else -0.05, **CRUISE)
    wire.append(cc.accel_last)
  assert all(CarControllerParams.ACCEL_COAST_MIN - 1e-9 <= w <= 0. for w in wire), \
    f"wire left the decel-side band: {min(wire):.3f}..{max(wire):.3f}"
  settle(cc, cs, -0.11, t=0.5)  # quiet, but deeper than the floor
  assert cc.accel_last >= CarControllerParams.ACCEL_COAST_MIN - 1e-9, "detent let the wire dig past the floor"


def test_recoveries_transit_the_band_at_the_slew_limits(cc, cs):
  # from a real decel the command returns to the quiet plan at the slew limits; the detent
  # must not snap it onto the band edge, and it never crosses onto the wrong side
  settle(cc, cs, -0.4)
  assert cc.accel_last == pytest.approx(-0.4, abs=1e-6)
  prev, steps = cc.accel_last, []
  for _ in range(seconds(1.0)):
    step_long(cc, cs, accel=0.0, **CRUISE)
    steps.append(cc.accel_last - prev)
    prev = cc.accel_last
  assert max(steps) <= CarControllerParams.ACCEL_WINDUP_LIMIT + 1e-9, "detent snapped the recovery"
  assert cc.accel_last == pytest.approx(0., abs=1e-6)


def test_the_detent_is_armed_only_after_a_real_move(cc, cs):
  # a fresh engagement under a quiet plan passes the plan through: the wire still takes
  # both signs, and no side has been picked
  lo, hi = 10., -10.
  for i in range(seconds(2.0)):
    step_long(cc, cs, accel=0.05 if (i // 50) % 2 == 0 else -0.05, **CRUISE)
    lo, hi = min(lo, cc.accel_last), max(hi, cc.accel_last)
  assert cc.coast_side == 0
  assert lo < 0. < hi, f"a fresh engagement should still cross zero: {lo:.3f}..{hi:.3f}"


def test_the_band_stays_off_below_min_speed(cc, cs):
  # the creep and stop grammar owns the command at low speed, whatever side is armed
  settle(cc, cs, 0.4)
  assert cc.coast_side == 1
  settle(cc, cs, -0.05, t=0.5, v_ego=2.0)
  assert cc.accel_last == pytest.approx(-0.05, abs=1e-6), "band held the wire at low speed"


def test_a_spike_does_not_flip_the_resting_side(cc, cs):
  settle(cc, cs, -0.4)
  settle(cc, cs, -0.05, t=1.0)
  assert cc.coast_side == -1 and cc.accel_last == pytest.approx(-0.05, abs=1e-6)

  # one frame past the zone is not a move: the side stays and the wire stays off the
  # accel side through the spike
  step_long(cc, cs, accel=0.2, **CRUISE)
  assert cc.coast_side == -1
  assert cc.accel_last <= 0.

  # a sustained move does flip it
  settle(cc, cs, 0.3, t=0.5)
  assert cc.coast_side == 1


def test_stopping_owns_the_command_over_the_detent(cc, cs):
  # the freeze paths keep their grammar: STOPPING with a quiet plan just past the band
  # floor passes through where the detent would hold
  settle(cc, cs, -0.4)
  settle(cc, cs, -0.08, t=1.0)
  assert cc.accel_last == pytest.approx(-0.08, abs=1e-6)
  for _ in range(5):
    step_long(cc, cs, long_state=STOPPING, accel=-0.11, standstill=False, cruise_engaged=True, v_ego=20.)
  assert cc.accel_last == pytest.approx(-0.11, abs=1e-6), "detent clipped the stopping grammar"


def test_disengagement_re_arms_the_detent(cc, cs):
  settle(cc, cs, 0.4)
  assert cc.coast_side == 1
  for _ in range(seconds(0.5)):
    step_long(cc, cs, long_active=False, enabled=True, long_state=LongCtrlState.off, accel=0., cruise_engaged=True)
  assert cc.coast_side == 0 and cc.coast_streak == 0
