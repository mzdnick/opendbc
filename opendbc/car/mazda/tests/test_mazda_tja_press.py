"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The TJA press, both sides of it. The camera press: whenever the camera's own TJA/CTS is
armed (0x440 TJA nonzero), steering or not, the controller presses the camera's button
off on its own bus so the two lane-centering systems never run at once and a MADS-off
press cannot hand the wheel to the camera. One frame per press, at least one 0x440
period between presses, three per arming episode, then one stockLkas pulse if openpilot
is steering; the episode resets when the camera reads 0. Not gated on the TJA button
declaration. The MRCC undo: the same physical press arms MRCC on the car's bus, so on a
declared car with cruise off before the press the controller answers with the driver's
own MRCC master press until raw PEDALS confirms the arm is gone. Driver activity wins;
nothing sends while the button is held.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda.tests.conftest import CRZ_BTNS, SendButtonState, car_controller, frames, mazda_car_state, step
from opendbc.car.mazda.values import CarControllerParams
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

INTERVAL = int(CarControllerParams.TJA_PRESS_INTERVAL_T / DT_CTRL)
PRESS = bytes.fromhex("0009ff")
HOLD_CYCLES = 15  # past the shared button pacing before the release


def rig(alpha_long=False, tja_button=False):
  cc = car_controller(alpha_long=alpha_long)
  cc.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON if tja_button else 0
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  return cc, cs


def drive(cc, cs, cycles, **kwargs):
  sends = []
  for _ in range(cycles):
    sends.extend(step(cc, cs, **kwargs)[1])
  return sends


def presses(sends):
  # the one frame openpilot may put on the camera-side CRZ_BTNS
  out = frames(sends, CRZ_BTNS, bus=2)
  for dat in out:
    assert dat[:3] == PRESS and dat[4:] == bytes(4) and dat[3] & 0xc3 == 0xc0
  assert not frames(sends, CRZ_BTNS, bus=0), "the TJA button must never go to the car"
  return len(out)


def mrcc_off_frames(sends) -> list[bytes]:
  # the wheel's MRCC master press on the car's bus: exact shape, CTR the only variable bits
  return [d for d in frames(sends, CRZ_BTNS)
          if d[:3] == b"\x00\x81\xfe" and d[3] & 0xc3 == 0xc0 and d[4:] == bytes(4)]


def undo_episode(alpha_long):
  # cruise off before a TJA press-and-hold: the press-induced arm is live, undo pending
  cc, cs = rig(alpha_long, tja_button=True)
  drive(cc, cs, 5, available=False)
  held = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)
  return cc, cs, held


class TestCameraPress:

  def test_one_press_on_the_first_steering_frame_with_the_camera_armed(self):
    cc, cs = rig()
    assert presses(step(cc, cs, lat_active=True, stock_tja=2, crz_btns_counter=7)[1]) == 1

  def test_counter_is_the_wheels_plus_one(self):
    cc, cs = rig()
    _, sends = step(cc, cs, lat_active=True, stock_tja=2, crz_btns_counter=7)
    assert frames(sends, CRZ_BTNS, bus=2)[0][3] == 0xc0 | (8 << 2)

  def test_no_press_with_the_camera_off(self):
    cc, cs = rig()
    for lat_active in (True, False):
      for _ in range(3 * INTERVAL):
        assert presses(step(cc, cs, lat_active=lat_active, stock_tja=0)[1]) == 0

  def test_pressed_off_with_lateral_off_and_no_warning(self):
    # the MADS-off press re-arms the camera (user report 2026-09-09): pressed off all the same,
    # but a camera that stays on with openpilot not steering is stock behaviour, no stockLkas
    cc, cs = rig()
    n = 0
    for i in range(5 * INTERVAL):
      _, sends = step(cc, cs, lat_active=False, stock_tja=2)
      n += presses(sends)
      assert n == min(i // INTERVAL + 1, CarControllerParams.TJA_PRESS_MAX), i
      assert not cs.stock_cts_stuck

  def test_cadence_cap_and_the_one_shot_warning(self):
    cc, cs = rig()
    n = 0
    for i in range(5 * INTERVAL):
      _, sends = step(cc, cs, lat_active=True, stock_tja=2)
      n += presses(sends)
      assert n == min(i // INTERVAL + 1, CarControllerParams.TJA_PRESS_MAX), i
      # the warning fires once, one interval after the last press, and openpilot keeps steering
      expect_stuck = i == CarControllerParams.TJA_PRESS_MAX * INTERVAL
      assert cs.stock_cts_stuck == expect_stuck, i
      cs.stock_cts_stuck = False  # carstate consumes it
    assert n == CarControllerParams.TJA_PRESS_MAX

  def test_the_episode_resets_when_the_camera_reads_off(self):
    cc, cs = rig()
    for _ in range(4 * INTERVAL):
      step(cc, cs, lat_active=True, stock_tja=2)
    cs.stock_cts_stuck = False
    step(cc, cs, lat_active=True, stock_tja=0)
    # the driver arms it again under us: a fresh episode, pressed at once
    assert presses(step(cc, cs, lat_active=True, stock_tja=2)[1]) == 1
    assert not cs.stock_cts_stuck

  def test_a_pause_in_steering_does_not_reset_the_count(self):
    # only the camera reading 0 ends an episode; dropping lateral for a moment does not
    cc, cs = rig()
    for _ in range(4 * INTERVAL):
      step(cc, cs, lat_active=True, stock_tja=2)
    cs.stock_cts_stuck = False
    for _ in range(INTERVAL):
      step(cc, cs, lat_active=False, stock_tja=2)
    for _ in range(2 * INTERVAL):
      assert presses(step(cc, cs, lat_active=True, stock_tja=2)[1]) == 0
      assert not cs.stock_cts_stuck

  def test_same_press_under_openpilot_longitudinal(self):
    cc, cs = rig(alpha_long=True)
    _, sends = step(cc, cs, lat_active=True, stock_tja=3, radar_was_silenced=True)
    assert presses(sends) == 1


@pytest.mark.parametrize("alpha_long", [False, True])
class TestMrccUndo:

  def test_off_before_press_undoes_the_arm(self, alpha_long):
    cc, cs, held = undo_episode(alpha_long)
    assert mrcc_off_frames(held) == []  # nothing while the driver holds the button
    released = drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(released)) <= 3  # the hold: at most the episode budget
    settled = drive(cc, cs, 10, available=False)  # arm reconciled
    assert mrcc_off_frames(settled) == []
    assert not cc.mrcc_undo_pending
    assert cc.mrcc_undo_frames == 0  # budget returned for the next press

  @pytest.mark.parametrize("pre_armed", [True, False])
  def test_the_drivers_own_arm_and_no_arm_are_left_alone(self, alpha_long, pre_armed):
    # cruise already on before the press, or a press that never arms: no undo either way
    cc, cs = rig(alpha_long, tja_button=True)
    drive(cc, cs, 5, available=pre_armed, mrcc_armed_raw=pre_armed)
    sends = drive(cc, cs, HOLD_CYCLES, available=pre_armed, mrcc_armed_raw=pre_armed, tja_button=1)
    sends += drive(cc, cs, 40, available=pre_armed, mrcc_armed_raw=pre_armed)
    assert mrcc_off_frames(sends) == []

  def test_budget_caps_the_hold_and_returns_after_reconciliation(self, alpha_long):
    cc, cs, _ = undo_episode(alpha_long)
    stuck = drive(cc, cs, 80, available=True, mrcc_armed_raw=True)  # the arm never clears
    assert len(mrcc_off_frames(stuck)) == 3
    assert not cc.mrcc_undo_pending
    drive(cc, cs, 10, available=False)  # reconciled: the budget is back
    drive(cc, cs, 5, available=False)
    again = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)
    again += drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert mrcc_off_frames(again) != []

  def test_a_second_press_before_reconciliation_keeps_the_episode(self, alpha_long):
    # a fast double-press: the second edge lands while the first press's arm is still
    # live, so the "armed before the press" sample is the first press's artifact; the
    # episode must survive and keep undoing, or MRCC stays armed until the driver clears it
    cc, cs, _ = undo_episode(alpha_long)
    released = drive(cc, cs, 10, available=True, mrcc_armed_raw=True)  # one press out, arm unreconciled
    assert len(mrcc_off_frames(released)) == 1
    again = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)  # second press
    assert mrcc_off_frames(again) == []  # nothing while held
    assert cc.mrcc_undo_pending  # not cancelled by the second edge
    after = drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(after)) <= 3  # the budget returned and undoes the second arm

  def test_a_second_press_at_reconciliation_waits_for_its_own_arm(self, alpha_long):
    # the first arm clears under the second press: PEDALS confirms the disarm inside the
    # hold, and the second press's own arm lands only after the release; the episode must
    # wait it out instead of standing down on the confirmed disarm
    cc, cs, _ = undo_episode(alpha_long)
    drive(cc, cs, 10, available=True, mrcc_armed_raw=True)
    held = drive(cc, cs, HOLD_CYCLES, available=False, mrcc_armed_raw=False, tja_button=1)
    assert mrcc_off_frames(held) == []
    assert cc.mrcc_undo_pending  # waiting for this press's arm, not stood down
    armed = drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(armed)) <= 3

  def test_brake_dropout_does_not_end_the_episode(self, alpha_long):
    # both PEDALS cruise bits read low through a brake transition; the filtered state
    # bridges it, so three raw-off cycles stay under the five-frame confirmation
    cc, cs, _ = undo_episode(alpha_long)
    drive(cc, cs, 3, available=True, mrcc_armed_raw=False)
    assert cc.mrcc_undo_pending

  def test_arm_clearing_stands_down_without_a_second_press(self, alpha_long):
    # the undo's own press disarms the car ~80 ms before PEDALS confirms it; the raw gate
    # and the 200 ms slot must hold the second press back, or it would re-arm the car
    cc, cs, _ = undo_episode(alpha_long)
    pressing = drive(cc, cs, 10, available=True, mrcc_armed_raw=True)  # one press out, arm still reflected
    assert len(mrcc_off_frames(pressing)) == 1
    clearing = drive(cc, cs, 40, available=False, mrcc_armed_raw=False)  # the arm clears mid-budget
    assert mrcc_off_frames(clearing) == []
    assert not cc.mrcc_undo_pending  # one press was enough: stood down without spending the budget

  def test_no_arm_wait_is_bounded_and_brake_held(self, alpha_long):
    # a car that never arms through the press: the undo stands down after 1 s brake-free
    # and ICBM gets the button stream back; a held brake restarts the clock, since PEDALS
    # cannot witness an arm under braking
    cc, cs = rig(alpha_long, tja_button=True)
    drive(cc, cs, 5, available=False)
    drive(cc, cs, HOLD_CYCLES, available=False, tja_button=1)
    drive(cc, cs, 60, available=False)  # brake-free wait, still inside 1 s
    assert cc.mrcc_undo_pending
    held = drive(cc, cs, 100, available=False, brake_pressed=True, send_button=SendButtonState.increase)
    assert cc.mrcc_undo_pending  # the brake restarts the clock
    assert frames(held, CRZ_BTNS, bus=0) == []  # ICBM still suppressed while the undo owns the stream
    resumed = drive(cc, cs, 110, available=False, send_button=SendButtonState.increase)
    assert not cc.mrcc_undo_pending  # 1 s brake-free elapsed
    assert frames(resumed, CRZ_BTNS, bus=0) != []  # ICBM has the stream back

  @pytest.mark.parametrize("activity", [dict(mrcc_button=1), dict(cancel_button=1), dict(accel_button=1),
                                        dict(handback=True)])
  def test_driver_activity_aborts(self, alpha_long, activity):
    # the driver's own cruise presses and a stock ECU hand-back all stand down at once
    cc, cs, _ = undo_episode(alpha_long)
    drive(cc, cs, 30, available=True, mrcc_armed_raw=True, **activity)
    assert not cc.mrcc_undo_pending

  def test_openpilot_cancel_waits_then_resumes(self, alpha_long):
    cc, cs, _ = undo_episode(alpha_long)
    waiting = drive(cc, cs, 20, available=True, mrcc_armed_raw=True, cancel=True)
    assert mrcc_off_frames(waiting) == []  # no frame races openpilot's own cancel
    assert cc.mrcc_undo_pending  # waited out, not aborted
    resumed = drive(cc, cs, 30, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(resumed)) <= 3


class TestMrccUndoShipsDark:

  def test_radar_handback_aborts_with_the_budget_frozen(self):
    # under alpha-long the session manager owns radar_handback_active and overwrites a
    # seeded flag, so the abort runs on a stock-long controller
    cc, cs, _ = undo_episode(alpha_long=False)
    drive(cc, cs, 3, available=True, mrcc_armed_raw=True)  # episode live, at most one frame out
    spent = cc.mrcc_undo_frames
    sends = drive(cc, cs, 30, available=True, mrcc_armed_raw=True, radar_handback_active=True)
    assert not cc.mrcc_undo_pending
    assert cc.mrcc_undo_frames == spent  # aborted, not budget-exhausted
    assert mrcc_off_frames(sends) == []

  def test_undeclared_button_never_sends(self):
    cc, cs = rig()
    drive(cc, cs, 5, available=False)
    drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)
    sends = drive(cc, cs, 60, available=True, mrcc_armed_raw=True)
    assert mrcc_off_frames(sends) == []
    assert not cc.mrcc_undo_pending

  def test_icbm_suppressed_through_the_hold(self):
    cc, cs = rig(tja_button=True)
    held = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1,
                 send_button=SendButtonState.increase)
    # the wheel owns the counter stream during the hold: ICBM stays quiet too
    assert frames(held, CRZ_BTNS, bus=0) == []
