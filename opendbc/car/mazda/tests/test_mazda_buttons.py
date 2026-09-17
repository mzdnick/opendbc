"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CRZ_BTNS from the controller: the resume button's ownership under alpha long and the cancel
carve-out while the stock radar still owns the bus.
"""
import pytest

from opendbc.car.mazda.longitudinal import RELEASE_DEBOUNCE_FRAMES
from opendbc.car.mazda.tests.conftest import CRZ_BTNS, LongCtrlState, addrs, car_control, step, step_long


class TestResumeButton:

  @pytest.mark.parametrize("accel", [0.3, -1.024])
  @pytest.mark.parametrize("standstill", [True, False])
  def test_no_resume_button_while_openpilot_owns_longitudinal(self, cc, accel, standstill):
    # We are the ACC here, so the hold is released in-protocol. The car's own MRCC never presses
    # RES either: 0 of 23 stock body-latched-hold releases put one on the bus. A press would also
    # put a second writer on CRZ_BTNS, which ICBM owns.
    assert not cc.resume_requested(car_control(accel=accel, resume=True))

  def test_resume_button_still_sent_with_stock_longitudinal(self, stock_cc):
    # stock ACC owns the hold there, and the button is the only lever openpilot has on it
    assert stock_cc.resume_requested(car_control(accel=0.3, resume=True))
    assert not stock_cc.resume_requested(car_control(accel=0.3, resume=False))

  def test_body_latched_hold_releases_in_protocol(self, cc, cs):
    # the release the button used to stand in for: stop bits already relaxed to the body, then
    # the plan asks to move and the unlatch pulse fires with the release
    for _ in range(200):
      step_long(cc, cs, long_state=LongCtrlState.stopping, accel=-1.024, standstill=True, cruise_engaged=True, brake_hold=True)
    assert cc.stop_and_go.holding and cc.stop_and_go.car_has_hold
    assert not cc.stop_and_go.stop_bits  # body owns the brakes, stock relaxes here

    for _ in range(RELEASE_DEBOUNCE_FRAMES):
      sends = step_long(cc, cs, accel=0.3, standstill=True, cruise_engaged=True, brake_hold=True)
      assert CRZ_BTNS not in addrs(sends), "CRZ_BTNS written at the release"
    assert not cc.stop_and_go.holding
    assert cc.stop_and_go.resume_unlatching, "the pulse must fire with the release"


def cancel_frame(cc, cs, cancel, radar_was_silenced, stock_radar_alive, enabled=False, cruise_engaged=True, tja_button=0):
  cc.frame = 10  # off the 50-frame alert cadence, on the 10-frame cancel cadence
  _, sends = step(cc, cs, long_active=False, enabled=enabled, accel=0., long_state=LongCtrlState.off, available=False,
                  cruise_engaged=cruise_engaged, cancel=cancel, stock_radar_alive=stock_radar_alive, fsc_settled=False,
                  radar_was_silenced=radar_was_silenced, tja_button=tja_button)
  return addrs(sends)


class TestCancelCarveOut:
  """controlsd raises cruiseControl.cancel whenever cruiseState.enabled has no matching
  CC.enabled (mazda reports pcmCruise). While the stock radar still owns the bus that
  engagement is the driver's own stock MRCC and a CANCEL turns its main off within ~100 ms,
  so the documented stay-stock fallback used to leave the driver with no cruise at all. Once
  the radar has been silenced a stock engagement is impossible and cancel handles desync."""

  def test_no_cancel_while_the_radar_is_stock(self, cc, cs):
    # pre-teardown settle window, and equally the silencing-failed drive: a driver SET is
    # their own stock MRCC and must be left alone
    sent = cancel_frame(cc, cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS not in sent, "CANCELed the driver's own stock MRCC"

  def test_cancel_still_sent_after_the_teardown(self, cc, cs):
    # post-teardown a stock engagement is impossible: cancel keeps handling state desync
    sent = cancel_frame(cc, cs, cancel=True, radar_was_silenced=True, stock_radar_alive=False)
    assert CRZ_BTNS in sent

  def test_stock_longitudinal_never_joined_is_not_canceled(self, stock_cc, stock_cs):
    # the feature: ACC with the LKA button off, openpilot fully out
    for _ in range(3):
      sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
      assert CRZ_BTNS not in sent, "CANCELed a cruise openpilot never joined"

  def test_stock_longitudinal_joined_cruise_still_canceled(self, stock_cc, stock_cs):
    # once openpilot has been enabled on the engagement, a later disengage cancels as before
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True, enabled=True)
    sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS in sent

  def test_the_latch_resets_at_cruise_idle(self, stock_cc, stock_cs):
    # the suppressed engagement ends; the next engagement is again the driver's until
    # openpilot joins it
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True, cruise_engaged=False)
    sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS not in sent


class TestTjaHandBack:
  """The declared TJA button turns MADS lateral off with a press stock treats as lane-centering
  only: MRCC never disarms for it. The disable the press triggers must hand the joined cruise
  run back to the driver, the same protection a never-joined run already has."""

  def test_the_press_hands_the_joined_cruise_back(self, stock_cc, stock_cs):
    # MADS ran on the engagement, then the driver pressed TJA: through the press window and
    # past it, the cruise underneath stays theirs. The press cycle still reads CC.enabled,
    # so the clear must land before the disable does.
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True, enabled=True)
    for _ in range(2):
      sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True,
                          enabled=True, tja_button=1)
      assert CRZ_BTNS not in sent
    for _ in range(2):
      sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
      assert CRZ_BTNS not in sent, "CANCELed the cruise the TJA press handed back"

  def test_re_engaging_re_arms_the_sync_cancel(self, stock_cc, stock_cs):
    # the protection belongs to the press, not to the car: joining the same cruise again
    # brings the desync cancel back
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True, enabled=True)
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True, tja_button=1)
    cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True, enabled=True)
    sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS in sent

  def test_alpha_long_still_cancels_through_the_press(self, cc, cs):
    # under alpha-long the engagement is openpilot's own, so the lateral-only disable is a
    # desync and cancels, the same split as the never-joined latch
    cancel_frame(cc, cs, cancel=True, radar_was_silenced=True, stock_radar_alive=False, enabled=True)
    sent = cancel_frame(cc, cs, cancel=True, radar_was_silenced=True, stock_radar_alive=False, tja_button=1)
    assert CRZ_BTNS in sent
