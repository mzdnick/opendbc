"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

openpilot never presses the TJA button. On the car's side it toggles MADS and arms MRCC; on the
camera's side it switches the car's own lane-keep setting off (CAM_SETTINGS), after which the
EPS applies none of our request (route 7c735af5fce56485/00000105, 2026-09-12).

The stock-TJA clear: every physical TJA press arms the camera's own TJA/CTS whatever the MRCC
main does, and with cruise set the camera steers on while the panda holds its frames back
(route 0000013b: 57 s blind, a dash camera fault). On a declared TJA car the controller answers
with the wheel's own MRCC master press for as long as an armed camera and an armed main
coexist. The condition is a level: no press memory, no arm-wait window, no frame budget.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda.tests.conftest import CRZ_BTNS, SendButtonState, car_controller, frames, mazda_car_state, step
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

DWELL = int(0.2 / DT_CTRL)  # the body-paced slot between clear presses, shared with ICBM


def rig(alpha_long=False, tja_button=False):
  cc = car_controller(alpha_long=alpha_long)
  cc.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON if tja_button else 0
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  return cc, cs


def drive(cc, cs, cycles, **kwargs):
  sends = []
  for _ in range(cycles):
    sends.extend(step(cc, cs, **kwargs)[1])
  for dat in frames(sends, CRZ_BTNS, bus=0):
    assert not dat[1] & 0x08, "the TJA bit must never go to the car"
  return sends


def live_condition(cc, cs, cycles, **kwargs):
  """The clear condition held: camera steering, raw PEDALS armed, a quiet wheel."""
  return drive(cc, cs, cycles, stock_tja=3, mrcc_armed_raw=True, **kwargs)


def mrcc_off_frames(sends) -> list[bytes]:
  # the wheel's MRCC master press on the car's bus: exact shape, CTR the only variable bits
  return [d for d in frames(sends, CRZ_BTNS)
          if d[:3] == b"\x00\x81\xfe" and d[3] & 0xc3 == 0xc0 and d[4:] == bytes(4)]


class TestNoCameraPress:

  def test_no_button_frame_on_the_camera_bus(self):
    for alpha_long in (False, True):
      cc, cs = rig(alpha_long=alpha_long)
      for stock_tja in (0, 2, 3, 4):
        for lat_active in (False, True):
          for _ in range(150):
            _, sends = step(cc, cs, lat_active=lat_active, stock_tja=stock_tja, radar_was_silenced=alpha_long)
            assert not frames(sends, CRZ_BTNS, bus=2)
            for dat in frames(sends, CRZ_BTNS, bus=0):
              assert not dat[1] & 0x08, "the TJA bit must never go to the car"


@pytest.mark.parametrize("alpha_long", [False, True])
class TestStockTjaClear:

  @pytest.mark.parametrize("state", [2, 3, 4, 5])
  def test_armed_camera_with_armed_main_clears(self, alpha_long, state):
    # state 2 alone is enough: the rule fires on "camera armed + main armed" and never lets
    # the pair survive to a press
    cc, cs = rig(alpha_long, tja_button=True)
    clearing = drive(cc, cs, DWELL + 1, stock_tja=state, mrcc_armed_raw=True)
    assert len(mrcc_off_frames(clearing)) == 1

  def test_the_condition_self_extinguishes(self, alpha_long):
    # the main drops within a body beat of the first press; raw PEDALS then reads disarmed
    # and the stream stops on its own
    cc, cs = rig(alpha_long, tja_button=True)
    live_condition(cc, cs, DWELL + 1)
    settled = drive(cc, cs, 10, stock_tja=3, mrcc_armed_raw=False)
    assert mrcc_off_frames(settled) == []

  def test_camera_armed_with_main_disarmed_sends_nothing(self, alpha_long):
    # the press did not arm the body, or a takeover already disarmed it: nothing to clear
    cc, cs = rig(alpha_long, tja_button=True)
    sends = drive(cc, cs, DWELL * 2, stock_tja=3, mrcc_armed_raw=False)
    assert mrcc_off_frames(sends) == []

  def test_camera_disarmed_with_main_armed_sends_nothing(self, alpha_long):
    # a restored main at boot (0000013b t=0.142) with a camera that never armed: the
    # driver's own arm is not openpilot's to drop
    cc, cs = rig(alpha_long, tja_button=True)
    sends = drive(cc, cs, DWELL * 2, stock_tja=0, mrcc_armed_raw=True)
    assert mrcc_off_frames(sends) == []

  @pytest.mark.parametrize("button", ["tja_button", "mrcc_button", "cancel_button", "resume_button",
                                      "accel_button", "decel_button", "distance_button",
                                      "distance_more_button"])
  def test_a_held_wheel_button_waits(self, alpha_long, button):
    # one CRZ_BTNS source at a time; the condition outlives the hold and finishes the clear
    cc, cs = rig(alpha_long, tja_button=True)
    held = drive(cc, cs, DWELL * 2, stock_tja=3, mrcc_armed_raw=True, **{button: 1})
    assert mrcc_off_frames(held) == []
    after = live_condition(cc, cs, DWELL + 1)
    assert len(mrcc_off_frames(after)) >= 1  # the clear resumes

  @pytest.mark.parametrize("activity", [dict(cancel=True), dict(resume=True)])
  def test_openpilot_cancel_and_resume_wait(self, alpha_long, activity):
    # openpilot's own button frames own the stream; the clear resumes when they fall
    cc, cs = rig(alpha_long, tja_button=True)
    waiting = drive(cc, cs, DWELL * 2, stock_tja=3, mrcc_armed_raw=True, **activity)
    assert mrcc_off_frames(waiting) == []
    after = live_condition(cc, cs, DWELL + 1)
    assert len(mrcc_off_frames(after)) >= 1  # the clear resumes

  def test_stock_ecu_handback_waits(self, alpha_long):
    cc, cs = rig(alpha_long, tja_button=True)
    held = drive(cc, cs, DWELL * 2, stock_tja=3, mrcc_armed_raw=True, handback=True)
    assert mrcc_off_frames(held) == []
    after = live_condition(cc, cs, DWELL + 1)
    assert len(mrcc_off_frames(after)) >= 1  # the clear resumes

  def test_the_dwell_paces_the_stream(self, alpha_long):
    # one press per body-paced slot: 2 * DWELL + 1 cycles hold the condition, two presses land
    cc, cs = rig(alpha_long, tja_button=True)
    sends = live_condition(cc, cs, DWELL * 2 + 1)
    assert len(mrcc_off_frames(sends)) == 2

  def test_a_sustained_condition_never_stops_sending(self, alpha_long):
    # no frame budget: a body that ignores the press sees a held physical button, which the
    # body tolerates at 10 Hz, not a count that runs out
    cc, cs = rig(alpha_long, tja_button=True)
    sends = live_condition(cc, cs, DWELL * 15)
    # frames 0 through DWELL * 15 - 1: one press every slot from the first
    assert len(mrcc_off_frames(sends)) == (DWELL * 15 - 1) // DWELL

  def test_icbm_yields_the_stream_while_the_condition_holds(self, alpha_long):
    cc, cs = rig(alpha_long, tja_button=True)
    live = drive(cc, cs, DWELL * 2, stock_tja=3, mrcc_armed_raw=True,
                 send_button=SendButtonState.increase)
    # the only CRZ_BTNS traffic while live is the clear press itself; ICBM stays quiet
    assert frames(live, CRZ_BTNS, bus=0) == mrcc_off_frames(live)
    assert len(mrcc_off_frames(live)) >= 1
    resumed = drive(cc, cs, 110, available=False, send_button=SendButtonState.increase)
    assert frames(resumed, CRZ_BTNS, bus=0) != []  # ICBM has the stream back


class TestRadarHandbackWaits:

  def test_radar_handback_waits(self):
    # under alpha-long the session manager owns radar_handback_active and overwrites a seeded
    # flag, so the wait runs on a stock-long controller
    cc, cs = rig(alpha_long=False, tja_button=True)
    held = drive(cc, cs, DWELL * 2, stock_tja=3, mrcc_armed_raw=True, radar_handback_active=True)
    assert mrcc_off_frames(held) == []
    after = live_condition(cc, cs, DWELL + 1)
    assert len(mrcc_off_frames(after)) >= 1  # the clear resumes
