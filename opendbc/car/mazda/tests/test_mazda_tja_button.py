#!/usr/bin/env python3
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, Buttons

ButtonType = structs.CarState.ButtonEvent.Type


def _interface(candidate=CAR.MAZDA_CX5_2022, alpha_long=False):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


class ButtonHarness:
  """Feeds CRZ_BTNS and CRZ_CTRL the way the wheel and the body ECU send them."""

  def __init__(self, candidate=CAR.MAZDA_CX5_2022, alpha_long=False):
    self.ci = _interface(candidate, alpha_long)
    self.packer = CANPacker("mazda_2017")
    self.time = 0

  def step(self, *, tja=0, mode_x=0, mode_y=0, bit1=1, available=0, active=0):
    self.time += 10_000_000
    buttons = self.packer.make_can_msg("CRZ_BTNS", 0, {
      "TJA_BUTTON": tja, "MODE_X": mode_x, "MODE_Y": mode_y,
      "BIT1": bit1, "BIT1_INV": (bit1 + 1) % 2, "BIT2": 1, "BIT3": 1,
    })
    cruise = self.packer.make_can_msg("CRZ_CTRL", 0, {"CRZ_AVAILABLE": available, "CRZ_ACTIVE": active})
    return self.ci.update([(self.time, [buttons, cruise])])[0]


def _events(state, event_type):
  return [event for event in state.buttonEvents if event.type == event_type]


def test_first_tja_press_latches_and_emits_lkas():
  harness = ButtonHarness()
  harness.step()
  # the very first press both proves the hardware and is a real press
  pressed = harness.step(tja=1)
  held = harness.step(tja=1)
  released = harness.step()
  assert harness.ci.CS.tja_hw_seen
  assert [(event.pressed, event.type) for event in _events(pressed, ButtonType.lkas)] == [(True, ButtonType.lkas)]
  assert not _events(held, ButtonType.lkas)
  assert [(event.pressed, event.type) for event in _events(released, ButtonType.lkas)] == [(False, ButtonType.lkas)]


def test_main_button_still_emits_main_cruise_on_the_tja_platform():
  # events stay additive: the MODE pair keeps its meaning on every car, TJA or not
  harness = ButtonHarness()
  harness.step()
  mode = harness.step(mode_x=1, mode_y=1)
  tja = harness.step(tja=1)
  assert [(event.pressed, event.type) for event in _events(mode, ButtonType.mainCruise)] == [(True, ButtonType.mainCruise)]
  assert [(event.pressed, event.type) for event in _events(tja, ButtonType.lkas)] == [(True, ButtonType.lkas)]


def test_non_target_platform_never_latches_or_emits():
  harness = ButtonHarness(candidate=CAR.MAZDA_CX5)
  harness.step()
  state = harness.step(tja=1)
  assert not harness.ci.CS.tja_hw_seen
  assert not _events(state, ButtonType.lkas)
  # and the main button keeps working there
  mode = harness.step(mode_x=1, mode_y=1)
  assert len(_events(mode, ButtonType.mainCruise)) == 1


def test_mrcc_button_follows_the_active_low_master():
  harness = ButtonHarness()
  harness.step()  # warm-up: the first update registers lazily-read signals
  harness.step(bit1=1)
  assert harness.ci.CS.mrcc_button == 0
  harness.step(bit1=0)
  assert harness.ci.CS.mrcc_button == 1
  harness.step(bit1=1)
  assert harness.ci.CS.mrcc_button == 0


def test_raw_armed_state_tracks_availability():
  harness = ButtonHarness()
  harness.step()  # warm-up: the first update registers lazily-read signals
  harness.step(available=0)
  assert not harness.ci.CS.mrcc_armed_raw
  harness.step(available=1)
  assert harness.ci.CS.mrcc_armed_raw
  harness.step(available=0)
  assert not harness.ci.CS.mrcc_armed_raw


def _decode_buttons(data):
  parser = CANParser("mazda_2017", [("CRZ_BTNS", 10)], 0)
  parser.update([(0, [(0x09d, data, 0)])])
  return parser.vl["CRZ_BTNS"]


@pytest.mark.parametrize("button", [Buttons.CANCEL, Buttons.RESUME, Buttons.SET_PLUS, Buttons.SET_MINUS])
def test_synthetic_buttons_preserve_the_live_wheel_state(button):
  # a latched TJA car echoes the wheel while openpilot speaks over it, so a synthetic
  # frame cannot fabricate a TJA or MODE release mid-hold
  ci = _interface()
  packer = CANPacker("mazda_2017")
  ci.CS.tja_hw_seen = True
  ci.CS.tja_button = 1
  ci.CS.mode_x = 1
  ci.CS.mode_y = 0
  _, data, _ = mazdacan.create_button_cmd(packer, ci.CP, 3, button, ci.CS)
  decoded = _decode_buttons(data)
  assert decoded["TJA_BUTTON"] == 1
  assert decoded["MODE_X"] == 1
  assert decoded["MODE_Y"] == 0


def test_synthetic_buttons_stay_zeroed_before_the_latch():
  ci = _interface()
  packer = CANPacker("mazda_2017")
  ci.CS.tja_button = 1
  ci.CS.mode_x = 1
  ci.CS.mode_y = 1
  ci.CS.tja_hw_seen = False
  _, data, _ = mazdacan.create_button_cmd(packer, ci.CP, 3, Buttons.CANCEL, ci.CS)
  decoded = _decode_buttons(data)
  assert decoded["TJA_BUTTON"] == 0
  assert decoded["MODE_X"] == 0
  assert decoded["MODE_Y"] == 0


def test_synthetic_buttons_stay_zeroed_on_other_platforms():
  ci = _interface(candidate=CAR.MAZDA_CX5)
  packer = CANPacker("mazda_2017")
  state = SimpleNamespace(tja_hw_seen=True, tja_button=1, mode_x=1, mode_y=1)
  _, data, _ = mazdacan.create_button_cmd(packer, ci.CP, 3, Buttons.CANCEL, state)
  decoded = _decode_buttons(data)
  assert decoded["TJA_BUTTON"] == 0
  assert decoded["MODE_X"] == 0
  assert decoded["MODE_Y"] == 0


def test_mrcc_off_tap_encodes_the_active_low_master():
  ci = _interface()
  packer = CANPacker("mazda_2017")
  state = SimpleNamespace(tja_hw_seen=False, tja_button=0, mode_x=0, mode_y=0)
  _, tap, _ = mazdacan.create_button_cmd(packer, ci.CP, 4, Buttons.MRCC_OFF, state)
  decoded = _decode_buttons(tap)
  assert decoded["BIT1"] == 0
  assert decoded["BIT1_INV"] == 1
  # every other button rests
  assert decoded["CTR"] == 5
  # and ordinary buttons keep BIT1 high, the at-rest level
  _, cancel, _ = mazdacan.create_button_cmd(packer, ci.CP, 4, Buttons.CANCEL, state)
  assert _decode_buttons(cancel)["BIT1"] == 1
