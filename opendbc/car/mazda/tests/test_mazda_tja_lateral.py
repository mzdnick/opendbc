"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The TJA button as the lateral switch: the press arms MRCC as a bus-0 side effect, and the
controller undoes only that arm by replaying the car's own latched main-press frame. The
white assist display (TJA=2) rides the HUD relay while lateral is ours and cruise is
verifiably off.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.conftest import car_control, car_control_sp, car_controller, mazda_car_state

# a real MODE_X/Y main press from this platform (counter 10), and the TJA trim's
# BIT1 active-low encoding: two trims, one replay mechanism
MAIN_PRESS = int.from_bytes(bytes.fromhex("0061ff2800000000"), "big")
TJA_TRIM_PRESS = int.from_bytes(bytes.fromhex("0081fec400000000"), "big")
CAMERA = int.from_bytes(bytes.fromhex("4201000000001040"), "big")


def tja_controller():
  cc = car_controller()
  cc.tja_button_car = True
  return cc


def step(cc, cs, *, tja=0, armed=False, available=False, counter=0, cancel_button=0,
         lat_active=False, master=MAIN_PRESS, cc_sp=None):
  cs.tja_button = tja
  cs.mrcc_armed_raw = armed
  cs.cruise_available = available
  cs.crz_btns_counter = counter
  cs.cancel_button = cancel_button
  cs.master_press_frame = master
  _, sends = cc.update(car_control(long_active=False, lat_active=lat_active),
                       cc_sp if cc_sp is not None else car_control_sp(), cs,
                       int(cc.frame * DT_CTRL * 1e9))
  return [(addr, dat.hex()) for addr, dat, bus in sends]


def test_master_replay_rewrites_only_the_counter():
  replay = mazdacan.create_master_replay(MAIN_PRESS, 11)
  assert replay[0] == 0x09d and replay[2] == 0
  assert replay[1].hex() == "0061ff2c00000000"          # this trim: ctr in bits 5:2
  replay = mazdacan.create_master_replay(TJA_TRIM_PRESS, 5)
  assert replay[1].hex() == "0081fed400000000"          # TJA trim keeps its 0xc0 flag bits


def test_cleanup_replays_after_a_tja_caused_arm_and_stops_when_answered():
  cc = tja_controller()
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  sends = []

  # press from cruise-off, arm appears during the hold, release
  sends += step(cc, cs, tja=1)
  sends += step(cc, cs, tja=1, armed=True)
  sends += step(cc, cs, tja=0, armed=True, counter=1)
  for counter in range(2, 8):
    sends += step(cc, cs, armed=True, counter=counter)
  expected = [mazdacan.create_master_replay(MAIN_PRESS, c)[1].hex() for c in (2, 3, 4)]
  assert [dat for addr, dat in sends if addr == 0x09d] == expected

  # the budget spent: nothing further even as counters keep advancing
  for counter in range(8, 12):
    sends += step(cc, cs, armed=True, counter=counter)
  assert [dat for addr, dat in sends if addr == 0x09d] == expected


def test_cleanup_skips_an_arm_that_was_already_there():
  cc = tja_controller()
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  sends = []
  sends += step(cc, cs, tja=1, armed=True, available=True)
  sends += step(cc, cs, tja=0, armed=True, available=True, counter=1)
  for counter in range(2, 6):
    sends += step(cc, cs, armed=True, available=True, counter=counter)
  assert not [dat for addr, dat in sends if addr == 0x09d]


def test_cleanup_aborts_when_no_arm_or_on_driver_button():
  cc = tja_controller()
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  # the press never armed MRCC
  sends = []
  sends += step(cc, cs, tja=1)
  sends += step(cc, cs, tja=0, counter=1)
  for counter in range(2, 6):
    sends += step(cc, cs, counter=counter)
  assert not [dat for addr, dat in sends if addr == 0x09d]

  # a physical cancel takes the bus back mid-episode
  cc, cs = tja_controller(), mazda_car_state(cc.CP, cc.CP_SP)
  sends = []
  sends += step(cc, cs, tja=1)
  sends += step(cc, cs, tja=1, armed=True)
  sends += step(cc, cs, tja=0, armed=True, counter=1)
  sends += step(cc, cs, armed=True, counter=2, cancel_button=1)
  for counter in range(3, 8):
    sends += step(cc, cs, armed=True, counter=counter)
  assert not [dat for addr, dat in sends if addr == 0x09d]


def cc_sp_with_mads():
  sp = structs.CarControlSP()
  sp.mads.active = True
  return sp


def test_white_hud_gates_on_the_relay():
  cc = tja_controller()
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  cs.out.cruiseState.available = False
  sp = cc_sp_with_mads()
  out = []
  for i in range(220):
    if i % 50 == 0:
      cs.cam_laneinfo_raw = CAMERA
      cs.cam_laneinfo_ts = int((i + 1) * DT_CTRL * 1e9)
      cs.cam_laneinfo_silent_frames = 0
      cs.cam_laneinfo_seen = True
    else:
      cs.cam_laneinfo_silent_frames += 1
    lat = i >= 100
    sends = step(cc, cs, lat_active=lat, cc_sp=sp)
    out += [(dat, i) for addr, dat in sends if addr == 0x440]

  # before lateral: the camera's frame relayed untouched
  assert out[0] == ("4201000000001040", 0)
  # lateral without cruise: after the confirm window the frame carries TJA=2 with the
  # lane lines blanked (steering quietly), never before the window completes
  white_frames = [(dat, i) for dat, i in out if dat == "4201000020001040"]
  assert len(white_frames) == 2 and all(i >= 150 for dat, i in white_frames)
  assert all(dat in ("4201000000001040", "4201000020001040") for dat, i in out)

  # cruise arming withdraws white on the very next control frame, off camera cadence
  sends = step(cc, cs, armed=True, available=True, lat_active=True, cc_sp=sp)
  assert ("4201000000001040", 220) in [(dat, 220) for addr, dat in sends if addr == 0x440]
