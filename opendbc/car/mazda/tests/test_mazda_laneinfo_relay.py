"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The HUD relay through the real CarController: fresh camera frames go out byte-exact
at the camera's own cadence, a silent camera holds the last frame at 2 Hz, and a
camera that has never spoken holds an empty frame after the stale window. The stale
hold is inherited behavior, not a new decision: the rebuilt frame the relay replaced
kept sending the parser's frozen last values at the same 2 Hz through a dropout.
"""

from opendbc.car import DT_CTRL
from opendbc.car.mazda.tests.conftest import car_control, car_control_sp, car_controller, mazda_car_state

CAMERA_A = bytes.fromhex("4202000640001040")
CAMERA_B = bytes.fromhex("4202000000001040")
EMPTY_HOLD = "0" * 16


def drive(cc, cs, cam=None):
  """One controller frame; cam = (raw, ts) for a camera frame landing this cycle."""
  if cam is not None:
    cs.cam_laneinfo_raw, cs.cam_laneinfo_ts = cam
  _, sends = cc.update(car_control(long_active=False), car_control_sp(), cs, int(cc.frame * DT_CTRL * 1e9))
  return [dat.hex() for addr, dat, bus in sends if addr == 0x440]


def camera_ts(i):
  return int(i * DT_CTRL * 1e9) + 1


def test_relay_tracks_the_camera_and_holds_a_dropout():
  cc = car_controller()
  cs = mazda_car_state(cc.CP, cc.CP_SP)

  # camera never spoke: nothing for the stale window, then the empty hold at 2 Hz
  out = []
  for _ in range(150):
    out += drive(cc, cs)
  assert out == [EMPTY_HOLD]

  # fresh frames relay immediately and byte-exact, one per camera frame
  out = []
  for k in range(3):
    out += drive(cc, cs, cam=(int.from_bytes(CAMERA_A, "big"), camera_ts(150 + k)))
  assert out == [CAMERA_A.hex()] * 3

  # dropout: the last frame holds at 2 Hz past the stale window
  out = []
  for _ in range(200):
    out += drive(cc, cs)
  assert out == [CAMERA_A.hex()] * 2

  # a recovered camera relays on the very next frame
  out = drive(cc, cs, cam=(int.from_bytes(CAMERA_B, "big"), camera_ts(400)))
  assert out == [CAMERA_B.hex()]
