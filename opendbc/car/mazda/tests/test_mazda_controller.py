#!/usr/bin/env python3
"""Tests for the Mazda CX-5 2022+ EPS steering parameters (gated on the EPS, not the model)
and the longitudinal message builders and standstill hold."""

from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.longitudinal import LEAD_DEBOUNCE_FRAMES, RESUME_UNLATCH_FRAMES, StandstillHold
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams, MazdaFlags


class TestCarControllerParams:

  @pytest.fixture
  def cx5_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5_2022
      minSteerSpeed = 0.0   # steer_to_zero -> CX-5 2022+ EPS present
    return CarControllerParams(FakeCP())

  @pytest.fixture
  def eps_swap_params(self):
    # A CX-5 2022+ EPS swapped into (or shared by) another Mazda: different model, same EPS.
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX9_2021
      minSteerSpeed = 0.0
    return CarControllerParams(FakeCP())

  @pytest.fixture
  def pre_2022_params(self):
    class FakeCP:
      carFingerprint = CAR.MAZDA_CX5
      minSteerSpeed = 12.5   # no CX-5 EPS -> low-speed lockout, minSteerSpeed > 0
    return CarControllerParams(FakeCP())

  def test_cx5_2022_has_lookup(self, cx5_2022_params):
    assert hasattr(cx5_2022_params, 'STEER_MAX_LOOKUP')
    assert cx5_2022_params.STEER_MAX == 1200

  def test_cx5_2022_low_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [0.0, 5.0, 10.0, 14.2]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 1200

  def test_cx5_2022_high_speed(self, cx5_2022_params):
    p = cx5_2022_params
    for v in [14.5, 20.0, 30.0]:
      sm = round(float(np.interp(v, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1])))
      assert sm == 800

  def test_cx5_2022_rate_limits(self, cx5_2022_params):
    assert cx5_2022_params.STEER_DELTA_UP == 12
    assert cx5_2022_params.STEER_DELTA_DOWN == 25

  def test_cx5_eps_driver_multiplier(self, cx5_2022_params):
    # 15 is the CX-5-EPS tune (upstream stock is 1)
    assert cx5_2022_params.STEER_DRIVER_MULTIPLIER == 15

  def test_eps_swap_gets_cx5_tune(self, eps_swap_params):
    # EPS present (minSteerSpeed == 0) on a non-CX-5 model still gets the higher-authority tune
    assert eps_swap_params.STEER_MAX == 1200
    assert eps_swap_params.STEER_DRIVER_MULTIPLIER == 15
    assert hasattr(eps_swap_params, 'STEER_MAX_LOOKUP')

  def test_no_eps_no_lookup(self, pre_2022_params):
    assert not hasattr(pre_2022_params, 'STEER_MAX_LOOKUP')
    assert pre_2022_params.STEER_MAX == 800
    assert pre_2022_params.STEER_DRIVER_MULTIPLIER == 1


def crz_info_reference_checksum(dat):
  # independent reimplementation of the CRZ_INFO checksum, validated against 1.94M stock
  # frames including all 10,350 stop-bit frames
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


def decode_accel_cmd_raw(dat):
  return (((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096


class TestMazdaLongitudinalMessages:
  """The synthetic CRZ_INFO/CRZ_CTRL/radar frames must reproduce stock captures byte for
  byte; the hex values below come from real radar traffic."""

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  def test_crz_info_standby_matches_stock(self, packer):
    for counter in range(16):
      checksum = (0x5d - counter) & 0xff
      expected = f"01ffe3ffc000{counter:02x}{checksum:02x}"
      dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, False, False, False, False)[1]
      assert dat.hex() == expected

  def test_crz_info_available_matches_stock(self, packer):
    for counter in range(16):
      checksum = (0x99 - counter) & 0xff
      expected = f"01ffe2000480{counter:02x}{checksum:02x}"
      dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, False, True, False, False)[1]
      assert dat.hex() == expected

  @pytest.mark.parametrize(("accel", "stopping", "unlatching", "counter", "expected"), [
    (0.0, False, False, 0, "01ffe20006800097"),     # engaged, zero command
    (2.0, False, False, 3, "01ffe2fa0680039a"),     # ISO max accel, raw 2000
    (-3.5, False, False, 7, "01ffe04a868007c8"),    # ISO max brake, raw -3500
    (-1.024, True, False, 5, "01ffe18006841503"),   # standstill hold, raw -1024 + stop bits
    (-0.001, False, False, 9, "01ffe1ffe68009b0"),  # latched hold, raw -1
    (0.0, False, True, 11, "01ffe20006804b4c"),     # resume unlatch pulse
  ])
  def test_crz_info_engaged_golden_bytes(self, packer, accel, stopping, unlatching, counter, expected):
    dat = mazdacan.create_acc_command(packer, 0, counter, accel, True, False, stopping, unlatching)[1]
    assert dat.hex() == expected

  def test_crz_info_accel_encoding_and_checksum(self, packer):
    # the packed command must round-trip at the 0.001 factor and carry a valid masked-bit
    # checksum over the whole command window, stop bits set or not
    for raw in range(-3500, 2001, 137):
      for stopping in (False, True):
        dat = mazdacan.create_acc_command(packer, 0, raw % 16, raw / 1000.0, True, False, stopping, False)[1]
        assert decode_accel_cmd_raw(dat) == raw
        assert dat[7] == crz_info_reference_checksum(dat)
        assert bool(dat[5] & 0x04) == stopping
        assert bool(dat[6] & 0x10) == stopping

  @pytest.mark.parametrize(("long_active", "acc_available", "gap", "has_lead", "phase", "acc_active_2", "expected"), [
    (False, False, 0, False, 0, False, "0201010000000000"),  # standby
    (False, True, 2, False, 0, False, "02010b0000000000"),   # MRCC armed, SET allowed
    (True, True, 2, True, 1, True, "0a018b2000001000"),      # engaged, cruise, no lead
    (True, True, 2, True, 2, True, "0a018b4000001000"),      # engaged, following a lead
    (True, True, 2, True, 3, True, "0a018b6000001000"),      # stop-and-go hold (near phase)
    (True, True, 2, True, 4, True, "0a018b8000001000"),      # stop-and-go hold (far phase)
    (True, True, 2, True, 3, False, "0a018b6000000000"),     # relaxed hold, ACC_ACTIVE_2 drops
    (True, True, 1, True, 2, True, "0a01874000001000"),      # driver gap 1 mirrored to the dash
  ])
  def test_crz_ctrl_golden_bytes(self, packer, long_active, acc_available, gap, has_lead, phase, acc_active_2, expected):
    dat = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, has_lead, phase, acc_active_2)[1]
    assert dat.hex() == expected

  def test_radar_frames_match_stock(self):
    expected = [
      (0x499, "0008c00000000000"),
      (0x361, "fff7fefe1fc00080"),
      (0x362, "fff7fefe1fc78c80"),
      (0x363, "fff7fefe1fc00000"),
      (0x364, "fff7fefe1fc00000"),
      (0x365, "fff7fe7ffbff3fc0"),
      (0x366, "fff7fe7ffbff3fc0"),
    ]
    frames = mazdacan.create_radar_frames(0, 0, None)
    assert [(f.address, f.dat.hex()) for f in frames] == expected

  def test_radar_frames_counter_and_lead_track(self):
    frames = mazdacan.create_radar_frames(2, 15, (mazdacan.LEAD_TRACK_DIST, 0.))
    assert all(f.src == 2 for f in frames)
    # counter stamps the low nibble of the last byte on every track
    assert [f.dat[7] & 0x0f for f in frames[1:]] == [15] * 6
    tracks = {f.address: f.dat.hex() for f in frames}
    assert tracks[0x364] == "0a4000001dc0000f"

  def test_lead_track_at_template_range_is_the_capture(self):
    assert mazdacan.create_lead_track(mazdacan.LEAD_TRACK_DIST, 0.) == mazdacan.LEAD_TRACK_TEMPLATE

  @pytest.mark.parametrize("d_rel,v_rel", [
    (0., 0.), (6.5, 1.5), (10.25, -2.0), (29.4, 2.9375), (255.875, 63.9375), (400., 100.), (5., -80.),
  ])
  def test_lead_track_round_trips_through_the_dbc(self, d_rel, v_rel):
    dat = mazdacan.create_lead_track(d_rel, v_rel)
    cp = CANParser("mazda_2017", [("RADAR_TRACK_364", float("nan"))], 0)
    cp.update([(0, [(0x364, dat, 0)])])
    vl = cp.vl["RADAR_TRACK_364"]
    assert vl["DIST_OBJ"] == pytest.approx(min(max(d_rel, 0.), 255.875), abs=0.0625)
    assert vl["RELV_OBJ"] == pytest.approx(min(max(v_rel, -64.), 63.9375), abs=0.0625)
    # the bits outside the two fields we drive stay exactly as captured
    assert dat[1] & 0x0f == mazdacan.LEAD_TRACK_TEMPLATE[1] & 0x0f
    assert dat[2] == mazdacan.LEAD_TRACK_TEMPLATE[2]
    assert dat[4] & 0x1f == mazdacan.LEAD_TRACK_TEMPLATE[4] & 0x1f
    assert dat[5:] == mazdacan.LEAD_TRACK_TEMPLATE[5:]


class TestSteeringOverlay:
  """The engaged 0x243 overlays the camera's exact frame: torque, counter and angle bits
  are ours, the line-visibility bit is forced off (the EPS gates torque on it), and every
  other bit, defined or not, is the camera's."""

  LKAS = {"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 1,
          "STEERING_ANGLE": 0, "ANGLE_ENABLED": 0}
  # bits no DBC signal describes, per byte
  UNDEFINED = {2: 0x76, 3: 0x9F, 4: 0xFC, 6: 0x2F}

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  @classmethod
  def _steer(cls, packer, ctr=0, torque=0, lkas=None, cam_raw=None):
    class FakeCP:
      flags = MazdaFlags.GEN1
    return mazdacan.create_steering_control(packer, FakeCP(), ctr, torque,
                                            dict(cls.LKAS if lkas is None else lkas), cam_raw)[1]

  def test_overlay_matches_curated_on_a_defined_camera_frame(self, packer):
    # a camera frame carrying no undefined bits: the overlay must reproduce the curated
    # build byte for byte, checksum included
    cam = self._steer(packer, ctr=5, torque=-200)
    overlay = self._steer(packer, ctr=9, torque=300, cam_raw=int.from_bytes(cam, "big"))
    assert overlay == self._steer(packer, ctr=9, torque=300)

  def test_undefined_bits_ride_through(self, packer):
    cam = bytearray(self._steer(packer, ctr=5, torque=-200))
    for i, mask in self.UNDEFINED.items():
      cam[i] |= mask
    cam[2] |= 0x80   # the camera's LDW alert bit rides through too
    overlay = self._steer(packer, ctr=9, torque=300, cam_raw=int.from_bytes(cam, "big"))
    for i, mask in self.UNDEFINED.items():
      assert overlay[i] & mask == cam[i] & mask
    parser = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)
    parser.update([(0, [(0x243, overlay, 0)])])
    vl = parser.vl["CAM_LKAS"]
    for k in ("BIT_1", "ERR_BIT_2", "LDW"):
      assert vl[k] == 1
    assert vl["CTR"] == 9
    assert vl["LKAS_REQUEST"] == 300

  def test_line_not_visible_is_forced_off(self, packer):
    # the EPS gates torque on the camera's visibility state (v4 on-device: openpilot
    # could only steer while the camera saw lanes), so the overlay clears it and the
    # checksum delta pays for the removal
    cam = bytearray(self._steer(packer, ctr=5, torque=-200))
    cam[2] |= mazdacan.LKAS_LNV_MASK_B2
    cam[7] = (cam[7] - mazdacan.LKAS_LNV_MASK_B2) % 256   # a checksum valid for LNV set
    overlay = self._steer(packer, ctr=9, torque=300, cam_raw=int.from_bytes(cam, "big"))
    assert overlay == self._steer(packer, ctr=9, torque=300)

  def test_camera_angle_bits_replaced_with_ours(self, packer):
    # a camera frame carrying a nonzero angle and enable: the overlay swaps in our
    # zero-angle pattern and the checksum delta pays for the removal, so the result is
    # the curated build byte for byte (the curated path never writes the camera's angle)
    vals = {"CTR": 5, "LKAS_REQUEST": -200, "STEERING_ANGLE": -300, "ANGLE_ENABLED": 1,
            "LDW": 0, "LINE_NOT_VISIBLE": 1, "BIT_1": 1, "ERR_BIT_2": 1}
    angled = bytearray(packer.make_can_msg("CAM_LKAS", 0, dict(vals, CHKSUM=0))[1])
    tmp = -200 + 2048
    angled[7] = (249 - 5 - (tmp >> 8) - (tmp & 0xFF) - (1 << 3) - (1 << 4) - (1 << 5)
                 - mazdacan._angle_checksum_terms(-300, 1)) % 256
    overlay = self._steer(packer, ctr=9, torque=300, cam_raw=int.from_bytes(angled, "big"),
                          lkas=dict(self.LKAS, STEERING_ANGLE=-300, ANGLE_ENABLED=1))
    assert overlay == self._steer(packer, ctr=9, torque=300)

  def test_write_masks_match_the_packer(self, packer):
    # flipping every owned field (counter, torque, angle, enable) may only flip bits
    # inside the write masks; bytes 2 and 3 must never move
    base = self._steer(packer, ctr=0, torque=0)
    variants = (
      self._steer(packer, ctr=10, torque=900),
      self._steer(packer, ctr=0, torque=0, lkas=dict(self.LKAS, STEERING_ANGLE=-300, ANGLE_ENABLED=1)),
    )
    for other in variants:
      assert other[2] == base[2] and other[3] == base[3]
      for i in range(7):
        diff = base[i] ^ other[i]
        assert diff & mazdacan.LKAS_WRITE_MASKS.get(i, 0) == diff, \
          f"byte {i}: packer flipped bits outside the write mask"


class TestLaneinfoRelay:
  """CAM_LANEINFO relay: the camera's frame reaches the dash byte for byte; while
  openpilot steers, only the steering-assist indicator bits are its own."""

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  @staticmethod
  def _cam_dat(packer, values=None):
    _, dat, _ = packer.make_can_msg("CAM_LANEINFO", 2, dict(values or {}))
    return bytearray(dat)

  def test_camera_frame_relays_byte_for_byte(self, packer):
    # bits the DBC does not describe (all of byte 2 among them) must survive the trip
    cam_dat = self._cam_dat(packer)
    cam_dat[2] = 0xAB
    cam_dat[5] = 0xCD
    relay = mazdacan.create_laneinfo_relay(int.from_bytes(cam_dat, "big"))
    assert relay.dat == bytes(cam_dat)
    assert relay.address == 0x440 and relay.src == 0

  def test_indicator_bits_set_and_clear_touch_nothing_else(self, packer):
    cam_dat = self._cam_dat(packer)
    cam_dat[2] = 0xAB
    raw = int.from_bytes(cam_dat, "big")
    for lit in (True, False):
      relay = mazdacan.create_laneinfo_relay(raw, lit)
      assert relay.dat[:6] == bytes(cam_dat[:6])
      b6 = (cam_dat[6] | mazdacan.STEER_IND_B6) if lit else (cam_dat[6] & (0xFF ^ mazdacan.STEER_IND_B6))
      b7 = (cam_dat[7] | mazdacan.STEER_IND_B7) if lit else (cam_dat[7] & (0xFF ^ mazdacan.STEER_IND_B7))
      assert relay.dat[6] == b6
      assert relay.dat[7] == b7

  def test_indicator_masks_match_the_packer_mapping(self, packer):
    # the masks must hit exactly the bits the packer assigns to the HANDS_* signals
    dark = self._cam_dat(packer)
    lit = self._cam_dat(packer, {"HANDS_WARN_3_BITS": 0b111, "HANDS_ON_STEER_WARN": 1,
                                 "HANDS_ON_STEER_WARN_2": 1})
    diff = [(i, a ^ b) for i, (a, b) in enumerate(zip(dark, lit, strict=True)) if a != b]
    assert diff == [(6, mazdacan.STEER_IND_B6), (7, mazdacan.STEER_IND_B7)]

  def test_line_suppression_touches_only_the_lanes_field(self, packer):
    cam_dat = self._cam_dat(packer, {"LANE_LINES": 4})
    cam_dat[2] = 0xAB
    relay = mazdacan.create_laneinfo_relay(int.from_bytes(cam_dat, "big"), suppress_lines=True)
    expected = bytearray(cam_dat)
    expected[1] = (expected[1] & (0xFF ^ mazdacan.LANE_LINES_MASK_B1)) | 1
    assert relay.dat == bytes(expected)
    # the mask covers exactly the bits the packer assigns to LANE_LINES
    other = self._cam_dat(packer, {"LANE_LINES": 1})
    assert (cam_dat[1] ^ other[1]) & mazdacan.LANE_LINES_MASK_B1 == cam_dat[1] ^ other[1]

  def test_camera_signals_decode_back(self, packer):
    values = {"LANE_LINES": 2, "LDW_WARN_LL": 1, "LDW_WARN_RL": 0, "TJA": 3,
              "TJA_TRANSITION": 2, "S1": 1, "S1_HBEAM": 1, "ERR_BIT": 1}
    cam_dat = self._cam_dat(packer, values)
    relay = mazdacan.create_laneinfo_relay(int.from_bytes(cam_dat, "big"))
    parser = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)
    parser.update([(0, [(0x440, relay.dat, 0)])])
    for k, v in values.items():
      assert parser.vl["CAM_LANEINFO"][k] == v


class TestSteeringCommand:
  """CAM_LKAS re-send: the camera's health and lane-departure bits ride along on every
  steering command."""

  @pytest.fixture
  def packer(self):
    return CANPacker("mazda_2017")

  @staticmethod
  def _steer_msg(packer, lkas=None, ctr=0, torque=0):
    class FakeCP:
      flags = MazdaFlags.GEN1
    lkas = {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0, "LDW": 0, "LINE_NOT_VISIBLE": 0} if lkas is None else lkas
    _, dat, _ = mazdacan.create_steering_control(packer, FakeCP(), ctr, torque, dict(lkas))
    parser = CANParser("mazda_2017", [("CAM_LKAS", 0)], 0)
    parser.update([(0, [(0x243, dat, 0)])])
    return parser.vl["CAM_LKAS"]

  def test_camera_bits_relay(self, packer):
    lkas = {"BIT_1": 1, "ERR_BIT_1": 1, "ERR_BIT_2": 1, "LDW": 1, "LINE_NOT_VISIBLE": 1}
    vl = self._steer_msg(packer, lkas)
    for k in ("BIT_1", "ERR_BIT_1", "ERR_BIT_2"):
      assert vl[k] == 1
    assert vl["LKAS_REQUEST"] == 0
    # the EPS gates torque on the visibility state, so the curated build never sends it;
    # the overlay carries the camera's alert bit from the raw bytes instead
    assert vl["LDW"] == 0
    assert vl["LINE_NOT_VISIBLE"] == 0

  def test_counter_wraps_at_sixteen(self, packer):
    for ctr in (0, 7, 15, 16, 33):
      assert self._steer_msg(packer, ctr=ctr)["CTR"] == ctr % 16

  def test_torque_round_trips(self, packer):
    for torque in (0, 100, -100, 1200):
      assert self._steer_msg(packer, torque=torque)["LKAS_REQUEST"] == torque


class TestRelayEmission:
  """Drives the real interface: the controller emits its own 0x243 every frame regardless of
  engagement, and relays the camera's 0x440 the moment a new camera frame lands (with a 2 Hz
  hold on the last frame once the camera has been quiet past the stale window). The panda,
  not the controller, yields the bus to the camera while disengaged."""

  CAM_LKAS_VALUES = {"BIT_1": 1, "ERR_BIT_1": 0, "ERR_BIT_2": 1, "LDW": 1, "LINE_NOT_VISIBLE": 1, "CTR": 5}
  CAM_LANEINFO_VALUES = {"LANE_LINES": 2, "LDW_WARN_LL": 1, "LDW_WARN_RL": 0, "TJA": 3,
                         "TJA_TRANSITION": 1, "HANDS_WARN_3_BITS": 0b101}

  @pytest.fixture
  def ci(self):
    CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=False,
                                 is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [],
                                       alpha_long=False, is_release_sp=False, docs=False)
    assert not CP.openpilotLongitudinalControl
    return CarInterface(CP, CP_SP)

  def _feed_camera(self, ci):
    packer = CANPacker("mazda_2017")
    msgs = [packer.make_can_msg("CAM_LKAS", 2, self.CAM_LKAS_VALUES),
            packer.make_can_msg("CAM_LANEINFO", 2, self.CAM_LANEINFO_VALUES)]
    for i in range(2):
      ci.update([(int(i * DT_CTRL * 1e9), [(m[0], m[1], m[2]) for m in msgs])])
    return msgs[0][1], msgs[1][1]

  @staticmethod
  def _control(enabled=False, lat_active=False, torque=0.0, steer_required=False):
    CC = structs.CarControl.new_message()
    CC.enabled = enabled
    CC.latActive = lat_active
    CC.actuators.torque = torque
    if steer_required:
      CC.hudControl.visualAlert = structs.CarControl.HUDControl.VisualAlert.steerRequired
    # card hands the controller a reader off the wire; update() calls actuators.as_builder()
    return CC.as_reader()

  def _apply(self, ci, i, CC):
    return ci.apply(CC, structs.CarControlSP(), int(i * DT_CTRL * 1e9))

  @staticmethod
  def _decode(sends, addr):
    dat = next(d for a, d, b in sends if a == addr)
    name = "CAM_LKAS" if addr == 0x243 else "CAM_LANEINFO"
    parser = CANParser("mazda_2017", [(name, 0)], 0)
    parser.update([(0, [(addr, dat, 0)])])
    return parser.vl[name]

  def test_disengaged_emission_and_relay(self, ci):
    _, cam_dat = self._feed_camera(ci)
    CC = self._control()
    steer_frames = 0
    steer_at = {}
    hud_at = {}
    for i in range(250):
      _, sends = self._apply(ci, i, CC)
      steer_frames += sum(1 for a, _, _ in sends if a == 0x243)
      if any(a == 0x243 for a, _, _ in sends):
        steer_at[i] = self._decode(sends, 0x243)
      if any(a == 0x440 for a, _, _ in sends):
        hud_at[i] = next(d for a, d, b in sends if a == 0x440)

    # 0x243 at 100 Hz with zero torque; the HUD relay fires on the first camera frame,
    # then only the 2 Hz hold fires while the camera stays quiet
    assert steer_frames == 250
    assert sorted(hud_at) == [0, 101, 151, 201]

    steer_vl = steer_at[100]
    assert steer_vl["LKAS_REQUEST"] == 0
    for k, v in self.CAM_LKAS_VALUES.items():
      if k not in ("CTR", "LINE_NOT_VISIBLE"):
        assert steer_vl[k] == v
    # the camera says the line is not visible; the EPS must not be told that
    assert steer_vl["LINE_NOT_VISIBLE"] == 0
    assert steer_vl["CTR"] == 100 % 16

    # the camera's HUD frame reaches the dash byte for byte, its indicator state intact
    for dat in hud_at.values():
      assert dat == cam_dat

  def test_engaged_emission_and_relay(self, ci):
    cam_lkas_dat, cam_dat = self._feed_camera(ci)
    CC = self._control(enabled=True, lat_active=True, torque=0.1)
    steer_at = {}
    steer_dat = {}
    hud_at = {}
    for i in range(250):
      _, sends = self._apply(ci, i, CC)
      if any(a == 0x243 for a, _, _ in sends):
        steer_at[i] = self._decode(sends, 0x243)
        steer_dat[i] = next(d for a, d, b in sends if a == 0x243)
      if any(a == 0x440 for a, _, _ in sends):
        hud_at[i] = next(d for a, d, b in sends if a == 0x440)

    # the HUD frame is the camera's with the indicator cleared and the lane
    # display quiet ("on, no lines"): while openpilot steers quietly, neither the
    # camera's "LAS applying torque" nag nor its lines belong on the dash
    expected = bytearray(cam_dat)
    expected[6] &= 0xFF ^ mazdacan.STEER_IND_B6
    expected[7] &= 0xFF ^ mazdacan.STEER_IND_B7
    expected[1] = (expected[1] & (0xFF ^ mazdacan.LANE_LINES_MASK_B1)) | 1
    assert sorted(hud_at) == [0, 101, 151, 201]
    for dat in hud_at.values():
      assert dat == bytes(expected)

    # the steering frame overlays the camera's: the counter continues the camera's
    # sequence (camera CTR 5 -> ours starts at 6), torque flows, and every bit outside
    # the write masks stays the camera's own -- LDW included
    assert [steer_at[i]["CTR"] for i in range(6)] == [6, 7, 8, 9, 10, 11]
    assert steer_at[100]["LKAS_REQUEST"] > 0
    for k, v in self.CAM_LKAS_VALUES.items():
      if k not in ("CTR", "LINE_NOT_VISIBLE"):
        assert steer_at[100][k] == v
    assert steer_at[100]["LINE_NOT_VISIBLE"] == 0
    out = steer_dat[100]
    assert out[3] == cam_lkas_dat[3]
    for i, mask in mazdacan.LKAS_WRITE_MASKS.items():
      assert out[i] & (0xFF ^ mask) == cam_lkas_dat[i] & (0xFF ^ mask)

  def test_engaged_steer_required_lights_the_steering_assist_indicator(self, ci):
    cam_dat = self._feed_camera(ci)[1]
    CC = self._control(enabled=True, lat_active=True, steer_required=True)
    hud_at = {}
    for i in range(250):
      _, sends = self._apply(ci, i, CC)
      if any(a == 0x440 for a, _, _ in sends):
        hud_at[i] = next(d for a, d, b in sends if a == 0x440)
    # the pre-branch channel is back: openpilot's hold-the-wheel alerts reach the dash
    expected = bytearray(cam_dat)
    expected[6] |= mazdacan.STEER_IND_B6
    expected[7] |= mazdacan.STEER_IND_B7
    for dat in hud_at.values():
      assert dat == bytes(expected)

  def test_new_camera_frame_relays_immediately(self, ci):
    self._feed_camera(ci)
    CC = self._control()
    for i in range(10):
      _, sends = self._apply(ci, i, CC)
      if i > 0:
        assert not any(a == 0x440 for a, _, _ in sends)

    # a fresh camera frame goes out on the very cycle it arrives
    values = dict(self.CAM_LANEINFO_VALUES, LDW_WARN_RL=1)
    dat = CANPacker("mazda_2017").make_can_msg("CAM_LANEINFO", 2, values)[1]
    ci.update([(int(11 * DT_CTRL * 1e9), [(0x440, dat, 2)])])
    _, sends = self._apply(ci, 11, CC)
    assert next(d for a, d, b in sends if a == 0x440) == dat

  def test_reengage_reseeds_the_counter(self, ci):
    self._feed_camera(ci)
    engaged = self._control(enabled=True, lat_active=True)
    off = self._control()
    ctrs = []
    for CC in (engaged, off, engaged):
      for _ in range(5):
        _, sends = self._apply(ci, 0, CC)
        ctrs.append(self._decode(sends, 0x243)["CTR"])
    # camera CTR 5: every engage edge restarts our sequence at 6
    assert ctrs[0] == ctrs[10] == 6

  def test_no_camera_frame_holds_the_zero_frame(self, ci):
    for i in range(2):
      ci.update([(int(i * DT_CTRL * 1e9), [])])
    CC = self._control(enabled=True, lat_active=True, steer_required=True)
    hud_at = {}
    for i in range(200):
      _, sends = self._apply(ci, i, CC)
      if any(a == 0x440 for a, _, _ in sends):
        hud_at[i] = next(d for a, d, b in sends if a == 0x440)
    # nothing from the camera: only the stale hold fires, zeros under the indicator bits
    assert sorted(hud_at) == [100, 150]
    assert hud_at[100] == bytes([0, 0, 0, 0, 0, 0, mazdacan.STEER_IND_B6, mazdacan.STEER_IND_B7])


class TestStandstillHold:

  @pytest.fixture
  def sm(self):
    return StandstillHold()

  @staticmethod
  def run(sm, frames, **kwargs):
    defaults = dict(long_active=True, stopping=False, standstill=False, plan_accel=-1.024,
                    brake_hold=False, lead_visible=True)
    defaults.update(kwargs)
    for _ in range(frames):
      sm.update(**defaults)
    return sm

  def test_holds_while_the_plan_is_stopping(self, sm):
    self.run(sm, 1)
    assert not sm.holding
    self.run(sm, 1, stopping=True)
    assert sm.holding and sm.stop_bits and sm.acc_active_2
    assert sm.ctrl_phase() == 3
    # arriving at a standstill changes nothing: the plan is still asking for the brakes
    self.run(sm, 500, stopping=True, standstill=True)
    assert sm.holding and sm.stop_bits

  def test_hold_never_relaxes_on_its_own(self, sm):
    # the creep-into-the-lead regression: without the car taking the hold over, the command
    # must stay on the plan's brake no matter how long the stop lasts
    self.run(sm, 1, stopping=True)
    self.run(sm, int(30.0 / DT_CTRL), stopping=True, standstill=True)
    assert sm.holding and sm.stop_bits and sm.acc_active_2
    assert not sm.car_has_hold

  def test_relax_follows_the_car_taking_the_hold(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 10, stopping=True, standstill=True)
    assert not sm.car_has_hold
    self.run(sm, 1, stopping=True, standstill=True, brake_hold=True)
    # stop bits and ACC_ACTIVE_2 drop with the command, together, exactly as stock does
    assert sm.car_has_hold and not sm.stop_bits and not sm.acc_active_2
    # and it is not a latch: if the car lets go, we brake again
    self.run(sm, 1, stopping=True, standstill=True, brake_hold=False)
    assert not sm.car_has_hold and sm.stop_bits and sm.acc_active_2

  def test_released_when_the_plan_asks_to_move(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 500, stopping=True, standstill=True, brake_hold=True)
    assert sm.holding
    self.run(sm, 1, standstill=True, plan_accel=0.1)
    assert not sm.holding and not sm.car_has_hold
    assert sm.resume_unlatching
    assert sm.ctrl_phase() == 2

  def test_release_holds_for_as_long_as_the_plan_wants_to_move(self, sm):
    # the failed-resume regression: no release window to run out from under the plan
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    self.run(sm, int(5.0 / DT_CTRL), standstill=True, plan_accel=0.4)
    assert not sm.holding and not sm.stop_bits

  def test_hold_comes_back_if_the_plan_changes_its_mind(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    self.run(sm, 5, standstill=True, plan_accel=0.2)
    assert not sm.holding
    self.run(sm, 1, stopping=True, standstill=True, plan_accel=-1.0)
    assert sm.holding and sm.stop_bits

  def test_unlatch_pulses_once_at_the_release(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True)
    assert not sm.resume_unlatching
    self.run(sm, 1, standstill=True, plan_accel=0.1)
    assert sm.resume_unlatching
    self.run(sm, RESUME_UNLATCH_FRAMES, standstill=True, plan_accel=0.1)
    assert not sm.resume_unlatching

  def test_long_disengage_resets(self, sm):
    self.run(sm, 1, stopping=True)
    self.run(sm, 100, stopping=True, standstill=True, brake_hold=True)
    self.run(sm, 1, long_active=False)
    assert not sm.holding and not sm.car_has_hold and not sm.stop_bits

  def test_stop_abort_releases(self, sm):
    self.run(sm, 1, stopping=True)
    assert sm.holding
    # lead speeds up again before the car reaches standstill
    self.run(sm, 1, stopping=False, plan_accel=0.3)
    assert not sm.holding

  def test_lead_follows_only_a_steady_state(self, sm):
    # a lead is adopted once leadVisible has held for the debounce window, not before
    self.run(sm, LEAD_DEBOUNCE_FRAMES - 1, lead_visible=True)
    assert not sm.radar_has_lead() and sm.ctrl_phase() == 1
    self.run(sm, 1, lead_visible=True)
    assert sm.radar_has_lead() and sm.ctrl_phase() == 2
    # and dropped the same way
    self.run(sm, LEAD_DEBOUNCE_FRAMES - 1, lead_visible=False)
    assert sm.radar_has_lead()
    self.run(sm, 1, lead_visible=False)
    assert not sm.radar_has_lead()

  def test_lead_flicker_never_reaches_the_bus(self, sm):
    # the measured failure: a marginal 120 m vision lead toggled leadVisible 6 times in 1.4 s
    # (route 6bb2dc61c4 t+400); none of it may reach RADAR_HAS_LEAD or the track slot
    for frames, visible in ((15, True), (5, False), (7, True), (13, False), (10, True)):
      self.run(sm, frames, lead_visible=visible)
      assert not sm.radar_has_lead(), "a flickering lead leaked through the debounce"

  def test_disengage_resets_the_lead(self, sm):
    self.run(sm, 2 * LEAD_DEBOUNCE_FRAMES, lead_visible=True)
    assert sm.radar_has_lead()
    self.run(sm, 1, long_active=False)
    assert not sm.radar_has_lead()


def _mock_cc(long_active=True, accel=0.5, long_state=None, standstill=False, gas=False, override=False,
             resume=False, lead_visible=True, gap=2, available=True,
             stock_radar_alive=False, fsc_settled=True, handback=False, cruise_engaged=False,
             enabled=None, lead_d_rel=12.0, lead_v_rel=0.0, brake_hold=False):
  # openpilot is enabled whenever it is longitudinally active; a gas override is the case
  # where it stays enabled with longActive low
  enabled = long_active if enabled is None else enabled
  out = SimpleNamespace(standstill=standstill, gasPressed=gas,
                        cruiseState=SimpleNamespace(available=available, enabled=cruise_engaged))
  actuators = SimpleNamespace(accel=accel, longControlState=long_state)
  cruise = SimpleNamespace(resume=resume, override=override, cancel=False)
  hud = SimpleNamespace(leadVisible=lead_visible, leadDistanceBars=gap)
  cc = SimpleNamespace(enabled=enabled, longActive=long_active, actuators=actuators,
                       cruiseControl=cruise, hudControl=hud)
  cc_sp = SimpleNamespace(stockEcuHandBack=handback,
                          leadOne=SimpleNamespace(dRel=lead_d_rel, vRel=lead_v_rel))
  cs = SimpleNamespace(out=out, resume_button=0, brake_hold=brake_hold,
                       stock_radar_alive=stock_radar_alive, fsc_settled=fsc_settled)
  return cc, cc_sp, cs


@pytest.fixture
def cc():
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], alpha_long=True,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, {0: {}, 1: {}, 2: {}}, [], True, False, False)
  assert CP.openpilotLongitudinalControl
  return CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


def _long_frames(sends):
  """(ACCEL_CMD raw, CRZ_INFO.ACC_ACTIVE, CRZ_CTRL.CRZ_ACTIVE) from a bus 0 emission, or None."""
  info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
  ctrl = next((d for a, d, b in sends if a == 0x21c and b == 0), None)
  if info is None:
    return None
  cp = CANParser("mazda_2017", [("CRZ_INFO", float("nan")), ("CRZ_CTRL", float("nan"))], 0)
  cp.update([(0, [(0x21b, info, 0), (0x21c, ctrl, 0)])])
  return decode_accel_cmd_raw(info), cp.vl["CRZ_INFO"]["ACC_ACTIVE"], cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"]


def _lead_track(dat):
  """(DIST_OBJ, RELV_OBJ) decoded from a 0x364 track frame."""
  cp = CANParser("mazda_2017", [("RADAR_TRACK_364", float("nan"))], 0)
  cp.update([(0, [(0x364, dat, 0)])])
  return cp.vl["RADAR_TRACK_364"]["DIST_OBJ"], cp.vl["RADAR_TRACK_364"]["RELV_OBJ"]


def _step(cc, **kw):
  kw.setdefault("long_state", structs.CarControl.Actuators.LongControlState.pid)
  control, control_sp, carstate = _mock_cc(**kw)
  sends = cc.update_longitudinal(control, control_sp, carstate)
  cc.frame += 1
  return sends


class TestLongitudinalIntegration:
  """Drives the real CarController.update_longitudinal through an engage -> cruise -> stop ->
  hold -> resume timeline and checks the emitted CAN, not just the state machine in isolation."""

  def test_engaged_frame_rates_and_counters(self, cc):
    long = structs.CarControl.Actuators.LongControlState
    crz_info = crz_ctrl = radar_static = tester = 0
    for _ in range(100):  # 1 s at 100 Hz
      sends = _step(cc, long_state=long.pid, accel=1.0, gap=2)
      addrs = [a for a, _, _ in sends]
      buses = {a: [] for a, _, _ in sends}
      for a, _, b in sends:
        buses[a].append(b)
      crz_info += addrs.count(0x21b)
      crz_ctrl += addrs.count(0x21c)
      radar_static += addrs.count(0x499)
      tester += sum(1 for a, _, _ in sends if a == 0x764)
      # CRZ_INFO/CRZ_CTRL, when emitted, always go to both bus 0 and bus 2
      if 0x21b in buses:
        assert sorted(buses[0x21b]) == [0, 2]
        assert sorted(buses[0x21c]) == [0, 2]

    # 100 Hz loop: long msgs at 50 Hz (x2 buses), radar at 10 Hz (x2), tester at 2 Hz
    assert crz_info == crz_ctrl == 100    # 50 frames x 2 buses
    assert radar_static == 20             # 10 frames x 2 buses
    assert tester == 2                    # 2 Hz, single bus
    assert cc.long_counter == 50 and cc.radar_counter == 10

  def test_gap_setting_mirrors_driver(self, cc):
    for gap in (1, 2, 3):
      cc.frame = 0  # force emission on the first step
      sends = _step(cc, gap=gap, long_state=structs.CarControl.Actuators.LongControlState.pid)
      ctrl = next(dat for a, dat, b in sends if a == 0x21c and b == 0)
      cp = CANParser("mazda_2017", [("CRZ_CTRL", float("nan"))], 0)
      cp.update([(0, [(0x21c, ctrl, 0)])])
      assert cp.vl["CRZ_CTRL"]["DISTANCE_SETTING"] == gap

  def test_stop_emits_hold_then_relaxes(self, cc):
    long = structs.CarControl.Actuators.LongControlState

    def accel_cmd(sends):
      dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
      return None if dat is None else decode_accel_cmd_raw(dat)

    # approach the stop
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=False)
    # hold at a standstill: the command is the plan's own and must not relax on its own, no
    # matter how long the stop lasts (the creep-into-the-lead regression)
    cmds = []
    for _ in range(int(30.0 / 0.01)):
      cmd = accel_cmd(_step(cc, long_state=long.stopping, accel=-1.024, standstill=True))
      if cmd is not None:
        cmds.append(cmd)
    settled = cmds[len(cmds) // 2:]
    assert settled and set(settled) == {-1024}, f"hold command drifted off the plan: {sorted(set(settled))}"

    # once the body ECU takes the hold over, stock stops asking for the brakes and so do we
    relaxed = []
    for _ in range(int(1.0 / 0.01)):
      cmd = accel_cmd(_step(cc, long_state=long.stopping, accel=-1.024, standstill=True,
                            brake_hold=True))
      if cmd is not None:
        relaxed.append(cmd)
    assert relaxed and set(relaxed) == {round(CarControllerParams.ACCEL_HOLD_LATCHED * 1000)}

  def test_gas_override_stays_engaged(self, cc):
    """A gas press is an override, not a disengagement. The command goes to zero as on every
    other port, but the engaged bits stay set the way Honda drives CONTROL_ON off CC.enabled.
    Clearing them mid-decel takes the PCM out of ACC mode (docs/mazda-gas-override.md)."""
    long = structs.CarControl.Actuators.LongControlState

    # braking hard, then the driver taps the gas
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=-2.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-2.0)

    cmds = []
    for _ in range(100):  # 1 s of override
      sends = _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0.,
                    gas=True, override=True, cruise_engaged=True)
      frame = _long_frames(sends)
      if frame is not None:
        cmds.append(frame)

    raw, acc_active, crz_active = zip(*cmds, strict=True)
    assert all(acc_active), "ACC_ACTIVE dropped during a gas override"
    assert all(crz_active), "CRZ_ACTIVE dropped during a gas override"
    assert set(raw) == {0}, f"command should be zero through the override, got {sorted(set(raw))}"

  def test_command_slew_is_rate_limited(self, cc):
    """The plan can step; the wire should not. Windup is limited tightly because dumping the
    brake in one frame is what the driver feels, winddown loosely so braking is never delayed."""
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=-2.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-2.0)

    # plan jumps straight to +1.0: the command must ramp, not step
    prev = cc.accel_last
    for _ in range(5):
      _step(cc, long_state=long.pid, accel=1.0, cruise_engaged=True)
      assert cc.accel_last - prev == pytest.approx(CarControllerParams.ACCEL_WINDUP_LIMIT, abs=1e-6)
      prev = cc.accel_last

    # and the other way, at the looser winddown limit
    for _ in range(200):
      _step(cc, long_state=long.pid, accel=1.0, cruise_engaged=True)
    prev = cc.accel_last
    for _ in range(5):
      _step(cc, long_state=long.pid, accel=-3.0, cruise_engaged=True)
      assert cc.accel_last - prev == pytest.approx(CarControllerParams.ACCEL_WINDDOWN_LIMIT, abs=1e-6)
      prev = cc.accel_last

  def test_accel_last_tracks_the_wire_not_the_plan(self, cc):
    # update() reports accel_last as actuatorsOutput.accel, the way Toyota, Ford and Honda
    # report the value they sent. It must be the wire value, clip and hold included.
    long = structs.CarControl.Actuators.LongControlState

    # a plan beyond the envelope is reported clipped, not as asked
    for _ in range(400):
      sends = _step(cc, long_state=long.pid, accel=-9.0, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(CarControllerParams.ACCEL_MIN)
    frame = _long_frames(sends)
    if frame is not None:
      assert frame[0] == round(cc.accel_last * 1000)

    # the standstill hold is the plan's own command, and that is what gets reported
    for _ in range(int(0.5 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=True, cruise_engaged=True)
    assert cc.accel_last == pytest.approx(-1.5)

    # through a gas override we report the zero we actually send
    for _ in range(10):
      _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0., gas=True,
            override=True, cruise_engaged=True)
    assert cc.accel_last == 0.

  def test_gas_from_standstill_hold_releases_the_brake(self, cc):
    # gas out of a hold is a resume, not a slow release: the hold command must go straight to
    # zero rather than ramping off at the cruising override rate
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(int(3.0 / 0.01)):
      _step(cc, long_state=long.stopping, accel=-1.5, standstill=True, cruise_engaged=True)
    assert cc.accel_last < -0.5, "never reached the standstill hold"

    for _ in range(20):
      _step(cc, long_active=False, enabled=True, long_state=long.off, accel=0., gas=True,
            override=True, standstill=True, cruise_engaged=True)
    assert cc.accel_last == 0., f"hold not released for the driver's gas: {cc.accel_last}"

  def test_lead_track_follows_the_measured_lead(self, cc):
    # a frozen track is what latches the camera's SCBS fault, so the range we advertise has to
    # move with the lead we are actually following
    long = structs.CarControl.Actuators.LongControlState
    # let the lead debounce adopt the visible lead before sampling the track
    for _ in range(LEAD_DEBOUNCE_FRAMES):
      _step(cc, long_state=long.pid, accel=0.5, lead_visible=True, lead_d_rel=20.0, lead_v_rel=-1.5)
    seen = []
    for i in range(60):
      sends = _step(cc, long_state=long.pid, accel=0.5, lead_visible=True,
                    lead_d_rel=20.0 - 0.1 * i, lead_v_rel=-1.5)
      track = next((d for a, d, b in sends if a == 0x364 and b == 0), None)
      if track is not None:
        seen.append(_lead_track(track))
    assert len(seen) > 1
    dists = [d for d, _ in seen]
    assert all(a > b for a, b in zip(dists, dists[1:], strict=False)), f"range did not close with the lead: {dists}"
    assert all(v == pytest.approx(-1.5, abs=0.0625) for _, v in seen)

  def test_hold_fabricates_a_lead_but_drops_it_on_release(self, cc):
    # with no lead in view the hold still needs something to hold against, but carrying that
    # fabricated object through the release is what the camera latches on
    long = structs.CarControl.Actuators.LongControlState

    def tracks(sends):
      return [d for a, d, b in sends if a == 0x364 and b == 0]

    held = []
    for _ in range(int(3.0 / 0.01)):
      held += tracks(_step(cc, long_state=long.stopping, accel=-1.5, standstill=True,
                           lead_visible=False, cruise_engaged=True))
    assert held
    assert all(_lead_track(d)[0] == pytest.approx(mazdacan.LEAD_TRACK_DIST) for d in held)

    released = []
    for _ in range(50):
      released += tracks(_step(cc, long_state=long.pid, accel=0.3, standstill=True,
                               lead_visible=False, cruise_engaged=True))
    assert released
    empty = mazdacan.RADAR_TRACK_MSGS[0x364]
    assert all(d[:7] == empty[:7] for d in released), \
      f"fabricated lead survived the release: {released[0].hex()}"

  def test_resume_asks_while_the_plan_wants_to_move_and_the_car_has_not(self, cc):
    # the RES press has to outlast cruiseState.standstill, which drops for ~3 s after a press,
    # so it is keyed on the car actually still being stopped
    control, _, carstate = _mock_cc(standstill=True, accel=0.3)
    assert cc.resume_requested(control, carstate)

    # plan still braking: no press, even though the car is sitting in a hold
    control, _, carstate = _mock_cc(standstill=True, accel=-1.024)
    assert not cc.resume_requested(control, carstate)

    # car is rolling: the hold is gone, stop asking
    control, _, carstate = _mock_cc(standstill=False, accel=0.3)
    assert not cc.resume_requested(control, carstate)

    # not longitudinally active: never our press to send
    control, _, carstate = _mock_cc(long_active=False, enabled=True, standstill=True, accel=0.3)
    assert not cc.resume_requested(control, carstate)

  def test_resume_matches_the_hold_release(self, cc):
    # the press and the release run off the same condition, so the body is never asked to let
    # go while we are still commanding the brake
    long = structs.CarControl.Actuators.LongControlState
    for _ in range(200):
      _step(cc, long_state=long.stopping, accel=-1.024, standstill=True, cruise_engaged=True)
    control, _, carstate = _mock_cc(standstill=True, accel=-1.024)
    assert cc.stop_and_go.holding and not cc.resume_requested(control, carstate)

    _step(cc, long_state=long.pid, accel=0.3, standstill=True, cruise_engaged=True)
    control, _, carstate = _mock_cc(standstill=True, accel=0.3)
    assert not cc.stop_and_go.holding and cc.resume_requested(control, carstate)

  def test_gas_pedal_without_cruise_stays_disengaged(self, cc):
    # gas pressed while openpilot is not enabled must not advertise an engaged ACC
    off = structs.CarControl.Actuators.LongControlState.off
    cc.frame = 0
    sends = _step(cc, long_active=False, enabled=False, long_state=off, gas=True, available=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe2000480")  # armed-but-idle pattern, zero command

  def test_disengaged_emits_stock_patterns(self, cc):
    off = structs.CarControl.Actuators.LongControlState.off
    # main off, not available: the exact standby pattern the panda allowlists byte-for-byte
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=False)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe3ffc000")
    # MRCC armed but not engaged: stock advertises ACC_SET_ALLOWED with a zero command
    cc.frame = 0
    sends = _step(cc, long_active=False, long_state=off, available=True)
    info = next(dat for a, dat, b in sends if a == 0x21b and b == 0)
    assert info.hex().startswith("01ffe2000480")


SESSION_PROG_DAT = bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0])
SESSION_DFLT_DAT = bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])
TESTER_PRESENT_DAT = bytes([0x02, 0x3e, 0x80, 0, 0, 0, 0, 0])


class TestRadarSessionSequencing:
  """Boot teardown deferral and the ordered hand-back: what goes on the bus in each
  radar session state, driven through the real CarController.update_longitudinal."""

  def _step(self, cc, stock_radar_alive, fsc_settled, handback=False, cruise_engaged=False):
    off = structs.CarControl.Actuators.LongControlState.off
    return _step(cc, long_active=False, accel=0., long_state=off, lead_visible=False, available=False,
                 stock_radar_alive=stock_radar_alive, fsc_settled=fsc_settled,
                 handback=handback, cruise_engaged=cruise_engaged)

  @staticmethod
  def _uds(sends):
    return [dat for a, dat, b in sends if a == 0x764]

  @staticmethod
  def _synthetic(sends):
    return [a for a, _, _ in sends if a in (0x21b, 0x21c, 0x499)]

  def test_stock_state_is_silent(self, cc):
    # radar alive, gate not yet passed: nothing at all goes on the bus
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=False)
      assert sends == []

  def test_boot_teardown_sequence(self, cc):
    # gate passes with the stock radar alive: programming-session requests at 2 Hz,
    # still no synthetic frames and no tester present
    for i in range(100):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True)
      if i % CarControllerParams.RADAR_UDS_STEP == 0:
        assert self._uds(sends) == [SESSION_PROG_DAT]
      else:
        assert self._uds(sends) == []
      assert self._synthetic(sends) == []
    # radar goes quiet: synthetic frames + tester present take over, session requests stop
    saw_tester = False
    for _ in range(100):
      frame = cc.frame
      sends = self._step(cc, stock_radar_alive=False, fsc_settled=True)
      assert SESSION_PROG_DAT not in self._uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(self._synthetic(sends)) > 0
      saw_tester |= TESTER_PRESENT_DAT in self._uds(sends)
    assert saw_tester

  def test_handback_sequence(self, cc):
    # reach SILENCED
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    # hand-back requested: default-session requests at 2 Hz, tester present stops,
    # synthetic frames continue while the radar is still quiet
    saw_default = False
    for _ in range(100):
      frame = cc.frame
      sends = self._step(cc, stock_radar_alive=False, fsc_settled=True, handback=True)
      assert TESTER_PRESENT_DAT not in self._uds(sends)
      saw_default |= SESSION_DFLT_DAT in self._uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(self._synthetic(sends)) > 0
    assert saw_default
    # stock radar returns: everything stops
    for _ in range(200):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, handback=True)
      assert sends == []

  def test_handback_before_teardown_stops_everything(self, cc):
    # toggle-off while still waiting on the gate: no session ever entered, so no
    # hand-back traffic either
    self._step(cc, stock_radar_alive=True, fsc_settled=False)
    for _ in range(120):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=False, handback=True)
      assert sends == []

  def test_teardown_waits_for_stock_cruise_disengage(self, cc):
    # driver engaged stock MRCC before the gate passed (warm boot): hold the teardown
    for _ in range(120):
      sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, cruise_engaged=True)
      assert sends == []
    # driver disengages: teardown proceeds
    cc.frame = 0
    sends = self._step(cc, stock_radar_alive=True, fsc_settled=True, cruise_engaged=False)
    assert SESSION_PROG_DAT in self._uds(sends)

  def test_s3_recovery_resilences(self, cc):
    # radar reappears mid-drive (dropped tester present, S3 timeout): re-request the session
    self._step(cc, stock_radar_alive=False, fsc_settled=True)
    cc.frame = CarControllerParams.RADAR_UDS_STEP  # align to a session-request frame
    sends = self._step(cc, stock_radar_alive=True, fsc_settled=True)
    assert SESSION_PROG_DAT in self._uds(sends)
    # and settles back to silenced once quiet again
    sends = self._step(cc, stock_radar_alive=False, fsc_settled=True)
    assert SESSION_PROG_DAT not in self._uds(sends)
