from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons, MazdaFlags

# Radar frames the body ECU expects to keep receiving for stop-and-go to work. Byte-exact
# captures from a 0x764 radar with no objects in view; only the counter nibble in the last
# byte changes. 0x364 carries the lead we are following, if any.
RADAR_STATIC_MSG = (0x499, bytes.fromhex("0008c00000000000"))
RADAR_TRACK_MSGS = {
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc78c80"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
  0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
  0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
}
LEAD_TRACK_ADDR = 0x364
# An occupied track slot, captured from a 0x764 radar holding a stopped lead at 10.25 m.
# create_lead_track only rewrites DIST_OBJ and RELV_OBJ; the rest is the radar's track-valid
# pattern, which is not understood well enough to synthesize.
LEAD_TRACK_TEMPLATE = bytes.fromhex("0a4000001dc00000")
LEAD_TRACK_DIST = 10.25   # m, the template's own range
DIST_OBJ_SCALE = 0.0625   # m per bit, DIST_OBJ and RELV_OBJ share it


def crz_info_checksum(dat: bytes) -> int:
  # Inverted sum of the first seven bytes; the radar leaves the STOPPING bit out of the
  # sum. Verified against 1.94M stock frames, including every stop-bit frame.
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04)) & 0xFF)) & 0xFF


def create_acc_command(packer, bus, counter, accel, long_active, acc_available, stopping, resume_unlatching):
  # CRZ_INFO stands in for the disabled radar's accel command frame. While MRCC is armed
  # but not engaged, stock advertises ACC_SET_ALLOWED with a zero command so the dash
  # accepts SET; with the main switch off it broadcasts a static standby pattern with the
  # command field pegged high.
  values = {
    "STATUS": 1,
    "STATIC_1": 0x7ff,
    "CTR1": counter % 16,
  }
  if long_active or acc_available:
    values.update({
      "ACCEL_CMD": accel,
      "ACC_ACTIVE": int(long_active),
      "ACC_SET_ALLOWED": 1,
      "NEW_SIGNAL_7": 1,
      "STOPPING": int(stopping),
      "STOPPING_2": int(stopping),
      "RESUME_UNLATCHING": int(resume_unlatching),
    })
  else:
    values["ACCEL_CMD"] = 4.094  # standby pattern, raw 8190

  dat = packer.make_can_msg("CRZ_INFO", bus, values)[1]
  values["CHKSUM"] = crz_info_checksum(dat)
  return packer.make_can_msg("CRZ_INFO", bus, values)


def create_crz_ctrl(packer, bus, long_active, acc_available, gap_setting, radar_has_lead, stop_go_phase, acc_active_2):
  # CRZ_CTRL stands in for the disabled radar's cruise-state frame. stop_go_phase mirrors
  # stock's stop-and-go progression through RADAR_LEAD_RELATIVE_DISTANCE (see the DBC
  # comment); gap_setting mirrors the driver's distance setting on the dash.
  values = {
    "MSG_1_INV": 1,
    "MSG_1_INV_COPY": 1,
    "NEW_SIGNAL_8": 1,
    "CRZ_ACTIVE": int(long_active),
    "CRZ_AVAILABLE": int(long_active or acc_available),
    "DISTANCE_SETTING": gap_setting,
    "RADAR_HAS_LEAD": int(radar_has_lead),
    "RADAR_LEAD_RELATIVE_DISTANCE": stop_go_phase,
    "ACC_ACTIVE_2": int(acc_active_2),
  }
  return packer.make_can_msg("CRZ_CTRL", bus, values)


def create_lead_track(d_rel: float, v_rel: float) -> bytes:
  """Place the lead we are following on the track slot the camera reads.

  A stock radar re-measures every track every 100 ms, so its range and range rate move with
  the lead even at a standstill. Repeating one frozen frame instead makes the camera latch an
  SCBS fault the moment a standstill hold releases: it is told an object sits at a fixed range
  with zero closing speed while the car is commanded to drive off, which its own view of the
  lead pulling away contradicts. RELV_OBJ carries the same sign as vRel, positive opening.
  """
  dist = round(min(max(d_rel, 0.), 255.875) / DIST_OBJ_SCALE)
  relv = round(min(max(v_rel, -64.), 63.9375) / DIST_OBJ_SCALE) & 0x7ff
  dat = bytearray(LEAD_TRACK_TEMPLATE)
  dat[0] = dist >> 4
  dat[1] = ((dist & 0xf) << 4) | (dat[1] & 0x0f)
  dat[3] = relv >> 3
  dat[4] = ((relv & 0x7) << 5) | (dat[4] & 0x1f)
  return bytes(dat)


def create_radar_frames(bus, counter, lead):
  """lead is the (dRel, vRel) of the object to advertise on 0x364, or None for an empty slot."""
  frames = [CanData(RADAR_STATIC_MSG[0], RADAR_STATIC_MSG[1], bus)]
  for addr, dat in RADAR_TRACK_MSGS.items():
    if lead is not None and addr == LEAD_TRACK_ADDR:
      dat = create_lead_track(*lead)
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  return frames


# CAM_LKAS bits the controller owns, probed against the packer: CTR owns byte 0's high
# nibble, the torque field byte 0's low nibble plus byte 1, the angle fields bytes 4-6.
# LINE_NOT_VISIBLE is forced off (the EPS gates torque on it); every other bit rides through.
LKAS_WRITE_MASKS = {0: 0xFF, 1: 0xFF, 2: 0x08, 4: 0x03, 5: 0xFF, 6: 0xD0}
LKAS_LNV_MASK_B2 = 0x08


def _angle_checksum_terms(steering_angle: int, angle_enabled: int) -> int:
  # the checksum contribution the curated formula assigns to the angle fields
  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2
  return ahi + amd + alo + angle_enabled - (15 if ahi == 1 else 0)


def _overlay_steering_control(ours: bytes, cam_raw: int, lkas) -> bytes:
  dat = bytearray(cam_raw.to_bytes(8, "big"))

  # checksum delta over exactly the fields written below; the camera's own checksum
  # already covers every other bit, defined or not
  csum = dat[7]
  csum -= (ours[0] >> 4) - (dat[0] >> 4)
  csum -= (ours[0] & 0x0F) - (dat[0] & 0x0F)
  csum -= ours[1] - dat[1]
  csum += dat[2] & LKAS_LNV_MASK_B2   # visibility bit forced off; removing it adds back
  csum += _angle_checksum_terms(int(lkas["STEERING_ANGLE"]), int(lkas["ANGLE_ENABLED"]))
  csum -= _angle_checksum_terms(0, 0)

  for i, mask in LKAS_WRITE_MASKS.items():
    dat[i] = (dat[i] & (0xFF ^ mask)) | (ours[i] & mask)
  dat[7] = csum % 256
  return bytes(dat)


def create_steering_control(packer, CP, ctr, apply_torque, lkas, cam_raw: int | None = None):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  # LDW stays zero: the overlay carries the camera's alert bit from its raw bytes
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = ctr % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  if cam_raw is None:
    return packer.make_can_msg("CAM_LKAS", 0, values)
  # overlay: the controller's torque/counter/angle bits written into the camera's exact frame
  ours = packer.make_can_msg("CAM_LKAS", 0, values)[1]
  return CanData(0x243, _overlay_steering_control(ours, cam_raw, lkas), 0)


CAM_LANEINFO_ADDR = 0x440
# Hands-warn bits, byte positions matching the packer mapping: HANDS_WARN_3_BITS sits in
# byte 6, the two single-bit warnings in byte 7
HANDS_WARN_B6 = 0x0E
HANDS_WARN_B7 = 0x09
LANE_LINES_MASK_B1 = 0x07   # LANE_LINES, 0 = LKAS disabled


def create_laneinfo_relay(cam_raw: int | None, hands: bool | None = None, suppress_lines: bool = False):
  # byte-for-byte: bits the DBC does not describe must reach the dash as the camera sent them
  dat = bytearray(8 if cam_raw is None else cam_raw.to_bytes(8, "big"))
  if hands is not None:
    if hands:
      dat[6] |= HANDS_WARN_B6
      dat[7] |= HANDS_WARN_B7
    else:
      dat[6] &= 0xFF ^ HANDS_WARN_B6
      dat[7] &= 0xFF ^ HANDS_WARN_B7
  if suppress_lines:
    dat[1] &= 0xFF ^ LANE_LINES_MASK_B1
  return CanData(CAM_LANEINFO_ADDR, bytes(dat), 0)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
