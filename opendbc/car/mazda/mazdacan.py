from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons

# Captured empty radar tracks required by the body ECU for stop-and-go. Only the counter
# nibble changes; 0x364 carries the advertised lead when present.
RADAR_STATIC_MSG = (0x499, bytes.fromhex("0008c00000000000"))
RADAR_TRACK_MSGS = {
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc78c80"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
}
LEAD_TRACK_ADDR = 0x364
# Constant bytes for an occupied 0x364 track. create_lead_track replaces its measurements.
LEAD_TRACK_TEMPLATE = bytes.fromhex("000e00001c000000")
DIST_OBJ_SCALE = 0.0625   # m per bit, DIST_OBJ and RELV_OBJ share it
DIST_OBJ_MAX = 255.875    # m, the full-scale DIST_OBJ reading a track can carry

# The camera gates automatic high beams on a living 5/6 object world: replaying only the
# empty templates reads as a dead radar, and HBC stops deciding under the takeover. Each
# slot replays an 8 s stock capture (2022 CX-5, city drive) followed by an empty stretch
# sized to the stock duty cycle measured on the night drive where stock HBC was captured
# working (slots 5/6 at 59%/26% and 29%/12% across its segments, played at 40%/20%); the
# unequal periods keep the slots from cycling in lockstep. Bytes stay exact and the
# counter nibble is stamped per send like every track.
RADAR_TRACK_56_EMPTY = bytes.fromhex("fff7fe7ffbff3fc0")
RADAR_CLUTTER_ADDRS = (0x365, 0x366)
RADAR_CLUTTER_OCCUPIED = 80
RADAR_CLUTTER_EMPTY = {0x365: 120, 0x366: 320}
RADAR_CLUTTER_CYCLE = {
  0x365: bytes.fromhex(
    "1d21b433b83c27c11bd1c833203e27c21a91da32804127c31941f231e04227c418120631384427c516c22030984727c615823c2ff84927c714325a2f504b27c8"
    + "12f27e2ea85127c911b2a82e005327ca1062da2d585527cb0f031c2cb05727cc0db364ac105927cd0c73baab685927ce0b3420aac05a27cf09f496aa205a27c0"
    + "fff7fe29785b27c1837066a8d05b1bc2fff7fe28285c3fc380e06c27805f1bc47f806c26d8611bc57e306e2628631bc67ce06c2580631bc7fff7fe24d8623fc8"
    + "7a40702430621bc978f0722388641bca77a07422e0651bcb7650762238671bcc7500782190681bcd73b07a20e8681bce72607820406a1bcf7100761f906d1bc0"
    + "6fb0761ee86f1bc16e50761e406f1bc26d00761d90701bc36ba0781ce8731bc46a507c1c38771bc568f07a1b907a1bc667a07c9ae07d1bc76610801a18831bc8"
    + "64c0841968881bc963808418d08b1bca62408418288d1bcb60f0881780921bcc5f908c16d0961bcd5e20901610991bce5cd09015689d1bcf5b709414c0a21bc0"
    + "5a10941410a61bc158c0941360ac1bc257609412c0b11bc35610961210b71bc454a0981158bf1bc553409a10b0c71bc651c0a00ff0d11bc75080a40f48da1bc8"
    + "4f20a80ea0e41bc94dd0ae0df0ef1bca4c70b40d40fb1bcb4ae0ba0c810b1bcc4990bc0bd11a1bcd4810c00b112d1bce46d0c00a71401bcf4580c409c9591bc0"
    + "4420c83b584a188142c0cc7ffbff1bc24160ce39f04818834040d439404918843f00d8b8884918853dc0deb7d84a58863c60e2b7204918873b10e8b668495888"
    + "3980ec35b84818893800f2350848588a3670f6345849188b3500fa33a84b588c3390fc32f050188d3220feb23850188e30b106318852188f2f410e30d8531880"
  ) + RADAR_TRACK_56_EMPTY * RADAR_CLUTTER_EMPTY[0x365],
  0x366: bytes.fromhex(
    "5320a47ffbff37c151f0a87ffbff37c250b0ae7ffbff37c34f70b27ffbff37c44e20b47ffbff37c54ce0b67ffbff37c64b90be7ffbff37c74a50c47ffbff37c8"
    + "4900c87ffbff37c947b0cc7ffbff37ca4670d47ffbff37cb4520da7ffbff37cc43d0e07ffbff37cd4280e47ffbff37ce4130e87ffbff37cf3ff0e47ffbff37c0"
    + "3eb0e67ffbff37c13d60e635284336023c10f034804436033ac0f833d84636043970fe3330473605382100328847360636d10031e04536073581023138443608"
    + "342106309044360932e10c2fe843360a3181102f4043360b3031162e9843360c2ee11c2df042360d2d81222d4042360e2c31282c9841360f2ae1302bf0413600"
    + "2981382b404136012831402a9841360226e14a29f041360325815429483f360424316028983d360522d16c27e83d360621717a27383d36071ff18e26703f3608"
    + "1e91a025c04336091d51b2252048360a1c01c8248049360b1ab1dc23d84b360c1961f423304e360d17f21622704f360e16a23621c852360f15425a2118583600"
    + "13d28620685936011282b41fb85b36021132e61f185d36030fe3221e705f36040e63741db0613605fff7fe1d08643e06fff7fe1c48673e07fff7fe1ba8683e08"
    + "fff7fe1b006a7e09fff7fe1a586e3e0afff7fe19a8713e0bfff7fe18e0743e0cfff7fe1838773e0dfff7fe17707b3e0efff7fe16d07f3e0ffff7fe1628833e00"
    + "86606e157888160184607c14d08b160282a08c14208f160380f09a13689416047f60a812b89916057e60ac1208a016067d40b29150a3160761f08290a0ab1208"
    + "fff7fe0ff0b13e097870c60f40bc160a7700c80e88c3160b7590c60dd8cd160c7420c68d20da160d72c0c68c68f3160e7140cc0bb105160f6fd0d00b01151600"
  ) + RADAR_TRACK_56_EMPTY * RADAR_CLUTTER_EMPTY[0x366],
}

# The G46L radar (2016.5 bodies) sends only this static frame and no track messages at
# all, so the lead rides CRZ_CTRL alone; fully static — no counter, no checksum.
G46L_RADAR_STATIC_MSG = (0x499, bytes.fromhex("0098400000000000"))


def crz_info_checksum(dat: bytes) -> int:
  # Invert the sum of the first seven bytes, excluding STOPPING and RESUME_UNLATCHING.
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04) - (dat[6] & 0x40)) & 0xFF)) & 0xFF


def create_acc_command(packer, bus, counter, accel, *, long_active, acc_available,
                       brake_pressed=False, stopping=False, resume_unlatching=False):
  # CRZ_INFO replaces the disabled radar's acceleration command and armed-idle state.
  values = {
    "ERROR_STATUS": 1,
    "STATIC_1": 0x7ff,
    "CTR": counter % 16,
    "ACCEL_CMD": accel if long_active else 4.094,  # stock non-controlling sentinel
    "NEW_SIGNAL_7": int(long_active or acc_available),
  }
  if long_active:
    values.update({
      "ACC_ACTIVE": 1,
      "ACC_SET_ALLOWED": 1,
      "STOPPING": int(stopping),
      "STOPPING_2": int(stopping),
      "RESUME_UNLATCHING": int(resume_unlatching),
    })
  elif acc_available:
    values["ACC_SET_ALLOWED"] = int(not brake_pressed)

  dat = packer.make_can_msg("CRZ_INFO", bus, values)[1]
  values["CHKSUM"] = crz_info_checksum(dat)
  return packer.make_can_msg("CRZ_INFO", bus, values)


def create_crz_ctrl(packer, bus, long_active, acc_available, gap_setting, radar_has_lead, stop_go_phase, acc_active_2):
  # CRZ_CTRL replaces radar cruise state and mirrors stop phase and driver gap selection.
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
  """Encode the advertised lead in the camera's track slot.

  Range must advance with relative velocity between measurements. RELV_OBJ uses positive
  values for an opening lead.
  """
  dist = round(min(max(d_rel, 0.), DIST_OBJ_MAX) / DIST_OBJ_SCALE)
  relv = round(min(max(v_rel, -64.), 63.9375) / DIST_OBJ_SCALE) & 0x7ff
  dat = bytearray(LEAD_TRACK_TEMPLATE)
  dat[0] = dist >> 4
  dat[1] = ((dist & 0xf) << 4) | (dat[1] & 0x0f)
  dat[3] = relv >> 3
  dat[4] = ((relv & 0x7) << 5) | (dat[4] & 0x1f)
  return bytes(dat)


def create_radar_frames(bus, counter, lead, g46l=False):
  """lead is the (dRel, vRel) of the object to advertise on 0x364, or None for an empty slot."""
  if g46l:
    return [CanData(G46L_RADAR_STATIC_MSG[0], G46L_RADAR_STATIC_MSG[1], bus)]
  frames = [CanData(RADAR_STATIC_MSG[0], RADAR_STATIC_MSG[1], bus)]
  for addr, dat in RADAR_TRACK_MSGS.items():
    if lead is not None and addr == LEAD_TRACK_ADDR:
      dat = create_lead_track(*lead)
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  for addr in RADAR_CLUTTER_ADDRS:
    cycle = RADAR_CLUTTER_CYCLE[addr]
    i = (counter % (len(cycle) // 8)) * 8
    dat = cycle[i:i + 8]
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  return frames


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
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

  ctr = frame % 16
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

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  # Preserve camera LKAS state. Keep TJA modes clear because its state machine does not own
  # the injected steering command.
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button, bus=0):
  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)
  # Only ever on the camera bus: the panda refuses it on the car's side, where it would toggle
  # MADS and arm MRCC in the body.
  tja = int(button == Buttons.TJA)
  assert not (tja and bus == 0)

  values = {
    "TJA_BUTTON": tja,

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

  return packer.make_can_msg("CRZ_BTNS", bus, values)
