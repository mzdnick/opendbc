"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import deque
from enum import StrEnum

from opendbc.car import Bus, DT_CTRL, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV

# MORE_GAS (0x167) byte 7, the PCM's cylinder-status state machine.
CYL_MODE_NORMAL = 0x29
CYL_MODE_ENTRY_FIRST = 0x0A   # the entry ramp counts up from here ...
CYL_MODE_ENTRY_LAST = 0x11    # ... to here, then the latched state follows
CYL_MODE_DEACTIVATED = 0x14
CYL_MODE_EXITS = (0x1E, 0x28)  # exit transients on the way back to all cylinders

# The byte reports CYL_MODE_NORMAL during fuel cut, so engine braking is inferred from
# physics: pedal up, rolling, and the engine still coupled. Coupling means the rpm/speed
# ratio holds one gear line; unlock, a shift, or idle-fuel float makes it drift. There is
# deliberately no decel gate: fuel cut survives grades where the car holds speed. The
# confirm time covers the ~0.5 s the intake takes to empty after lift-off. The bus carries
# no discrete fuel-cut signal.
# The two speed floors are not one threshold twice: EB_MIN_V_EGO gates on the filtered
# vEgo, EB_RATIO_MIN_KPH on raw ENGINE_DATA speed. Braking to a stop, the raw signal dips
# below its floor first and empties the coupling window at crawl, where the gear lines
# crowd idle and the ratio is noise.
EB_MIN_V_EGO = 3.0            # m/s of filtered vEgo; the overrun gate
EB_CONFIRM_S = 0.5
EB_CONFIRM_FRAMES = round(EB_CONFIRM_S / DT_CTRL)   # round never shortens the confirm time
EB_LOCK_WINDOW_S = 0.5        # trailing window that proves the ratio flat
EB_LOCK_FRAMES = round(EB_LOCK_WINDOW_S / DT_CTRL)
EB_RATIO_SPREAD_FRAC = 0.1    # the window's p95-p5 must stay within this fraction of its median
EB_RATIO_MIN_KPH = 8.0        # raw ENGINE_DATA kph; empties the window below this


class CylinderState(StrEnum):
  # Values match the CarStateZP.CylinderDeactivation.State enumerants in custom.capnp.
  normal = "normal"
  entry = "entry"
  deactivated = "deactivated"
  engineBraking = "engineBraking"


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.cyl_state = CylinderState.normal
    self.cyl_entry_progress = 0.0
    self.eb_confirm_frames = 0
    self.eb_ratio_hist = deque(maxlen=EB_LOCK_FRAMES)

  def _track_coupling(self, rpm: float, speed_kph: float) -> bool:
    """True while the trailing rpm/speed ratio stays flat (converter locked)."""
    if speed_kph < EB_RATIO_MIN_KPH or rpm <= 0.0:
      self.eb_ratio_hist.clear()
      return False
    self.eb_ratio_hist.append(rpm / speed_kph)
    if len(self.eb_ratio_hist) < EB_LOCK_FRAMES:
      return False
    s = sorted(self.eb_ratio_hist)
    # A fraction of the window median, not an absolute rpm/kph band: gear lines sit
    # roughly 5x higher in 1st than in top gear, so one absolute band would be a tight
    # gate in low gears and a loose one in high gears.
    return s[len(s) * 95 // 100] - s[len(s) * 5 // 100] < EB_RATIO_SPREAD_FRAC * s[len(s) // 2]

  def update_cylinder_deactivation(self, mode: int, gas_pressed: bool, v_ego: float,
                                   rpm: float, speed_kph: float) -> None:
    """Resolve the cylinder status one frame. Separated from update() so the thresholds
    run under test without a bus."""
    coupled = self._track_coupling(rpm, speed_kph)
    if mode == CYL_MODE_DEACTIVATED:
      self.cyl_state = CylinderState.deactivated
      self.cyl_entry_progress = 1.0
      self.eb_confirm_frames = 0
    elif CYL_MODE_ENTRY_FIRST <= mode <= CYL_MODE_ENTRY_LAST:
      self.cyl_state = CylinderState.entry
      self.cyl_entry_progress = (mode - CYL_MODE_ENTRY_FIRST) / (CYL_MODE_ENTRY_LAST - CYL_MODE_ENTRY_FIRST)
      self.eb_confirm_frames = 0
    elif mode == CYL_MODE_NORMAL or mode in CYL_MODE_EXITS:
      # The byte has no exit ramp, so progress drops straight back to zero.
      self.cyl_entry_progress = 0.0
      overrun = not gas_pressed and v_ego > EB_MIN_V_EGO and coupled
      self.eb_confirm_frames = self.eb_confirm_frames + 1 if overrun else 0
      self.cyl_state = (CylinderState.engineBraking
                              if self.eb_confirm_frames >= EB_CONFIRM_FRAMES
                              else CylinderState.normal)
    else:
      # A mode with no mapping was never observed on the wire, so it carries no
      # information: hold the resolved state rather than flicker back to normal on a
      # stray value from an engine this decode was not validated on.
      pass

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp_cam = can_parsers[Bus.cam]

    # CAM_TRAFFIC_SIGNS.SPEED_SIGN_UNIT encodes display state and unit: 1 = mph, 2 = km/h,
    # 0 = none. The limit is camera-detected or the car's own map fallback; both are used.
    # Plausibility: 90 covers the highest US posting (85 mph); the 7-bit all-ones 127 is a
    # sentinel.
    sign = cp_cam.vl["CAM_TRAFFIC_SIGNS"]
    speed_sign = sign["SPEED_SIGN"]
    if sign["SPEED_SIGN_UNIT"] == 1 and 0 < speed_sign <= 90:
      ret_sp.speedLimit = float(speed_sign) * CV.MPH_TO_MS
    elif sign["SPEED_SIGN_UNIT"] == 2 and 0 < speed_sign < 127:
      ret_sp.speedLimit = float(speed_sign) * CV.KPH_TO_MS
    else:
      ret_sp.speedLimit = 0.0

    # Runs last so gasPressed and vEgo are this frame's values.
    ed = can_parsers[Bus.pt].vl["ENGINE_DATA"]
    self.update_cylinder_deactivation(int(can_parsers[Bus.pt].vl["MORE_GAS"]["CYL_DEACT_STATE"]),
                                      ret.gasPressed, ret.vEgo,
                                      float(ed["RPM"]), float(ed["SPEED"]))
