import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.mazda import mazdacan


class TestAlertCommand(unittest.TestCase):
  """CAM_LANEINFO re-send: the camera's departure-warn bits must reach the dash."""

  def _cam_laneinfo(self, packer, warn_ll, warn_rl):
    cam_msg = {s: 0 for s in ("LINE_VISIBLE", "LINE_NOT_VISIBLE", "LANE_LINES", "BIT1", "BIT2",
                              "BIT3", "NO_ERR_BIT", "S1", "S1_HBEAM", "LDW_WARN_LL", "LDW_WARN_RL")}
    cam_msg.update({"LANE_LINES": 2, "LDW_WARN_LL": warn_ll, "LDW_WARN_RL": warn_rl})
    _, dat, _ = mazdacan.create_alert_command(packer, cam_msg, steer_required=False)
    parser = CANParser("mazda_2017", [("CAM_LANEINFO", 0)], 0)
    parser.update([(0, [(0x440, dat, 0)])])
    return parser.vl["CAM_LANEINFO"]

  def test_ldw_warn_bits_pass_through(self):
    packer = CANPacker("mazda_2017")
    for warn_ll, warn_rl in ((1, 0), (0, 1), (0, 0)):
      vl = self._cam_laneinfo(packer, warn_ll, warn_rl)
      self.assertEqual(vl["LDW_WARN_LL"], warn_ll)
      self.assertEqual(vl["LDW_WARN_RL"], warn_rl)
