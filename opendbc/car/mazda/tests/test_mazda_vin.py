from opendbc.car import structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.mazda.fingerprints import FW_VERSIONS
from opendbc.car.mazda.values import CAR, match_fw_to_car_fuzzy
from opendbc.car.vin import VIN_UNKNOWN

Ecu = structs.CarParams.Ecu


def make_vin(wmi: str, chassis_code: str, year_code: str) -> str:
  # positions 6-9, 11-17 are arbitrary: the decoder reads the WMI, the model
  # line (positions 4-5) and the model year code (position 10)
  return wmi + chassis_code + '2L50' + year_code + '0' + '000042'


class TestMazdaVinMatch:
  # Real VINs from public listings, one per model year code, with the trim from the listing.
  # None marks a model line with no supported platform.
  REAL_VINS = [
    # KF 2017-21 -> MAZDA_CX5
    ('JM3KFBDL8H0189068', CAR.MAZDA_CX5),        # 2017 Grand Touring
    ('JM3KFBCM8J0391425', CAR.MAZDA_CX5),        # 2018 Touring
    ('JM3KFBDY8K0524140', CAR.MAZDA_CX5),        # 2019 Grand Touring Reserve
    ('JM3KFBBMXL0721103', CAR.MAZDA_CX5),        # 2020 Sport
    ('JM3KFBEY9M0334140', CAR.MAZDA_CX5),        # 2021 Signature
    # KF 2022-25 -> MAZDA_CX5_2022
    ('JM3KFBEM7N0646584', CAR.MAZDA_CX5_2022),   # 2022 Premium Plus
    ('JM3KFBXY2P0142737', CAR.MAZDA_CX5_2022),   # 2023 2.5 Turbo Signature
    ('JM3KFBCL4R0506329', CAR.MAZDA_CX5_2022),   # 2024 Preferred
    ('JM3KFBAY8S0594547', CAR.MAZDA_CX5_2022),   # 2025 Carbon Turbo
    # TC 2016-20 -> MAZDA_CX9, 2021-23 -> MAZDA_CX9_2021
    ('JM3TCBDY1G0107351', CAR.MAZDA_CX9),        # 2016 Grand Touring
    ('JM3TCBDY2K0314968', CAR.MAZDA_CX9),        # 2019 Grand Touring
    ('JM3TCBBY1L0416377', CAR.MAZDA_CX9),        # 2020 Sport
    ('JM3TCBEYXM0534974', CAR.MAZDA_CX9_2021),   # 2021 Signature
    ('JM3TCBAY3N0628864', CAR.MAZDA_CX9_2021),   # 2022 Touring Plus
    ('JM3TCBDY4P0655571', CAR.MAZDA_CX9_2021),   # 2023 Carbon Edition
    # GL 2017-21 -> MAZDA_6, BN 2017-18 -> MAZDA_3 (Salamanca builds use 3MZ)
    ('JM1GL1U58H1108261', CAR.MAZDA_6),          # 2017 Sport
    ('JM1GL1VM0J1336606', CAR.MAZDA_6),          # 2018 Touring
    ('JM1GL1TYXK1503013', CAR.MAZDA_6),          # 2019 Grand Touring
    ('JM1GL1TY7L1523723', CAR.MAZDA_6),          # 2020 Grand Touring
    ('JM1GL1VM4M1613049', CAR.MAZDA_6),          # 2021 Touring
    ('3MZBN1K71HM135634', CAR.MAZDA_3),          # 2017 Sport
    ('3MZBN1V34JM170702', CAR.MAZDA_3),          # 2018 Touring
    # Unsupported model lines stay unmatched on real VINs too
    ('JM1BPALL3N1522302', None),                 # 2022 Mazda 3 (BP)
    ('3MVDMBEM0LM104467', None),                 # 2020 CX-30 (DM)
  ]

  def test_real_listing_vins(self):
    for vin, expected in self.REAL_VINS:
      expected_platforms = {str(expected)} if expected is not None else set()
      assert match_fw_to_car_fuzzy({}, vin, FW_VERSIONS) == expected_platforms

  def test_wrong_wmi_does_not_match(self):
    assert match_fw_to_car_fuzzy({}, make_vin('JM6', 'TC', 'M'), FW_VERSIONS) == set()

  def test_unsupported_models_do_not_match(self):
    # BP (Mazda 3 2019+), DM (CX-30), KE (pre-2017 CX-5), VA (CX-50), and a CX-9
    # past the last supported model year
    for wmi, chassis_code, year_code in (('JM1', 'BP', 'K'), ('JM3', 'DM', 'N'),
                                         ('JM3', 'KE', 'H'), ('7MM', 'VA', 'P'),
                                         ('JM3', 'TC', 'T')):
      assert match_fw_to_car_fuzzy({}, make_vin(wmi, chassis_code, year_code), FW_VERSIONS) == set()

  def test_invalid_vin_does_not_match(self):
    assert match_fw_to_car_fuzzy({}, 'JM3KF2L50NI000042', FW_VERSIONS) == set()  # banned character
    assert match_fw_to_car_fuzzy({}, 'JM3KF', FW_VERSIONS) == set()  # too short

  def test_vin_unknown_does_not_match(self):
    # all zeros passes the charset; '00' matches no model line
    assert match_fw_to_car_fuzzy({}, VIN_UNKNOWN, FW_VERSIONS) == set()


def _car_fw(ecu, address, version: bytes) -> structs.CarParams.CarFw:
  fw = structs.CarParams.CarFw()
  fw.ecu = ecu
  fw.address = address
  fw.subAddress = 0
  fw.fwVersion = version
  fw.brand = 'mazda'
  fw.bus = 0
  return fw


# Versions that exist in no database, standing in for dealer-updated ECUs
UNKNOWN_ENGINE_FW = b'ZZ99-9999X-Z-99' + b'\x00' * 9
UNKNOWN_ABS_FW = b'ZZ99-8888X-Z-99' + b'\x00' * 9


class TestMatchFwToCarVinFallback:
  """The EPS-swap scenario through the real matcher: the donor EPS breaks every
  exact match, unknown engine and ABS versions break generic fuzzy matching, and
  the VIN names the chassis."""

  def _swapped_mazda6_fw(self) -> list:
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    stock_trans = FW_VERSIONS[CAR.MAZDA_6][(Ecu.transmission, 0x7e1, None)][0]
    return [
      _car_fw(Ecu.eps, 0x730, donor_eps),
      _car_fw(Ecu.engine, 0x7e0, UNKNOWN_ENGINE_FW),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, stock_trans),
    ]

  def test_swapped_eps_matches_the_chassis_by_vin(self):
    vin = make_vin('JM1', 'GL', 'L')
    exact_match, matches = match_fw_to_car(self._swapped_mazda6_fw(), vin)
    assert not exact_match
    assert matches == {str(CAR.MAZDA_6)}

  def test_swapped_eps_without_a_vin_stays_unmatched(self):
    _, matches = match_fw_to_car(self._swapped_mazda6_fw(), VIN_UNKNOWN)
    assert matches == set()

  def test_stock_fw_set_still_exact_matches(self):
    car_fw = [_car_fw(ecu, addr, versions[0])
              for (ecu, addr, _), versions in FW_VERSIONS[CAR.MAZDA_CX9_2021].items()]
    vin = make_vin('JM3', 'TC', 'M')
    exact_match, matches = match_fw_to_car(car_fw, vin)
    assert exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}
