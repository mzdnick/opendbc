"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class MazdaSafetyFlagsSP:
  DEFAULT = 0
  # The physical TJA button is the MADS lateral switch; MRCC no longer drives the main edge.
  TJA_BUTTON = 1
  LKA_BUTTON = 2


class MazdaFlagsSP(IntFlag):
  # Fitted to some trims only and not predicted by the fingerprint, so the driver declares it.
  TJA_BUTTON = 1
  # The dash LKA button is the MADS lateral switch. Declare only where the dash button is
  # proven to drive LANE_LINES to zero; the whitelist liveness entry rides the same declaration.
  LKA_BUTTON = 2
