#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
// Physical TJA button, DBC start bit 11 (byte 1, bit 3). Observed on a CTS-equipped gen1
// Mazda; trims without the button hold it low for the life of a drive.
#define MAZDA_TJA_BUTTON_BIT 11U
// sunnypilot safety param: the TJA button is the MADS lateral switch
#define MAZDA_PARAM_SP_TJA_BUTTON 1U
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_2 0x362U
#define MAZDA_RADAR_TRACK_3 0x363U
#define MAZDA_RADAR_TRACK_4 0x364U
#define MAZDA_RADAR_TRACK_5 0x365U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U
// Select the steer-to-zero EPS envelope from the firmware-derived interface flag.
#define MAZDA_PARAM_STEER_TO_ZERO_EPS 2U
// The same EPS hardware on firmware that keeps the 45 kph floor: the same envelope.
#define MAZDA_PARAM_LEGACY_FW_EPS 4U

// Keep SET/RES intent fresh until PEDALS reports engagement.
#define MAZDA_ENGAGE_BTN_WINDOW 10U
// A wheel cancel turns MRCC main off for real, and PEDALS trails the press: keep the context
// on the 50 Hz PEDALS clock for carstate's CANCEL_CONTEXT_T so the main-off edge lands under
// braking, where a brake-only bit dropout would otherwise be held.
#define MAZDA_CANCEL_CONTEXT_FRAMES 25U

static bool mazda_longitudinal = false;
// Declared by the driver: the TJA button owns lateral and MRCC no longer drives the main edge.
static bool mazda_tja_button = false;
static bool mazda_steer_to_zero_eps = false;
static bool mazda_legacy_fw_eps = false;
static uint32_t mazda_engage_btn_frames = 0U;
static uint32_t mazda_cancel_context_frames = 0U;

// Pin replaced-radar traffic to captured stock patterns where possible.

// Each radar generation sends its own static capture; the controller picks the dialect (mazdacan.py)
static bool mazda_radar_static_msg_valid(const CANPacket_t *msg) {
  bool capture_2022 = (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
                      (msg->data[2] == 0xc0U) && (msg->data[3] == 0x00U) &&
                      (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                      (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
  bool capture_g46l = (msg->data[0] == 0x00U) && (msg->data[1] == 0x98U) &&
                      (msg->data[2] == 0x40U) && (msg->data[3] == 0x00U) &&
                      (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                      (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
  return capture_2022 || capture_g46l;
}

static bool mazda_empty_radar_track_msg_valid(const CANPacket_t *msg) {
  bool valid = false;

  if ((msg->addr == MAZDA_RADAR_TRACK_1) || (msg->addr == MAZDA_RADAR_TRACK_2) ||
      (msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0xfeU) &&
            (msg->data[4] == 0x1fU);

    if ((msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x00U);
    } else {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    }
  } else if ((msg->addr == MAZDA_RADAR_TRACK_5) || (msg->addr == MAZDA_RADAR_TRACK_6)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0x7fU) &&
            (msg->data[4] == 0xfbU) && (msg->data[5] == 0xffU) &&
            (msg->data[6] == 0x3fU) && ((msg->data[7] & 0xf0U) == 0xc0U);
  } else {
  }

  return valid;
}

// The occupied 5/6 replay: byte-exact stock captures (mazdacan.py RADAR_CLUTTER_CYCLE, 2022
// CX-5 city drive) cycled by the controller so the camera keeps deciding auto high beams
// under the radar takeover. The trailing empty-template frames of the cycle pass through the
// empty pattern below; only the captured frames need membership here.
#define MAZDA_CLUTTER_TRACK_FRAMES 80

static const uint64_t mazda_clutter_track_5[MAZDA_CLUTTER_TRACK_FRAMES] = {
  0x1d21b433b83c27c1ULL, 0x1bd1c833203e27c2ULL, 0x1a91da32804127c3ULL, 0x1941f231e04227c4ULL,
  0x18120631384427c5ULL, 0x16c22030984727c6ULL, 0x15823c2ff84927c7ULL, 0x14325a2f504b27c8ULL,
  0x12f27e2ea85127c9ULL, 0x11b2a82e005327caULL, 0x1062da2d585527cbULL, 0x0f031c2cb05727ccULL,
  0x0db364ac105927cdULL, 0x0c73baab685927ceULL, 0x0b3420aac05a27cfULL, 0x09f496aa205a27c0ULL,
  0xfff7fe29785b27c1ULL, 0x837066a8d05b1bc2ULL, 0xfff7fe28285c3fc3ULL, 0x80e06c27805f1bc4ULL,
  0x7f806c26d8611bc5ULL, 0x7e306e2628631bc6ULL, 0x7ce06c2580631bc7ULL, 0xfff7fe24d8623fc8ULL,
  0x7a40702430621bc9ULL, 0x78f0722388641bcaULL, 0x77a07422e0651bcbULL, 0x7650762238671bccULL,
  0x7500782190681bcdULL, 0x73b07a20e8681bceULL, 0x72607820406a1bcfULL, 0x7100761f906d1bc0ULL,
  0x6fb0761ee86f1bc1ULL, 0x6e50761e406f1bc2ULL, 0x6d00761d90701bc3ULL, 0x6ba0781ce8731bc4ULL,
  0x6a507c1c38771bc5ULL, 0x68f07a1b907a1bc6ULL, 0x67a07c9ae07d1bc7ULL, 0x6610801a18831bc8ULL,
  0x64c0841968881bc9ULL, 0x63808418d08b1bcaULL, 0x62408418288d1bcbULL, 0x60f0881780921bccULL,
  0x5f908c16d0961bcdULL, 0x5e20901610991bceULL, 0x5cd09015689d1bcfULL, 0x5b709414c0a21bc0ULL,
  0x5a10941410a61bc1ULL, 0x58c0941360ac1bc2ULL, 0x57609412c0b11bc3ULL, 0x5610961210b71bc4ULL,
  0x54a0981158bf1bc5ULL, 0x53409a10b0c71bc6ULL, 0x51c0a00ff0d11bc7ULL, 0x5080a40f48da1bc8ULL,
  0x4f20a80ea0e41bc9ULL, 0x4dd0ae0df0ef1bcaULL, 0x4c70b40d40fb1bcbULL, 0x4ae0ba0c810b1bccULL,
  0x4990bc0bd11a1bcdULL, 0x4810c00b112d1bceULL, 0x46d0c00a71401bcfULL, 0x4580c409c9591bc0ULL,
  0x4420c83b584a1881ULL, 0x42c0cc7ffbff1bc2ULL, 0x4160ce39f0481883ULL, 0x4040d43940491884ULL,
  0x3f00d8b888491885ULL, 0x3dc0deb7d84a5886ULL, 0x3c60e2b720491887ULL, 0x3b10e8b668495888ULL,
  0x3980ec35b8481889ULL, 0x3800f2350848588aULL, 0x3670f6345849188bULL, 0x3500fa33a84b588cULL,
  0x3390fc32f050188dULL, 0x3220feb23850188eULL, 0x30b106318852188fULL, 0x2f410e30d8531880ULL,
};

static const uint64_t mazda_clutter_track_6[MAZDA_CLUTTER_TRACK_FRAMES] = {
  0x5320a47ffbff37c1ULL, 0x51f0a87ffbff37c2ULL, 0x50b0ae7ffbff37c3ULL, 0x4f70b27ffbff37c4ULL,
  0x4e20b47ffbff37c5ULL, 0x4ce0b67ffbff37c6ULL, 0x4b90be7ffbff37c7ULL, 0x4a50c47ffbff37c8ULL,
  0x4900c87ffbff37c9ULL, 0x47b0cc7ffbff37caULL, 0x4670d47ffbff37cbULL, 0x4520da7ffbff37ccULL,
  0x43d0e07ffbff37cdULL, 0x4280e47ffbff37ceULL, 0x4130e87ffbff37cfULL, 0x3ff0e47ffbff37c0ULL,
  0x3eb0e67ffbff37c1ULL, 0x3d60e63528433602ULL, 0x3c10f03480443603ULL, 0x3ac0f833d8463604ULL,
  0x3970fe3330473605ULL, 0x3821003288473606ULL, 0x36d10031e0453607ULL, 0x3581023138443608ULL,
  0x3421063090443609ULL, 0x32e10c2fe843360aULL, 0x3181102f4043360bULL, 0x3031162e9843360cULL,
  0x2ee11c2df042360dULL, 0x2d81222d4042360eULL, 0x2c31282c9841360fULL, 0x2ae1302bf0413600ULL,
  0x2981382b40413601ULL, 0x2831402a98413602ULL, 0x26e14a29f0413603ULL, 0x25815429483f3604ULL,
  0x24316028983d3605ULL, 0x22d16c27e83d3606ULL, 0x21717a27383d3607ULL, 0x1ff18e26703f3608ULL,
  0x1e91a025c0433609ULL, 0x1d51b2252048360aULL, 0x1c01c8248049360bULL, 0x1ab1dc23d84b360cULL,
  0x1961f423304e360dULL, 0x17f21622704f360eULL, 0x16a23621c852360fULL, 0x15425a2118583600ULL,
  0x13d2862068593601ULL, 0x1282b41fb85b3602ULL, 0x1132e61f185d3603ULL, 0x0fe3221e705f3604ULL,
  0x0e63741db0613605ULL, 0xfff7fe1d08643e06ULL, 0xfff7fe1c48673e07ULL, 0xfff7fe1ba8683e08ULL,
  0xfff7fe1b006a7e09ULL, 0xfff7fe1a586e3e0aULL, 0xfff7fe19a8713e0bULL, 0xfff7fe18e0743e0cULL,
  0xfff7fe1838773e0dULL, 0xfff7fe17707b3e0eULL, 0xfff7fe16d07f3e0fULL, 0xfff7fe1628833e00ULL,
  0x86606e1578881601ULL, 0x84607c14d08b1602ULL, 0x82a08c14208f1603ULL, 0x80f09a1368941604ULL,
  0x7f60a812b8991605ULL, 0x7e60ac1208a01606ULL, 0x7d40b29150a31607ULL, 0x61f08290a0ab1208ULL,
  0xfff7fe0ff0b13e09ULL, 0x7870c60f40bc160aULL, 0x7700c80e88c3160bULL, 0x7590c60dd8cd160cULL,
  0x7420c68d20da160dULL, 0x72c0c68c68f3160eULL, 0x7140cc0bb105160fULL, 0x6fd0d00b01151600ULL,
};

static bool mazda_clutter_frame_match(uint64_t capture, const CANPacket_t *msg) {
  // byte 7's low nibble carries the live counter; every other bit must be exact
  uint64_t dat = 0U;
  for (int i = 0; i < 8; i++) {
    dat = (dat << 8) | msg->data[i];
  }
  return ((dat ^ capture) & 0xFFFFFFFFFFFFFFF0ULL) == 0U;
}

static bool mazda_clutter_track_msg_valid(const CANPacket_t *msg) {
  bool valid = false;
  const uint64_t *table = NULL;
  if (msg->addr == MAZDA_RADAR_TRACK_5) {
    table = mazda_clutter_track_5;
  } else if (msg->addr == MAZDA_RADAR_TRACK_6) {
    table = mazda_clutter_track_6;
  }
  if (table != NULL) {
    for (int i = 0; i < MAZDA_CLUTTER_TRACK_FRAMES; i++) {
      valid = valid || mazda_clutter_frame_match(table[i], msg);
    }
  }
  return valid;
}

static bool mazda_synthetic_lead_radar_track_msg_valid(const CANPacket_t *msg) {
  // Permit only the distance and relative-velocity fields in the occupied-track template.
  return (msg->addr == MAZDA_RADAR_TRACK_4) &&
         ((msg->data[1] & 0x0fU) == 0x0eU) && (msg->data[2] == 0x00U) &&
         ((msg->data[4] & 0x1fU) == 0x1cU) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && ((msg->data[7] & 0xf0U) == 0x00U);
}

static bool mazda_radar_track_msg_valid(const CANPacket_t *msg) {
  // Occupied tracks represent perception and remain valid while controls are disengaged.
  return mazda_empty_radar_track_msg_valid(msg) ||
         mazda_synthetic_lead_radar_track_msg_valid(msg) ||
         mazda_clutter_track_msg_valid(msg);
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_longitudinal) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      // With the TJA button owning lateral, MRCC no longer drives the MADS main edge: its
      // falling edge would exit the panda's lateral while the software's MADS stays on.
      if (!mazda_tja_button) {
        acc_main_on = GET_BIT(msg, 17U);
      }
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_tja_button) {
      // The physical TJA button is the MADS lateral switch, so lateral no longer follows MRCC.
      mads_button_press = GET_BIT(msg, MAZDA_TJA_BUTTON_BIT) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      // A physical cancel press always exits controls, and explains the main-off that follows.
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
        mazda_cancel_context_frames = MAZDA_CANCEL_CONTEXT_FRAMES;
      }
      // Record SET/RES intent for the engagement qualifier below.
      if (GET_BIT(msg, 2U) || GET_BIT(msg, 4U) || GET_BIT(msg, 5U)) {
        mazda_engage_btn_frames = MAZDA_ENGAGE_BTN_WINDOW;
      } else if (mazda_engage_btn_frames > 0U) {
        mazda_engage_btn_frames -= 1U;
      } else {
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // Derive cruise state from PEDALS after radar teardown. Ignore transient brake-only
        // samples where both cruise bits are low.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;
        bool brake_free = !brake && !brake_pressed_prev;

        // Main mirrors carstate's cruise_available: it follows arming, and a both-low sample is
        // held under braking unless a wheel cancel explains it. Without the cancel path, main
        // toggled at a stop with the brake held never falls, the next press has no rising edge,
        // and MADS runs into 200 rejected frames (route 000001c9--0b2a64a214 seg 0).
        if (mazda_tja_button) {
          // the button is the lateral switch; MRCC is cruise only
        } else if (acc_armed) {
          // Main follows PEDALS arming from the first frame; the radar takeover gates cruise
          // (controls_allowed below), never main.
          acc_main_on = true;
        } else if (brake_free || (mazda_cancel_context_frames > 0U)) {
          acc_main_on = false;
        } else {
        }
        if (mazda_cancel_context_frames > 0U) {
          mazda_cancel_context_frames -= 1U;
        }

        if (acc_armed || cruise_engaged_prev || brake_free) {
          // Require recent SET/RES intent on the engaged edge; ACC_ACTIVE alone may acknowledge
          // synthetic traffic rather than a driver request.
          if (cruise_engaged && !cruise_engaged_prev && (mazda_engage_btn_frames > 0U)) {
            controls_allowed = true;
          }
          if (!cruise_engaged) {
            controls_allowed = false;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }
      brake_pressed = brake;
    }
  }
}

static bool mazda_is_lka_addr(int addr) {
  return (((unsigned int)addr == MAZDA_LKAS) || ((unsigned int)addr == MAZDA_LKAS_HUD));
}

// The camera owns the LKAS addresses whenever openpilot is not steering. Lateral is its own
// axis under MADS (controls_allowed_lateral); with MADS off it follows cruise. Cruise alone must
// not claim them: under stock long that silenced the camera's own TJA/CTS with MADS off while the
// dash showed nothing (route 00000018--5655da2c1c seg 15).
static bool mazda_openpilot_controlling(void) {
  return controls_allowed_lateral || (controls_allowed && !m_mads_state.system_enabled);
}

// The one CRZ_BTNS frame openpilot may put on the camera bus: the TJA button pressed over the
// wheel's idle pattern (00 09 ff Cx 00 00 00 00, Cx = MODE_X_INV, MODE_Y_INV and the counter),
// no other button. It presses the camera's own TJA/CTS off whenever the camera is armed, so the
// two lane-centering systems never run at once and the camera never takes the wheel behind a
// MADS-off press. Accepted in every state: the frame only reaches the camera and can only
// toggle its lane centering, which the wheel button does anyway; the camera's torque is still
// vetoed whenever openpilot steers (docs/zoompilot/mazda-lateral.md, "The camera's own TJA/CTS
// state").
static bool mazda_cam_tja_press_msg_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x09U) && (msg->data[2] == 0xffU) &&
         ((msg->data[3] & 0xc3U) == 0xc0U) && (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  // Stock pre-2022 EPS envelope.
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // The measured EPS envelope, selected by either firmware bit: the EPS's 12-count hardware
  // slew, with max_rate_down equal to the controller retreat rate so driver-limit winddown
  // frames remain valid.
  const TorqueSteeringLimits MAZDA_STEER_TO_ZERO_EPS_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 12,
    .max_rate_down = 12,
    .max_rt_delta = 384,
    .driver_torque_multiplier = 15,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO.ACCEL_CMD uses 0.001 m/s2 raw units after removing the offset.
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_replacement_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    const TorqueSteeringLimits *limits = &MAZDA_STEERING_LIMITS;
    if (mazda_steer_to_zero_eps || mazda_legacy_fw_eps) {
      limits = &MAZDA_STEER_TO_ZERO_EPS_STEERING_LIMITS;
    } else {
      // upstream's pre-2022 envelope, no longer selected by the interface
    }
    if (steer_torque_cmd_checks(desired_torque, -1, *limits)) {
      tx = false;
    }
  }

  // Run after steering checks, which reset rate-limit state while disengaged.
  if (main_bus && mazda_is_lka_addr(msg->addr) && !mazda_openpilot_controlling()) {
    tx = false;
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    // Allow byte-exact stock standby patterns with the raw 8190 command sentinel.
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         ((msg->data[4] & 0xfbU) == 0xc0U) &&
                         ((msg->data[5] & 0x7fU) == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0xffU - ((msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3] +
                                                     msg->data[4] + msg->data[5] + msg->data[6]) & 0xffU)) & 0xffU));

    // Assemble the 13-bit ACCEL_CMD unsigned for MISRA 10.1, then remove its 4096 offset.
    uint32_t accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) | ((uint32_t)msg->data[3] << 3) | ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)accel_raw - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }

    // ACC_ACTIVE requires prior controls permission established from the physical SET edge.
    bool acc_active = GET_BIT(msg, 33U);
    if (!controls_allowed && acc_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    bool cruise_active = GET_BIT(msg, 3U);
    if (!controls_allowed && cruise_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_RADAR_STATIC)) {
    if (!mazda_radar_static_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr >= MAZDA_RADAR_TRACK_1) && (msg->addr <= MAZDA_RADAR_TRACK_6)) {
    if (!mazda_radar_track_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && main_bus && (msg->addr == MAZDA_RADAR_UDS)) {
    // Allow tester-present and default/programming session control only.
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    // Permit resume only while controlling and cancel only while not controlling.
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
    // The TJA button is never pressed on the car's side: it would toggle MADS through the
    // rx hook and arm MRCC in the body.
    if (GET_BIT(msg, MAZDA_TJA_BUTTON_BIT)) {
      tx = false;
    }
  }

  if ((msg->bus == (unsigned char)MAZDA_CAM) && (msg->addr == MAZDA_CRZ_BTNS)) {
    // The camera-side press exists only to switch the camera's TJA/CTS off: byte-exact, any state.
    if (!mazda_cam_tja_press_msg_valid(msg)) {
      tx = false;
    }
  }

  return tx;
}

static bool mazda_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == MAZDA_CAM) {
    if (mazda_is_lka_addr(addr)) {
      block_msg = mazda_openpilot_controlling();
    }
  }

  return block_msg;
}

static safety_config mazda_init(uint16_t param) {
  mazda_engage_btn_frames = 0U;
  mazda_cancel_context_frames = 0U;

  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    // The camera press: no relay check, so the wheel's own 0x09d keeps forwarding to the camera.
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
  };

// Replaced-radar addresses omit relay checks because the radar remains live during boot and
// hand-back. carstate enforces single ownership instead.
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // CRZ_CTRL intentionally disappears after radar teardown.
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  mazda_steer_to_zero_eps = GET_FLAG(param, MAZDA_PARAM_STEER_TO_ZERO_EPS);
  mazda_legacy_fw_eps = GET_FLAG(param, MAZDA_PARAM_LEGACY_FW_EPS);
  mazda_tja_button = GET_FLAG(current_safety_param_sp, MAZDA_PARAM_SP_TJA_BUTTON);
  acc_main_on = false;

  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
};
