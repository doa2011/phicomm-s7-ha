# S7 measurement outbox and HA archive

This revision changes measurement delivery from a delayed HA sensor-state snapshot to a device-originated event with an explicit acknowledgement.

## Runtime flow

1. The S7 MCU reports stable weight and the ESPHome component starts one CS1258 scan for that weighing session.
2. The ESP8266 waits for the HA profile sync (up to 12 seconds from the start of the weight session). If HA is not reachable in time, the measurement is marked Unassigned rather than assigned to the wrong person.
3. After a valid scan, ESPHome calculates body composition and stores a compact CSV record in one of two ESP8266 flash-backed outbox slots.
4. ESPHome emits `esphome.s7_measurement` while the HA Native API state-subscribing client is connected. It retries pending slots every 15 seconds and after API reconnection.
5. HA stores the record in the matching `input_text.s7_memberN_record` helper (or the unassigned helper), then calls `esphome.phicomm_s7_ack_measurement` with the record ID.
6. ESPHome clears only the slot matching that ACK. Duplicate deliveries are safe: the same record ID is stored again and acknowledged.

The outbox has **two slots**, not an unlimited queue. If both slots are occupied and another measurement is completed before an ACK arrives, the firmware logs an error and does not overwrite either older record. This is intentional fail-safe behavior; do not describe it as unlimited buffering.

## CSV field order

The event's `record` value is a comma-separated string with these 20 fields:

| Index (zero-based) | Field |
|---:|---|
| 0 | Random measurement ID |
| 1 | Member ID: 0 unassigned, 1–3 members |
| 2 | Weight (kg) |
| 3 | Composite impedance (ohm) |
| 4 | BMI |
| 5 | Body fat (%) |
| 6 | Water (%) |
| 7 | Fat-free mass (kg) |
| 8 | BMR (kcal) |
| 9 | Muscle (%) |
| 10 | Bone mass (kg) |
| 11–19 | Nine CS1258 channels: LR leg, RL leg, LR hand, right leg, left leg, LH-LL, LH-RL, RH-LL, RH-RL |

The complete CSV is retained in the HA `input_text` helper. The template sensors expose the main metrics with `state_class: measurement`, allowing Recorder/long-term statistics to retain their time series.

## Required Home Assistant setup

1. Install the updated ESPHome YAML and local `s7_mcu` component from this branch, then compile and flash the ESP8266.
2. Replace the existing HA package with `homeassistant/packages/phicomm_s7.yaml`; keep `homeassistant: packages: !include_dir_named packages` in `configuration.yaml`.
3. In HA, open **Settings → Devices & services → ESPHome → Phicomm S7 → Configure** and enable **Allow the device to perform Home Assistant actions**. This permission is required for ESPHome to emit HA events and call the profile-sync script.
4. Check HA configuration and restart Home Assistant.
5. Verify that these ESPHome actions exist after the updated firmware is online:
   - `esphome.phicomm_s7_set_active_member`
   - `esphome.phicomm_s7_ack_measurement`
6. Select a member in `input_select.s7_active_person` before stepping onto the scale. If HA cannot synchronize the profile before the 12-second timeout, the record is deliberately filed as Unassigned.

## Main HA entities

- `input_text.s7_member1_record`, `input_text.s7_member2_record`, `input_text.s7_member3_record`: latest complete CSV record for each member.
- `input_text.s7_unassigned_record`: latest unassigned record.
- `sensor.s7_member_1_weight`, `sensor.s7_member_1_bmi`, `sensor.s7_member_1_body_fat`, etc.: per-member metric sensors. Equivalent entities are created for members 2 and 3.

The input_text helpers contain the latest full record, while Recorder history of the metric template sensors provides the trend history. The helpers themselves are not a multi-row database.
