#!/usr/bin/env python3
"""Print the extended /micro_lqr/debug schema used by this package."""
NAMES = {
    0: "position_m", 1: "velocity_mps", 2: "pitch_rad", 3: "pitch_rate_rad_s",
    4: "target_position_m", 5: "target_velocity_mps", 28: "velocity_from_position_filtered_legacy",
    29: "position_error_m", 30: "velocity_error_mps", 35: "velocity_motor_based_mps",
    36: "velocity_mismatch_mps", 44: "full_pitch_reference_rad", 45: "outer_velocity_filtered_mps",
    47: "attitude_error_rad", 73: "velocity_from_position_raw_mps",
    74: "velocity_from_position_filtered_mps", 75: "manual_trim_rad",
    76: "auto_trim_rad", 77: "total_trim_rad", 78: "full_pitch_reference_rad",
    79: "auto_trim_learning_flag", 80: "encoder_pair_updated_flag",
    81: "stiction_compensation_total_nm",
}
for index, name in sorted(NAMES.items()):
    print(f"{index:2d}: {name}")
