/*
 * Measured physical parameters of the rover drive system.
 *
 * Single source of truth for hardware measurements shared by the
 * drive_control, wheel_speed, and motor_control nodes. Update here when
 * the hardware changes.
 */

#pragma once

// Distance between left and right wheel centers (center to center), meters
#define TRACK_WIDTH_METERS 0.86

// Drive wheel radius, meters
#define WHEEL_RADIUS_METERS 0.1
