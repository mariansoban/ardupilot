/*
  CADDX mount using serial protocol backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_CADDX_ENABLED

#include "AP_Mount_Backend_Serial.h"
#include <AP_Math/quaternion.h>

class AP_Mount_CADDX : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };

    // has_roll_control - returns true if this mount can control its roll
    bool has_roll_control() const override { return roll_range_valid(); };

    // has_pitch_control - returns true if this mount can control its tilt
    bool has_pitch_control() const override { return pitch_range_valid(); };

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // get the body-frame pan angle last sent to the gimbal, for display (e.g. OSD)
    // returns false if nothing was sent recently or the lock mode was zero (gimbal ignores angles)
    bool get_pan_bf_rad(float& pan_rad) override;

private:

    // lock mode enum
    enum class LockMode {
        PITCH_LOCK = (1<<0),
        ROLL_LOCK  = (1<<1),
        YAW_LOCK   = (1<<2),
    };

    // CADDX can only send angles
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_ONLY;
    };

    // send_target_angles
    void send_target_angles(const MountAngleTarget& angle_target_rad) override;

    uint32_t _first_send_ms;    // system time the first attitude packet was sent to the gimbal
    uint32_t _last_send_ms;     // system time the last attitude packet was sent to the gimbal
    uint8_t _last_sent_mode;    // lock mode bits of the last attitude packet
    float _last_sent_yaw_bf_rad;    // body-frame yaw angle of the last attitude packet
};
#endif // HAL_MOUNT_CADDX_ENABLED
