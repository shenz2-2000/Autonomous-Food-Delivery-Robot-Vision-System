void ChassisSKD::velocity_decompose(float vx, float vy, float w) {

    // FR, +vx, -vy, +w
    // FL, +vx, +vy, +w, since the motor is installed in the opposite direction
    // BL, -vx, +vy, +w, since the motor is installed in the opposite direction
    // BR, -vx, -vy, +w

    // install_mode_ = 1
    // w_to_v_ratio_ = (wheel_base_ + wheel_tread_) / 2.0f / 360.0f * 3.14159f;
    //               = (420.0f + 372.0f) / 2.0f / 360.0f * 3.14159f;
    // v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference_)
    //                              = (360.0f / 478.0f )

    target_velocity[FR] = install_mode_ * (+vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;

    target_velocity[FL] = install_mode_ * (+vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;

    target_velocity[BL] = install_mode_ * (-vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;

    target_velocity[BR] = install_mode_ * (-vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
}

