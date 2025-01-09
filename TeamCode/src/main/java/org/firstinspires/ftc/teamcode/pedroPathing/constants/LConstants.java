package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .002;
        TwoWheelConstants.strafeTicksToInches = .002;
        TwoWheelConstants.forwardY = -3.623;
        TwoWheelConstants.strafeX = 3.770;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "Front Right";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "Back Right";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.
                LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




