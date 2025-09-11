package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import javax.net.ssl.HandshakeCompletedEvent;

public class MecanumFieldRelative extends OpMode {

    MecanumDrive drive = new MecanumDrive();

    double forward, strafe, rotate;

    @Override
    public void init () {

    drive.init(hardwareMap);

    }

    @Override
    public void loop() {
    forward = gamepad1.left_stick_y;
    strafe = gamepad1.left_stick_x;
    rotate = gamepad1.right_stick_x;

    drive.driveFieldRelative(forward,strafe,rotate);
    }
}
