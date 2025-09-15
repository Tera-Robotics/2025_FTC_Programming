package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import javax.net.ssl.HandshakeCompletedEvent;


@TeleOp
public class MecanumFieldRelative extends OpMode {

    MecanumDrive drive = new MecanumDrive();

    double forward, strafe, rotate;

    private DcMotorEx testeMotor;

    @Override
    public void init () {

        testeMotor = hardwareMap.get(DcMotorEx.class, "testeMotor");
        testeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    drive.init(hardwareMap);

    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            testeMotor.setPower(1);
        }
        else if (gamepad1.b) {
            testeMotor.setPower(-1);
        }
        else {
            testeMotor.setPower(0);
        }


    forward = -gamepad1.left_stick_y;
    strafe = +gamepad1.left_stick_x;
    rotate = gamepad1.right_stick_x;

    drive.driveFieldRelative(forward,strafe,rotate);
    }
}
