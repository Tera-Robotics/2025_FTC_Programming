package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class MecanumFieldRelativeTeste extends OpMode {

    MecanumDrive drive = new MecanumDrive();

    double forward, strafe, rotate;

    //private DcMotorEx testeMotor;

    @Override
    public void init () {

        //testeMotor = hardwareMap.get(DcMotorEx.class, "testeMotor");
        //testeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    drive.init(hardwareMap);

    }

    @Override
    public void loop() {

        /*if (gamepad1.a) {
            testeMotor.setPower(1);
        }
        else if (gamepad1.b) {
            testeMotor.setPower(-1);
        }
        else {
            testeMotor.setPower(0);
        }*/


    forward = gamepad1.left_stick_y;
    strafe = -gamepad1.left_stick_x*1.1;
    rotate = -gamepad1.right_stick_x;

    drive.driveFieldRelative(forward,strafe,rotate);
    }
}
