package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name = "OpModeMecanum")

public class OpModeMecanum extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart(); // espera o start no DS

        while (opModeIsActive()) {
            if (gamepad1.a) {
                leftFront.setPower(1);
            } else {
                leftFront.setPower(0);
            }
        }
    }


}
