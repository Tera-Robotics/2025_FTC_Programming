package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;




public class MecanumDrive {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private IMU imu;


    public void init(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);



        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(revHubOrientation));

    }

    public void drive(double forward, double strafe, double rotate) {

        double leftFrontPower = forward - strafe - rotate;
        double leftBackPower = forward + strafe - rotate;
        double rightFrontPower = forward + strafe + rotate;
        double rightBackPower = forward - strafe + rotate;

        double maxPower = 0.4;
        double maxSpeed = 0.4;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFront.setPower(maxSpeed * (leftFrontPower / maxPower));
        leftBack.setPower(maxSpeed * (leftBackPower / maxPower));
        rightFront.setPower(maxSpeed * (rightFrontPower / maxPower));
        rightBack.setPower(maxSpeed * (rightBackPower / maxPower));



           /* x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            turn = - gamepad1.right_stick_x;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            leftFrontPower = ((y + x + turn) / denominator);
            leftBackPower = ((y - x + turn) / denominator);
            rightFrontPower  = ((y - x - turn) / denominator);
            rightBackPower = ((y + x - turn) / denominator);

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);*/

    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(-newForward,newStrafe, rotate);

    }
}





