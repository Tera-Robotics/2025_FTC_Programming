package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "First Auto", group = "Autonomous")

public class Auto extends LinearOpMode {
private DcMotor leftFront, leftBack, rightFront,rightBack;

    static final double COUNTS_PER_MOTOR_REV = 448;
    static final double WHELL_DIAMETER_CM = 7.5;

    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHELL_DIAMETER_CM*Math.PI);

    static final double TURN_SPEED = 0.5;

    static final double DRIVE_SPEED = 0.6;

    public void DriveEncoder (double speed, double leftFrontCm, double leftBackCm, double rightFrontCm,double rightBackCm) {
        int newleftFrontTarget;
        int newleftBackTarget;
        int newrightBackTarget;
        int newrightFrontTarget;

        if (opModeIsActive()) {

            newleftFrontTarget = leftFront.getCurrentPosition() + (int) (leftFrontCm * COUNTS_PER_CM);
            newleftBackTarget = leftBack.getCurrentPosition() + (int) (leftBackCm * COUNTS_PER_CM);
            newrightFrontTarget = rightFront.getCurrentPosition() + (int) (rightFrontCm * COUNTS_PER_CM);
            newrightBackTarget = rightBack.getCurrentPosition() + (int) (rightBackCm * COUNTS_PER_CM);

            leftFront.setTargetPosition(newleftFrontTarget);
            leftBack.setTargetPosition(newleftBackTarget);
            rightFront.setTargetPosition(newrightFrontTarget);
            rightBack.setTargetPosition(newrightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

        }
    }


    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);



        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoder();

        waitForStart();

        DriveEncoder(DRIVE_SPEED,10,10,10,10);
        DriveEncoder(TURN_SPEED,10,10,10,10);

        }

    private void stopMotors() {
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
    }

    private void resetEncoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}