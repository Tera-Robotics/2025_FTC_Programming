package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "First Auto", group = "Autonomous")

public class Auto extends LinearOpMode {
private DcMotor leftFront, leftBack, rightFront,rightBack;

private IMU imu;


    private double headingError = 0;
    private double  targetHeading = 0;
    private double  turnSpeed     = 1;
    private int     leftFrontTarget    = 0;

    private int     leftBackTarget    = 0;

    private int     rightFrontTarget    = 0;
    private int     rightBackTarget   = 0;
    static final double COUNTS_PER_MOTOR_REV = 318;
    static final double WHELL_DIAMETER_CM = 9.6;

    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (WHELL_DIAMETER_CM*Math.PI);

    static final double TURN_SPEED = 1;

    static final int DRIVE_SPEED = 1;

    static final double STRAFE_SPEED = 0.6;

    static final double     HEADING_THRESHOLD       = 2;

    static final double     P_TURN_GAIN            = 0.01;
    static final double     P_DRIVE_GAIN           = 0.003;

    Intake intake = null;
    Shooter shooter = null;



    @Override
    public void runOpMode() {
        
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        // Primeiro Ciclo de shooting
        shooter.moveToDefault();
        sleep(1000);
        shooter.moveToShoot();
        sleep(1000);

        //Segundo ciclo de shooting
        shooter.stop();
        driveStraight(0.6,70,0,0.6);
        turnToHeading(TURN_SPEED,40,1);
        strafe(STRAFE_SPEED,80,0.6);
        intake.starCollectBall();
        shooter.moveToDefault();
        driveStraight(0.3,-70,0,0.3);
        shooter.stop();
        sleep(800);
        intake.stopCollectBall();
        driveStraight(0.2,30,0,0.2);
        strafe(0.6,-120,1);
        turnToHeading(TURN_SPEED,1,0.8);
        driveStraight(1,-30,0,0.8);
        shooter.moveToDefault();
        sleep(1000);
        shooter.moveToShoot();
        sleep(1000);
        shooter.stop();

        //Terceiro ciclo de shooting
        driveStraight(0.6,70,0,0.6);
        turnToHeading(TURN_SPEED,41,1);
        strafe(STRAFE_SPEED,165,0.7);
        intake.starCollectBall();
        shooter.moveToDefault();
        driveStraight(0.2,-80,0,0.2);
        shooter.stop();
        sleep(800);
        intake.stopCollectBall();
        driveStraight(0.2,20,0,0.2);
        strafe(0.6,-180,1);
        turnToHeading(TURN_SPEED,2,1);
        driveStraight(1,-30,0,0.8);
        shooter.moveToDefault();
        sleep(1000);
        shooter.moveToShoot();
        sleep(1000);
        shooter.stop();
        strafe(0.6,180,0.8);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }


    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading,
                              double SPEED_STRAIGHT) {
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_CM);
            leftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBack.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBack.getCurrentPosition() + moveCounts;

            leftFront.setTargetPosition(leftFrontTarget);
            leftBack.setTargetPosition(leftBackTarget);
            rightFront.setTargetPosition(rightFrontTarget);
            rightBack.setTargetPosition(rightBackTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0,0,SPEED_STRAIGHT);


            while (opModeIsActive() &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(DRIVE_SPEED, 0,turnSpeed,SPEED_STRAIGHT);

                sendTelemetry(true);
            }

            moveRobot(0, 0,0,0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading,double TURN_SPEED) {

        while (opModeIsActive()) {

            headingError = heading - getHeading();

            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Agora sim está correto
            if (Math.abs(headingError) <= HEADING_THRESHOLD)
                break;

            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, 0, turnSpeed,TURN_SPEED);

            sendTelemetry(false);
        }

        moveRobot(0, 0, 0,0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed,0,0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0,0,0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void moveRobot(double forward, double strafe, double rotate, double speed) {

        double leftFrontPower  = forward + strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double leftBackPower   = forward - strafe + rotate;
        double rightBackPower  = forward + strafe - rotate;

        double maxPower = 1;
        double maxSpeed = speed;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFront.setPower(maxSpeed * (leftFrontPower / maxPower));
        leftBack.setPower(maxSpeed * (leftBackPower / maxPower));
        rightFront.setPower(maxSpeed * (rightFrontPower / maxPower));
        rightBack.setPower(maxSpeed * (rightBackPower / maxPower));
    }

    public void strafe(double speed, double distanceCm,double STRAFE) {
        if (!opModeIsActive()) return;

        int moveCounts = (int)(distanceCm * COUNTS_PER_CM);

        // Para strafe à direita: motores mecanum se movem assim:
        // LF: +, LB: -, RF: -, RB: +
        leftFrontTarget  = leftFront.getCurrentPosition()  + moveCounts;
        leftBackTarget   = leftBack.getCurrentPosition()   - moveCounts;
        rightFrontTarget = rightFront.getCurrentPosition() - moveCounts;
        rightBackTarget  = rightBack.getCurrentPosition()  + moveCounts;

        leftFront.setTargetPosition(leftFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        moveRobot(0, speed, 0,STRAFE);  // strafe positivo = para direita

        while (opModeIsActive() &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            sendTelemetry(true);
        }

        moveRobot(0, 0, 0,0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d", leftBackTarget, rightBackTarget,leftFrontTarget,rightFrontTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}