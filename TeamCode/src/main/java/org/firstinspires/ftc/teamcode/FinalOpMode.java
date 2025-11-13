package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;


@TeleOp
public class FinalOpMode extends OpMode {

    private enum RobotState{

        DEFAULT,

        PREPARAR,

        EXPELIR,

        ATIRAR,

    }

    RobotState robotState = RobotState.DEFAULT;
    MecanumDrive drive = new MecanumDrive();
    Intake intake = null;
    Shooter shooter = null;
    double forward, strafe, rotate;

    private DcMotorEx testeMotor;

    @Override
    public void init () {

       // testeMotor = hardwareMap.get(DcMotorEx.class, "testeMotor");
        //testeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    drive.init(hardwareMap);
    //intake = new Intake(hardwareMap);
   // shooter = new Shooter(hardwareMap);

    }

    @Override
    public void loop() {

        /*boolean buttonX = gamepad1.x;
        boolean buttonY = gamepad1.y;
        boolean buttonA = gamepad1.a;

     switch (robotState){
         case DEFAULT:
             intake.starCollectBall();
             shooter.shootDefault();

             if (buttonX) robotState = robotState.PREPARAR;
             break;
         case PREPARAR:
             intake.stopCollectBall();
             shooter.shootForGoal();


     }

        if (gamepad1.a) {
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
