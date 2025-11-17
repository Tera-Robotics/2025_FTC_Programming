package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDriveFieldRelative;
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

    private enum Marchas {
        ALTA,

        MEDIA,

        BAIXA,
    }

    RobotState robotState = RobotState.DEFAULT;

    Marchas marchaAtual = Marchas.ALTA;
    MecanumDriveFieldRelative drive = new MecanumDriveFieldRelative();
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

        boolean leftStick = gamepad1.leftStickButtonWasPressed();

        switch (marchaAtual){
            case ALTA:
                drive.setMaxSpeed(0.9);
                if (leftStick) marchaAtual = Marchas.MEDIA;
                break;
            case MEDIA:
                drive.setMaxSpeed(0.6);
                if (leftStick) marchaAtual = Marchas.BAIXA;
                break;
            case BAIXA:
                drive.setMaxSpeed(0.3);
                if (leftStick) marchaAtual = Marchas.ALTA;
                break;
        }

        telemetry.addData("Marcha Atual", marchaAtual);
        telemetry.update();

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


    forward = -gamepad1.left_stick_y;
    strafe = gamepad1.left_stick_x*1.3;
    rotate = -gamepad1.right_stick_x;

    drive.driveFieldRelative(forward,strafe,rotate);


    }
}
