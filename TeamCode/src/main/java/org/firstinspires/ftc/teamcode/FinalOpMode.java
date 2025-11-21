package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDriveFieldRelative;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;


@TeleOp
public class FinalOpMode extends OpMode {

    private enum RobotState{

        DEFAULT,

        PREPARAR,

        EXPELIR,

        DESLIGA,

    }

    private enum Marchas {
        ALTA,

        MEDIA,

        BAIXA,
    }

    RobotState robotState = RobotState.DEFAULT;
    RobotState previousRobotState = robotState;

    Marchas marchaAtual = Marchas.ALTA;
    MecanumDriveFieldRelative drive = new MecanumDriveFieldRelative();
    Intake intake = null;
    Shooter shooter = null;
    double forward, strafe, rotate;



    @Override
    public void init () {


    drive.init(hardwareMap);
    intake = new Intake(hardwareMap);
   shooter = new Shooter(hardwareMap);

    }

    boolean previousXButtonValue = false;
    boolean previousAButtonValue = false;
    boolean previousYButtonValue = false;

    boolean isPreviousBButtonValue = false;



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

        boolean buttonX = gamepad1.x;
        boolean buttonY = gamepad1.y;
        boolean buttonA = gamepad1.a;
        boolean buttonB = gamepad1.b;
        boolean triggerRight = gamepad1.right_bumper;
        boolean triggerLeft = gamepad1.left_bumper;



     switch (robotState){
         case DEFAULT:
                intake.starCollectBall();
             if(buttonX && !previousXButtonValue) {previousRobotState = robotState;
                 robotState =robotState.PREPARAR;}
             break;
         case PREPARAR:
             intake.defaultCollect();
             if(buttonX && !previousXButtonValue){ previousRobotState = robotState;
             robotState = robotState.DESLIGA;}
             break;
         case DESLIGA:
             intake.stopCollectBall();
         case EXPELIR:
         if(buttonA && !previousAButtonValue) {
             intake.expelBall();
             break;
         }
     }

        if(buttonB && !isPreviousBButtonValue) {
           robotState = previousRobotState;
        }
        if (triggerRight){
            shooter.moveToShoot();
        }
        else if (triggerLeft){
            shooter.moveToCharging();
        }
        else {
            shooter.stop();
        }



    forward = gamepad1.left_stick_y;
    strafe = -gamepad1.left_stick_x*1.1;
    rotate = -gamepad1.right_stick_x;

    drive.drive(forward,strafe,rotate);

        telemetry.addData("Marcha atual", marchaAtual);
        telemetry.addData("Estado atual do robo", robotState);
        telemetry.addData("Estado anterior do robo", previousRobotState);
       // telemetry.addData("Encoders", shooter.encoderValues());
        telemetry.update();



    }
}
