package org.firstinspires.ftc.teamcode.Pathing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.states.Intake;
import org.firstinspires.ftc.teamcode.states.Shooter;

@Autonomous(name = "Blue 1", group = "Autonomous")
public class BlueAllience1 extends LinearOpMode {

    static final double TURN_SPEED = 1;

    static final double STRAFE_SPEED = 0.6;

    Intake intake = null;
    Shooter shooter = null;

    Auto auto = null;

    @Override
    public void runOpMode() {

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        auto = new Auto(hardwareMap,this);

        waitForStart();

            // Primeiro Ciclo de shooting
            shooter.moveToDefault();
            sleep(1000);
            shooter.moveToShoot();
            sleep(1000);
            shooter.stop();

            //Segundo ciclo de shooting
            auto.driveStraight(0.6, 70, 0, 0.6);
            shooter.moveToDefault();
            sleep(600);
            shooter.moveToShoot();
            sleep(600);
            shooter.stop();
            auto.turnToHeading(TURN_SPEED, 40, 1);
            auto.strafe(STRAFE_SPEED, 87, 0.6);
            intake.starCollectBall();
            shooter.moveToDefault();
            auto.driveStraight(0.25, -70, 0, 0.25);
            shooter.stop();
            sleep(800);
            auto.driveStraight(0.2, 30, 0, 0.2);
            auto.strafe(0.6, -120, 1);
            auto.turnToHeading(TURN_SPEED, 1, 0.8);
            intake.stopCollectBall();
            auto.driveStraight(1, -30, 0, 0.8);
            shooter.moveToDefault();
            sleep(1000);
            shooter.moveToShoot();
            sleep(1000);
            shooter.stop();

            //Terceiro ciclo de shooting
            auto.driveStraight(0.6, 70, 0, 0.6);
            auto.turnToHeading(TURN_SPEED, 41, 1);
            auto.strafe(STRAFE_SPEED, 180, 0.7);
            intake.starCollectBall();
            shooter.moveToDefault();
            auto.driveStraight(0.2, -80, 0, 0.2);
            shooter.stop();
            sleep(800);
            auto.driveStraight(0.2, 20, 0, 0.2);
            auto.strafe(0.6, -180, 1);
            auto.turnToHeading(TURN_SPEED, 2, 1);
            intake.stopCollectBall();
            auto.driveStraight(1, -40, 0, 0.8);
            shooter.moveToDefault();
            sleep(1000);
            shooter.moveToShoot();
            sleep(1000);
            shooter.stop();
            auto.strafe(0.6, 180, 0.8);


            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // Pause to display last telemetry message.


        }
    }
