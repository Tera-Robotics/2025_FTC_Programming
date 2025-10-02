package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;




@TeleOp
public class Testenovatos extends OpMode {

    public void init () {
        //Initialize hardware
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // Wait for the game to start (driver presses PLAY)

        // Move forward at half power for 2 seconds
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        // Stop motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void loop () {

    }
}

