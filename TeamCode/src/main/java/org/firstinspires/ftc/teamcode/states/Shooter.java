package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private static final String SHOOTER_LEFT = "shooterLeft";
    private static final String SHOOTER_RIGHT = "shooterRight";

    private static double POWER_UP = -1;
    private static double POWER_DOWN = 1;

    public DcMotorEx leftMotorShoot, rightMotorShoot;

    public Shooter(HardwareMap hardwareMap) {

        leftMotorShoot = hardwareMap.get(DcMotorEx.class, SHOOTER_LEFT);
        rightMotorShoot = hardwareMap.get(DcMotorEx.class, SHOOTER_RIGHT);

        leftMotorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotorShoot.setDirection(DcMotor.Direction.REVERSE);
        rightMotorShoot.setDirection(DcMotor.Direction.FORWARD);
    }

    /** SUBIR por 1s */
    public void moveToCharging() {

        leftMotorShoot.setPower(POWER_UP);
        rightMotorShoot.setPower(POWER_UP);
    }

    /** DESCER por 1s */
    public void moveToShoot() {

        leftMotorShoot.setPower(POWER_DOWN);
        rightMotorShoot.setPower(POWER_DOWN);

    }

    /** SUBIR por 1s */
    public void moveToDefault() {

        leftMotorShoot.setPower(POWER_UP);
        rightMotorShoot.setPower(POWER_UP);
    }

    public void stop() {
        leftMotorShoot.setPower(0);
        rightMotorShoot.setPower(0);
    }

}
