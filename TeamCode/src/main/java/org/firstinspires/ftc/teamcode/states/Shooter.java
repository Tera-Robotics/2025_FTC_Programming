package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Shooter {


    private static final String SHOOTER_NAME = "shooter";

    private static double SHOOTER_DEFAULT = 0.4;
    private static double SHOOTTING = 0.6;

    private static final int RPM_TO_SHOOT = 3000;

    private static final int RPM_DEFAULT = 600;

    private static final int CPR = 28;
    private static final int MAX_RPM = 6000;

    private static final int MAX_TICKS_PER_SECOND = (CPR / MAX_RPM) * 60;

    public DcMotorEx shooterMotor;

    private int setPoint = 0;

    public Shooter (HardwareMap hardwareMap){

        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

    }

    public void shootDefault() {
        setVelocityRPM(RPM_DEFAULT);
    }

    public void shootForGoal (){
        setVelocityRPM(RPM_TO_SHOOT);
    }

    public void setVelocityRPM(int rpm) {
        setPoint = rpm;
       double tickesPerSecond = ((double) rpm/MAX_RPM) * MAX_TICKS_PER_SECOND;
       shooterMotor.setVelocity(tickesPerSecond);
    }

    public int getVelocityInRPM(){
        return(int)((shooterMotor.getVelocity()/MAX_TICKS_PER_SECOND));
    }

}
