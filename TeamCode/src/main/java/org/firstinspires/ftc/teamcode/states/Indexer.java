package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Indexer {

    private static final String SERVO_INDEXER_NAME = "servoIndexer";

    private final double INDEXER_180_DEGREES = 0.5;
    private final double INDEXER_0_DEGREES = 0;

    private Servo servoIndexer;

    //inicializamos como construtor para definir o que é o objeto

    public Indexer (HardwareMap hardwareMap) {

        servoIndexer = hardwareMap.get(Servo.class,SERVO_INDEXER_NAME);
    }


    public void activateIndexer() {
        servoIndexer.setPosition(INDEXER_180_DEGREES);
    }

    public void desactivate (){

        servoIndexer.setPosition(INDEXER_0_DEGREES);

    }


}
