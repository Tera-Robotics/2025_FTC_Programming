package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp
public class PinpointTeste extends OpMode {

    GoBildaPinpointDriver pin;

    @Override
    public void init() {
        pin = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // API ANTIGA → só 2 offsets
        pin.setOffsets(
                150,  // parallel Y offset (mm)
                -67,  // perpendicular X offset (mm)
                DistanceUnit.MM
        );

        pin.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pin.resetPosAndIMU();
    }

    @Override
    public void loop() {
        pin.update();

        telemetry.addLine("=== POSIÇÃO (mm) ===");
        telemetry.addData("X", pin.getPosX(DistanceUnit.MM));
        telemetry.addData("Y", pin.getPosY(DistanceUnit.MM));

        telemetry.addLine();
        telemetry.addLine("=== VELOCIDADE (mm/s) ===");
        telemetry.addData("VX", pin.getVelX(DistanceUnit.MM));
        telemetry.addData("VY", pin.getVelY(DistanceUnit.MM));

        telemetry.addLine();
        telemetry.addLine("=== ÂNGULO ===");
        telemetry.addData("Heading (rad)", pin.getHeading(UnnormalizedAngleUnit.RADIANS));
        telemetry.addData("Heading Vel", pin.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));

        telemetry.update();
    }
}
