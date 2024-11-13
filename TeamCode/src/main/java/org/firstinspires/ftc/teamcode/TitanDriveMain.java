package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.collection.ArmWrist;
import org.firstinspires.ftc.teamcode.control.collection.Intake;
import org.firstinspires.ftc.teamcode.control.movement.Drive;

@TeleOp(name="Titan: Drive and Controls", group="TeleOp")
public class TitanDriveMain extends LinearOpMode {

    private boolean fieldCentric = false;
    private final boolean useTelemetryData = true;

    private final Drive driveController = new Drive(useTelemetryData, fieldCentric, gamepad1);
    private final ArmWrist armWristController = new ArmWrist(useTelemetryData, 10, gamepad2);
    private final Intake intakeController = new Intake(useTelemetryData, gamepad2);

    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.options) {
                fieldCentric = !fieldCentric;
                driveController.setFieldCentric(fieldCentric);
            }
        }
    }
}
