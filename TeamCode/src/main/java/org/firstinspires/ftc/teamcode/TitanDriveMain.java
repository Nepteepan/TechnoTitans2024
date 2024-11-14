package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.control.collection.ArmWrist;
import org.firstinspires.ftc.teamcode.control.collection.Intake;
import org.firstinspires.ftc.teamcode.control.movement.Drive;

@TeleOp(name="Titan: Drive and Controls", group="TeleOp")
public class TitanDriveMain extends LinearOpMode {

    private boolean fieldCentric = false;
    private final boolean showTelemetryData = true;
    public Gamepad driver = gamepad1;
    public Gamepad operator = gamepad2;

    private final Drive driveController = new Drive(showTelemetryData, fieldCentric, driver);
    private final ArmWrist armWristController = new ArmWrist(showTelemetryData, 10, operator);
    private final Intake intakeController = new Intake(showTelemetryData, gamepad2);
    private boolean optionsPressed = false;

    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {

            // Toggle Field Centric mode
            if (driver.options && !optionsPressed) {
                fieldCentric = !fieldCentric;
                driveController.setFieldCentric(fieldCentric);
            }
            optionsPressed = driver.options;
        }
    }
}
