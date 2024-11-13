package org.firstinspires.ftc.teamcode.control.collection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class Intake extends LinearOpMode {

    private boolean showTelemetry = true;
    public CRServo intake;

    // Speed & Direction
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    // Constructor
    public Intake(boolean showTelemetryData) {
        showTelemetry = showTelemetryData;
    }

    private void initializeIntakeServo() {
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(INTAKE_OFF);
    }

    @Override
    public void runOpMode() {
        initializeIntakeServo();

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            } else {
                intake.setPower(INTAKE_OFF);
            }
        }

        if (showTelemetry) {
            telemetry.addLine("=== INTAKE OUTPUT ===");
            telemetry.addData("intake.power", intake.getPower());
            telemetry.addData("intake.controller", intake.getController());
            telemetry.addData("intake.direction", intake.getDirection());
            telemetry.addLine();
            telemetry.update();
        }
    }
}
