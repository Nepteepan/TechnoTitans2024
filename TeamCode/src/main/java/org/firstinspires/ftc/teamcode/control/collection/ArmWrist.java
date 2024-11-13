package org.firstinspires.ftc.teamcode.control.collection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmWrist extends LinearOpMode {

    private boolean showTelemetry = true;
    public DcMotor arm;
    public Servo wrist;

    // Constructor
    public ArmWrist(boolean showTelemetryData) {
        showTelemetry = showTelemetryData;
    }

    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; // exact fraction is (194481/9826)
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;

    // Wrist positional values
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    // Degrees that the triggers can adjust the arm position by
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    // Arm position variables
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;//exact fraction is (194481/9826)

    private void initializeAmrAndWrist() {
        arm = hardwareMap.get(DcMotor.class, "armMotor");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(WRIST_FOLDED_IN);
    }

    @Override
    public void runOpMode() {
        initializeAmrAndWrist();

        waitForStart();
        while (opModeIsActive()) {

            if (showTelemetry) {
                telemetry.addLine("=== ARM OUTPUT ===");
                telemetry.addData("armMotor.power", arm.getPower());
                telemetry.addData("armMotor.currentPosition", arm.getCurrentPosition());
                telemetry.addData("armMotor.targetPosition", arm.getTargetPosition());
                telemetry.addData("armMotor.mode", arm.getMode());
                telemetry.addLine();
                telemetry.addLine("=== WRIST OUTPUT ===");
                telemetry.addData("wrist.position", wrist.getPosition());
                telemetry.addData("wrist.direction", wrist.getDirection());
                telemetry.update();
            }
        }
    }
}
