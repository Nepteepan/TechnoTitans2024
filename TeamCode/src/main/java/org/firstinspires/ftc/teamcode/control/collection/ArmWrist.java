package org.firstinspires.ftc.teamcode.control.collection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmWrist extends LinearOpMode {

    private boolean showTelemetry = true;
    private final Gamepad controller;
    private final double gearBoxRatio;
    public DcMotor arm;
    public Servo wrist;

    // Constructor
    public ArmWrist(boolean showTelemetryData, double armGearRatio, Gamepad armController) {
        showTelemetry = showTelemetryData;
        controller = armController;
        gearBoxRatio = armGearRatio;
        initializeArmAndWrist();
    }

    final double TICKS_PER_DEGREE = 19.7924893140647; // exact fraction is (194481/9826)
    final double DEGREES_PER_REVOLUTION = 360.0;
    final double MOTOR_TICKS_PER_REVOLUTION = TICKS_PER_DEGREE * DEGREES_PER_REVOLUTION;

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * TICKS_PER_DEGREE;

    final double ARM_STOW_POSITION = 230 * TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * TICKS_PER_DEGREE;

    // Wrist positional values
    final double WRIST_STOW_POSITION = 0.8333;
    final double WRIST_COLLECTION_POSITION = 0.455;

    // Degrees that the triggers can adjust the arm position by
    final double FUDGE_FACTOR = 15 * TICKS_PER_DEGREE;

    // Arm position variables
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor; // exact fraction is (194481/9826)

    private void initializeArmAndWrist() {
        wrist = hardwareMap.get(Servo.class, "wrist");
        arm = hardwareMap.get(DcMotor.class, "armMotor");
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setStartingPosition() {
        arm.setTargetPosition((int) ARM_STOW_POSITION);
        sleep(500);
        wrist.setPosition(WRIST_COLLECTION_POSITION);
    }

    private void rotateArmToPosition(double targetDegrees, double speed) {
        double effectiveTicksPerRevolution = MOTOR_TICKS_PER_REVOLUTION * gearBoxRatio;
        int targetTicks = (int) (targetDegrees / DEGREES_PER_REVOLUTION * effectiveTicksPerRevolution);
        // Set the target position in ticks
        arm.setTargetPosition(targetTicks);

        // Set the speed
        arm.setPower(speed);

        // Stop the motor once the target position is reached
        arm.setPower(0);

        // Reset to use the encoder for any future commands
        // arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        setStartingPosition();

        waitForStart();
        while (opModeIsActive()) {

            arm.setPower(-controller.left_trigger / 1.2);
            arm.setPower(controller.right_trigger / 1.2);

            if (controller.y) {
                wrist.setPosition(WRIST_STOW_POSITION);
            }

            if (showTelemetry) {
                telemetry.addLine("=== ARM OUTPUT ===");
                telemetry.addData("armMotor.power", arm.getPower());
                telemetry.addData("armMotor.currentPosition", arm.getCurrentPosition());
                telemetry.addData("armMotor.targetPosition", arm.getTargetPosition());
                telemetry.addData("armMotor.gearBoxRatio", gearBoxRatio);
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
