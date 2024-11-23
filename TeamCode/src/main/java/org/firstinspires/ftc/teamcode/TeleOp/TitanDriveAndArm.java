package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Titan: Bot Controls v3.0", group="TeleOp")
public class TitanDriveAndArm extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private IMU imu;
    private int targetArmPosition = 100;
    private int targetElbowPosition = 0;

    private int ARM_ELBOW_INCEMENTOR = 12;

    public DcMotor elbow;
    public DcMotor armMotor; // the arm motor
    public Servo leftClaw;
    public Servo rightClaw;

    private clawStatus _clawStatus = clawStatus.CLOSED;
    private enum clawStatus {
        OPEN,
        CLOSED
    }

    private void initializeDriveMotors() {
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void armStartupSequence() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setTargetPosition(targetArmPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(2.0);
        sleep(500);

        elbow = hardwareMap.get(DcMotor.class, "elbow");
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        sleep(500);
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        leftClaw.setDirection(Servo.Direction.FORWARD);
        leftClaw.setPosition(0);

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        rightClaw.setDirection(Servo.Direction.REVERSE);
        rightClaw.setPosition(0);
    }

    private void openClaw() {
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        _clawStatus = clawStatus.OPEN;
    }

    private void closeClaw() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
        _clawStatus = clawStatus.CLOSED;
    }

    private String getFrontDriveTelemetry(double leftPower, double rightPower) {
        leftPower = Math.round((leftPower) * 100);
        rightPower = Math.round((rightPower) * 100);
        return "Left Front / Right Front: " + leftPower + "% | " + rightPower + "%";
    }

    private String getRearDriveTelemetry(double leftPower, double rightPower) {
        leftPower = Math.round((leftPower) * 100);
        rightPower = Math.round((rightPower) * 100);
        return "Left Rear / Right Rear: " + leftPower + "% | " + rightPower + "%";
    }

    @Override
    public void runOpMode() {
        // Initialize Variables and Drive Motors
        initializeDriveMotors();

        // Initialize Variables for Arm, Elbow, and Intake
        armStartupSequence();

        waitForStart();
        while (opModeIsActive()) {
            // DRIVE CODE START
            // Y Axis: Forward & Backwards
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            // X Axis: Rotate Clockwise & Counter-Clockwise
            double x = -gamepad1.left_stick_x;
            // Strafe Left and Right
            double rx = -gamepad1.right_stick_x;

            // FIELD CENTRIC START
            imu.resetYaw();
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // FIELD CENTRIC END

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.25);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
            if (gamepad1.right_bumper){
                leftFront.setPower(frontLeftPower / 8);
                leftRear.setPower(backLeftPower / 8);
                rightFront.setPower(frontRightPower / 8);
                rightRear.setPower(backRightPower / 8);

            }
            // DRIVE CODE END

            // ARM ELBOW CLAW CODE START

            // CLAW - START
            // Close Claw
            if (gamepad2.x) {
                // intake.setPower(INTAKE_OFF);
                openClaw();
            }
            // Close Claw
            if (gamepad2.b) {
                closeClaw();
            }
            // CLAW - END

            // ELBOW - START
            if (gamepad2.dpad_down && targetElbowPosition >= 0) {
                targetElbowPosition -= ARM_ELBOW_INCEMENTOR;
            }
            if (gamepad2.dpad_up && targetElbowPosition <= 240) {
                targetElbowPosition += ARM_ELBOW_INCEMENTOR;
            }
            elbow.setTargetPosition(targetElbowPosition);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setPower(1);
            // ELBOW - END

            // ARM - START
            if (gamepad2.right_bumper && targetArmPosition <= 490) {
                targetArmPosition += ARM_ELBOW_INCEMENTOR;
            }
            if (gamepad2.left_bumper && targetArmPosition >= 0) {
                targetArmPosition -= ARM_ELBOW_INCEMENTOR;
            }

            armMotor.setTargetPosition(targetArmPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(2.0);
            // ARM - END

            // ARM ELBOW INTAKE CODE END

            // TELEMETRY OUTPUT
            telemetry.addLine("=== DRIVE TELEMETRY ===");
            telemetry.addLine(getFrontDriveTelemetry(leftFront.getPower(), rightFront.getPower()));
            telemetry.addLine(getRearDriveTelemetry(leftRear.getPower(), rightRear.getPower()));
            telemetry.addLine();
            telemetry.addLine("=== ARM TELEMETRY ===");
            telemetry.addData("Target Position", armMotor.getTargetPosition());
            telemetry.addData("Current Position", armMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine("=== ELBOW TELEMETRY ===");
            telemetry.addData("Target Position", elbow.getTargetPosition());
            telemetry.addData("Current Position", elbow.getTargetPosition());
            telemetry.addLine();
            telemetry.addLine("=== CLAW TELEMETRY ===");
            telemetry.addData("STATUS", _clawStatus);
            telemetry.update();
        }
    }
}
