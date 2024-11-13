package org.firstinspires.ftc.teamcode.control.movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive extends LinearOpMode {

    private boolean showTelemetry = true;
    private boolean fieldCentric = true;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    private IMU imu;

    public Drive(boolean showTelemetryData, boolean useFieldCentric) {
        showTelemetry = showTelemetryData;
        fieldCentric = useFieldCentric;
    }

    private void initializeDriveMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    // resetDriveMotors(leftFront, rightFront, leftRear, rightRear);
    private void resetDriveMotors(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.resetDeviceConfigurationForOpMode();
        }
    }

    @Override
    public void runOpMode() {
        initializeDriveMotors();

        waitForStart();
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double leftFrontPower = (rotY + rotX + rx) / denominator;
            double rightFrontPower = (rotY - rotX - rx) / denominator;
            double leftRearPower = (rotY - rotX + rx) / denominator;
            double rightRearPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftRear.setPower(leftRearPower);
            rightRear.setPower(rightRearPower);

            if (showTelemetry) {
                telemetry.addLine("=== DRIVE OUTPUT ===");
                telemetry.addData("Drive.leftFront", leftFront.getPower());
                telemetry.addData("Drive.rightFront", rightFront.getPower());
                telemetry.addData("Drive.leftRear", leftRear.getPower());
                telemetry.addData("Drive.rightRear", rightRear.getPower());
                telemetry.addLine();
                telemetry.update();
            }
        }
    }
}
