package org.firstinspires.ftc.teamcode;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

// NEED THIS
import org.firstinspires.ftc.teamcode.common.subsystems.lift.Shoulder;
import org.firstinspires.ftc.teamcode.common.subsystems.lift.Wrist;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Titan: ARM TESTING", group="TeleOp")
public class TitanArm extends LinearOpMode {
    private final double falloff = 2;
    private final double steepness = 2;
    //
    // private Wrist wristMover;
    // private Shoulder shoulderMover;
    //

    private DcMotorEx fL, fR, bL, bR;
    private Gamepad lifterController = gamepad1;

    DcMotorEx shoulder;
    DcMotorEx wrist;
    IMU imu;

    public void initializeArmComponents() {
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        wrist = hardwareMap.get(DcMotorEx.class, "wrist");

    }

    public void moveWrist(Integer position) {
        // wristMover.moveToPosition(position);
    }

    public void moveArm(Integer position) {
        // shoulderMover.moveToPosition(position);
    }

    @Override
    public void runOpMode() {
        initializeArmComponents();

        waitForStart();
        while (opModeIsActive()) {
            // Drive Code
            // End Drive Code

            //Lifter Code
            if (gamepad2.y) {
                // moveArm(750);
                // moveWrist(328);
                // shoulderMover.raiseArm();
                shoulder.setMotorEnable();
                shoulder.setTargetPosition(2);
                // wristMover.raiseWrist();
            }

            if (gamepad2.a) {
                // theGaber.openSmallClaw();
                // theGaber.openLargeClaw();
                // shoulderMover.lowerArm();
                // wristMover.lowerWrist();
            }

            if (gamepad2.x) {
                // theGaber.closeSmallClaw();
                // theGaber.closeLargeClaw();
                // sleep(750);
                // shoulderMover.setDrivePosition();
                // wristMover.lowerWrist();
            }

            if (gamepad2.b) {
                // theGaber.openLargeClaw();
                // sleep(500);
                // theGaber.openSmallClaw();
                // sleep(100);
            }
            // End Drone Code

            // The Gaber Code
            if (lifterController.left_bumper) {
                // theGaber.openSmallClaw();
            }
            if (lifterController.right_bumper) {
                // theGaber.closeSmallClaw();
            }
            if (lifterController.left_trigger > 0) {
                // theGaber.openLargeClaw();
            }
            if (lifterController.right_trigger > 0) {
                // theGaber.closeLargeClaw();
            }
            // End Gaber Code
        }
    }
}
