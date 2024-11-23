package org.firstinspires.ftc.teamcode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeleOp.TitanDriveAndArm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="RedSide")
public class RedSide extends LinearOpMode {

    public Servo leftClaw;
    public Servo rightClaw;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor elbow;
    private IMU imu;
    private int targetArmPosition = 0;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 1.0;

    public DcMotor armMotor; //the arm motor
    public CRServo intake = null; //the active intake servo
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
    public void driveMotorInalazation(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void openClaw() {
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        //_clawStatus = TitanDriveAndArm.clawStatus.OPEN;
    }

    private void closeClaw() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
        //_clawStatus = TitanDriveAndArm.clawStatus.CLOSED;
    }
    public void moveNumberOfInches(double numberOfInches, double power) {
        double totalRevs = 42 * numberOfInches;
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setTargetPosition((int) (totalRevs));
        rightRear.setTargetPosition((int) (totalRevs));
        leftFront.setTargetPosition((int) (totalRevs));
        rightFront.setTargetPosition((int) (totalRevs));
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void runOpMode()  {
        driveMotorInalazation();
        armStartupSequence();

        waitForStart();
        while (opModeIsActive()){
            armMotor.setTargetPosition(200);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            sleep(500);
            moveNumberOfInches(68, 1);
            sleep(500);
            openClaw();
            sleep(2000);
            closeClaw();
            sleep(500);
            moveNumberOfInches(-10, 1);
            sleep(500);
            armMotor.setTargetPosition(0);
            requestOpModeStop();
        }
    }
}
