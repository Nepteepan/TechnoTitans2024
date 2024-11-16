package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private IMU imu;
    private int targetArmPosition = 0;
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 1.0;
    public DcMotor armMotor; //the arm motor
    public CRServo intake = null; //the active intake servo
    private void armStartupSequence() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(INTAKE_OFF);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // armMotor.setTargetPosition((int) ARM_CLEAR_BARRIER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveMotorInalazation(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
    }
    public void moveForwardInches(double numberOfInches, double power) {
        double totalRevs = 35 * numberOfInches;
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
            armMotor.setTargetPosition(40);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(1);
            sleep(500);
            moveForwardInches(10, 1);
            intake.setPower(INTAKE_DEPOSIT);
        }
    }
}
