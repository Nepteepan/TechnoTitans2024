package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="org.firstinspires.ftc.teamcode.TitanDriveTest", group="TeleOp")
public class TitanDriveTest extends LinearOpMode {
    private DcMotor fL, fR, bL, bR;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    /*private Gamepad driverController;
    public DcMotor leftDrive = null; //the left drivetrain motor
    public DcMotor rightDrive = null; //the right drivetrain motor*/
    public DcMotor armMotor = null; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo testArm = null; //the wrist servo








    /* Declare OpMode members. */





    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)


    /*make sure the
    arm is reset to collapsed inside the robot before you start the program.*/


    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 15 * ARM_TICKS_PER_DEGREE;


    //Variables to store the speed
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /*  tthe wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    private IMU imu;

    @Override
    public void runOpMode() {

        // Initialize Variables and Drive Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        armMotor = hardwareMap.get(DcMotor.class, "testArm"); //the arm motor
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.

        // The equivalent button is start on Xbox-style controllers.


        //arm code
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake = hardwareMap.get(CRServo.class, "intake");
        testArm = hardwareMap.get(Servo.class, "wrist");
        intake.setPower(INTAKE_OFF);
        testArm.setPosition(WRIST_FOLDED_IN);


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
                if (gamepad1.right_bumper) {
                    /* This is the intaking/collecting arm position */
                    armPosition = ARM_COLLECT;
                    testArm.setPosition(WRIST_FOLDED_OUT);
                    intake.setPower(INTAKE_COLLECT);
                } else if (gamepad1.left_bumper) {
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                    armPosition = ARM_CLEAR_BARRIER;
                } else if (gamepad1.y) {
                    /* This is the correct height to score the sample in the LOW BASKET */
                    armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                } else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                    armPosition = ARM_COLLAPSED_INTO_ROBOT;
                    intake.setPower(INTAKE_OFF);
                    testArm.setPosition(WRIST_FOLDED_IN);
                } else if (gamepad1.dpad_right) {
                    /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                    armPosition = ARM_SCORE_SPECIMEN;
                    testArm.setPosition(WRIST_FOLDED_IN);
                } else if (gamepad1.dpad_up) {
                    /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                    armPosition = ARM_ATTACH_HANGING_HOOK;
                    intake.setPower(INTAKE_OFF);
                    testArm.setPosition(WRIST_FOLDED_IN);
                } else if (gamepad1.dpad_down) {
                    /* this moves the arm down to lift the robot up once it has been hooked */
                    armPosition = ARM_WINCH_ROBOT;
                    intake.setPower(INTAKE_OFF);
                    testArm.setPosition(WRIST_FOLDED_IN);
                }
                imu.resetYaw();
            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
                armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();











































        }
    }

}

