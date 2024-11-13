package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Titan: Drive with Arm", group="TeleOp")
public class TitanDriveAndArm extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private IMU imu;


    public DcMotor armMotor; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo

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
    final double INTAKE_DEPOSIT = 1.0;

    /*  the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.455;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() {

        // Initialize Variables and Drive Motors
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");
        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize Variables for Arm, Wrist, and Intake
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int) ARM_CLEAR_BARRIER);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            // DRIVE CODE START
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

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
            // DRIVE CODE END

            // ARM WRIST INTAKE CODE START

            // INTAKE
            if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            }
            if (gamepad2.b) {
                intake.setPower(INTAKE_DEPOSIT);
            } else if (gamepad2.a) {
                intake.setPower(INTAKE_COLLECT);
            } else {
                intake.setPower(INTAKE_OFF);
            }
            // INTAKE - END
            if (gamepad2.y) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            if (gamepad2.right_bumper) {
                armMotor.setPower(0.2);
            }

            // if (armMotor.getCurrentPosition() < ARM_COLLECT) {
            armMotor.setPower((gamepad2.left_trigger * -1) / 1.2);
            // } else if (armMotor.getCurrentPosition() > ARM_CLEAR_BARRIER) {
            armMotor.setPower(gamepad2.right_trigger / 1.2);
            // ARM WRIST INTAKE CODE END
        }
    }
}