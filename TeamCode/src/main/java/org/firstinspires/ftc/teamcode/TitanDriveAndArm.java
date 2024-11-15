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

    private void initializeDriveMotors() {
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    private void armStartupSequence() {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition((int) ARM_CLEAR_BARRIER);
        // Wait .5 seconds
        sleep(500);
        // wrist.setPosition(WRIST_FOLDED_OUT);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {

        // Initialize Variables and Drive Motors
        initializeDriveMotors();

        // Initialize Variables for Arm, Wrist, and Intake
        armStartupSequence();

        waitForStart();
        while (opModeIsActive()) {
            // DRIVE CODE START
            // Y Axis: Forward & Backwards
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            // X Axis: Rotate Clockwise & Counter-Clockwise
            double x = gamepad1.left_stick_x;
            // Strafe Left and Right
            double rx = gamepad1.right_stick_x;

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
            // Intake Off
            if (gamepad2.x) {
                intake.setPower(INTAKE_OFF);
            }

            if (gamepad2.b) {
                // Intake Deposit
                intake.setPower(INTAKE_DEPOSIT);
            } else if (gamepad2.a) {
                // Intake Collect
                intake.setPower(INTAKE_COLLECT);
            } else {
                // Intake Off
                intake.setPower(INTAKE_OFF);
            }
            // INTAKE - END

            // WRIST START
            // Wrist into Collection position
            if (gamepad2.y) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            // Wrist into folded position
            if (gamepad2.x){
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            // WRIST END

            // Rotate Arm Out
            armMotor.setPower(-gamepad2.left_trigger * 1.5);
            // Rotate Arm In
            armMotor.setPower(gamepad2.right_trigger * 1.5);
            // ARM WRIST INTAKE CODE END
            telemetry.addLine(".:/ ARM TELEMETRY \\:.");
            telemetry.addData("Power", armMotor.getPower());
            telemetry.addData("arm Encoder", armMotor.getCurrentPosition());
            telemetry.addData("Intake", intake.getDirection());
            telemetry.update();
        }
    }
}
