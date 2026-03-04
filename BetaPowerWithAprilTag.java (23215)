package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "BETA POWER CODE WITH APRILTAG", group = "Concept")
public class BetaPowerWithAprilTag extends LinearOpMode {

    // ============ MOTOR DECLARATIONS ============
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotor fw1, fw2, fw3, fw4;

    // ============ SPEED SETTINGS ============
    private double shooterRPM = 4000;
    private static final double MAX_SHOOTER_RPM = 6000;
    private static final double MIN_SHOOTER_RPM = 4000;
    private double shooterRPMInc = 500;

    private double carrySpeed = 0.5;
    private double intakeSpeed = 1;
   
    // Shooter direction control
    private boolean shooterReversed = false;

    // ============ STATE VARIABLES ============
    private ElapsedTime runtime = new ElapsedTime();
    private boolean fieldCentric = false;
    private double robotHeading = 0;
    private VoltageSensor battery;

    private boolean yLast = false, xLast = false;
    private boolean a2Last = false;

    // ============ APRILTAG VARIABLES ============
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
   
    private int targetTagId = -1;
    private boolean autoDriveToTag = false;
    private static final double APPROACH_SPEED = 0.3;
    private static final double STRAFE_SPEED = 0.2;
    private static final double TURN_SPEED = 0.15;
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double DISTANCE_TOLERANCE = 2.0;
    private static final double BEARING_TOLERANCE = 3.0;

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addLine("READY - Press START");
        telemetry.addData("Mode", fieldCentric ? "FIELD" : "ROBOT");
        telemetry.addData("Shooter", "%.0f RPM", shooterRPM);
        telemetry.addData("Shooter Direction", shooterReversed ? "REVERSED" : "FORWARD");
        telemetry.addData("Carry/Intake", "%.0f%% / %.0f%%", carrySpeed*100, intakeSpeed*100);
        telemetry.addData("AprilTag", "Limelight via Ethernet Portal");
        telemetry.addLine("Controls: D-pad Up/Down = Camera Stream");
        telemetry.addLine("Gamepad2 X = Auto Drive to Any Detected Tag");
        telemetry.addLine("Gamepad1 Left Trigger = Reverse Intake");
        telemetry.addLine("Gamepad2 Left Trigger = Adjust Shooter Power");
        telemetry.addLine("Gamepad2 Up Analog = Toggle Shooter Reverse");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
           
            if (autoDriveToTag) {
                autoDriveToAprilTag();
            } else {
                drive();
            }
           
            shooter();
            carry();
            intake();
            handleAprilTagControls();
            showTelemetry();
           
            if (gamepad1.dpad_down) {
                if (visionPortal != null) visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                if (visionPortal != null) visionPortal.resumeStreaming();
            }

            sleep(20);
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initHardware() {
        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        backRightMotor = hardwareMap.dcMotor.get("br");

        fw1 = hardwareMap.dcMotor.get("fw1");
        fw2 = hardwareMap.dcMotor.get("fw2");
        fw3 = hardwareMap.dcMotor.get("fw3");
        fw4 = hardwareMap.dcMotor.get("fw4");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        fw1.setDirection(DcMotorSimple.Direction.REVERSE);
        fw2.setDirection(DcMotorSimple.Direction.FORWARD);
        fw3.setDirection(DcMotorSimple.Direction.FORWARD);
        fw4.setDirection(DcMotorSimple.Direction.FORWARD);

        fw1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fw2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fw1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fw2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fw3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fw4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    private void initAprilTag() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

            CameraName limelight = null;
           
            String[] possibleNames = {"Limelight", "camera", "Camera", "limelight"};
           
            for (String name : possibleNames) {
                try {
                    limelight = hardwareMap.get(CameraName.class, name);
                    if (limelight != null) {
                        telemetry.addData("Found camera as", name);
                        break;
                    }
                } catch (Exception e) {
                    // Try next name
                }
            }
           
            if (limelight == null) {
                telemetry.addData("Camera Error", "Limelight not found. Check configuration name.");
                visionPortal = null;
                return;
            }

            visionPortal = new VisionPortal.Builder()
                .setCamera(limelight)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();
           
        } catch (Exception e) {
            telemetry.addData("Camera Error", "Failed: " + e.getMessage());
            visionPortal = null;
        }
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double voltage = battery.getVoltage();
        double boost = Math.min(12.0 / voltage, 1.2);

        frontLeftMotor.setPower(frontLeftPower * boost);
        backLeftMotor.setPower(backLeftPower * boost);
        frontRightMotor.setPower(frontRightPower * boost);
        backRightMotor.setPower(backRightPower * boost);
    }

    private void autoDriveToAprilTag() {
        if (aprilTag == null || visionPortal == null) {
            telemetry.addData("Auto Drive", "Limelight not available");
            autoDriveToTag = false;
            return;
        }
       
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection targetDetection = null;
       
        if (targetTagId != -1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == targetTagId && detection.ftcPose != null) {
                    targetDetection = detection;
                    break;
                }
            }
        }
        else if (!currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.ftcPose != null) {
                    targetDetection = detection;
                    targetTagId = detection.id;
                    break;
                }
            }
        }
       
        if (targetDetection == null) {
            stopDriveMotors();
            telemetry.addData("Auto Drive", "No Target Found");
            return;
        }
       
        double range = targetDetection.ftcPose.range;
        double bearing = targetDetection.ftcPose.bearing;
        double yaw = targetDetection.ftcPose.yaw;
       
        if (Math.abs(range - DESIRED_DISTANCE) <= DISTANCE_TOLERANCE &&
            Math.abs(bearing) <= BEARING_TOLERANCE) {
            stopDriveMotors();
            telemetry.addData("Auto Drive", "Target Reached!");
            return;
        }
       
        double forwardPower = 0;
        double strafePower = 0;
        double turnPower = 0;
       
        if (range > DESIRED_DISTANCE + DISTANCE_TOLERANCE) {
            forwardPower = APPROACH_SPEED;
        } else if (range < DESIRED_DISTANCE - DISTANCE_TOLERANCE) {
            forwardPower = -APPROACH_SPEED;
        }
       
        if (Math.abs(bearing) > BEARING_TOLERANCE) {
            strafePower = Math.signum(bearing) * STRAFE_SPEED;
        }
       
        if (Math.abs(yaw) > BEARING_TOLERANCE) {
            turnPower = Math.signum(yaw) * TURN_SPEED;
        }
       
        double denominator = Math.max(Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
       
        double frontLeftPower = (forwardPower + strafePower + turnPower) / denominator;
        double backLeftPower = (forwardPower - strafePower + turnPower) / denominator;
        double frontRightPower = (forwardPower - strafePower - turnPower) / denominator;
        double backRightPower = (forwardPower + strafePower - turnPower) / denominator;
       
        double voltage = battery.getVoltage();
        double boost = Math.min(12.0 / voltage, 1.2);
       
        frontLeftMotor.setPower(frontLeftPower * boost);
        backLeftMotor.setPower(backLeftPower * boost);
        frontRightMotor.setPower(frontRightPower * boost);
        backRightMotor.setPower(backRightPower * boost);
    }
   
    private void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
   
    private void handleAprilTagControls() {
        if (gamepad2.x) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.ftcPose != null) {
                        targetTagId = detection.id;
                        autoDriveToTag = true;
                        telemetry.addData("Auto Drive", "Targeting ID " + targetTagId);
                        break;
                    }
                }
            } else {
                telemetry.addData("Auto Drive", "No tags detected!");
            }
            sleep(300);
        }
       
        if (gamepad2.a && !autoDriveToTag) {
            autoDriveToTag = false;
            targetTagId = -1;
            telemetry.addData("Auto Drive", "Stopped");
            sleep(200);
        }
    }

    private void shooter() {
        // Toggle shooter reverse with up analog stick (Gamepad2)
        if (gamepad2.left_stick_y < -0.5 && !yLast) {
            shooterReversed = !shooterReversed;
            sleep(200);
        }
        yLast = gamepad2.left_stick_y < -0.5;
       
        // Adjust shooter RPM with left trigger (Gamepad2)
        if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger <= 0.5) {
            shooterRPM += shooterRPMInc;
            if (shooterRPM > MAX_SHOOTER_RPM) shooterRPM = MIN_SHOOTER_RPM;
            sleep(200);
        }
       
        double shooterPower = shooterRPM / 6000.0;
        shooterPower = Math.min(Math.max(shooterPower, 0.0), 1.0);
       
        if (shooterReversed) {
            shooterPower = -shooterPower;
        }

        if (gamepad2.right_trigger > 0.5) {
            fw1.setPower(shooterPower);
            fw2.setPower(shooterPower);
        } else {
            fw1.setPower(0);
            fw2.setPower(0);
        }
    }

    private void carry() {
        if (gamepad2.a && !a2Last && !autoDriveToTag) {
            carrySpeed += 0.1;
            if (carrySpeed > 1.0) carrySpeed = 0.1;
        }
        a2Last = gamepad2.a;

        if (gamepad2.right_bumper) {
            fw3.setPower(carrySpeed);
        } else if (gamepad2.left_bumper) {
            fw3.setPower(-carrySpeed);
        } else {
            fw3.setPower(0);
        }
    }

    private void intake() {
        // Normal intake with right trigger (Gamepad1)
        if (gamepad1.right_trigger > 0.1) {
            fw4.setPower(-intakeSpeed);
        }
        // Reverse intake with left trigger (Gamepad1) - NOW ON GAMEPAD 1
        else if (gamepad1.left_trigger > 0.1) {
            fw4.setPower(intakeSpeed); // Positive = reverse direction
        }
        else {
            fw4.setPower(0);
        }
    }

    private void showTelemetry() {
        telemetry.addLine("==== ROBOT STATUS ====");
        telemetry.addData("Time", "%.1fs", runtime.seconds());
        telemetry.addData("Voltage", "%.1fV", battery.getVoltage());

        telemetry.addLine("");
        telemetry.addLine("==== APRILTAG INFO ====");
       
        if (aprilTag != null && visionPortal != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("Tags Detected", currentDetections.size());
            telemetry.addData("Target Tag ID", targetTagId == -1 ? "None" : targetTagId);
            telemetry.addData("Auto Drive Mode", autoDriveToTag ? "ON" : "OFF");
           
            if (!currentDetections.isEmpty()) {
                AprilTagDetection firstTag = currentDetections.get(0);
                if (firstTag.ftcPose != null) {
                    telemetry.addLine(String.format("Closest Tag ID %d: Range %.1f\" Bearing %.1f°",
                        firstTag.id, firstTag.ftcPose.range, firstTag.ftcPose.bearing));
                }
            }
        } else {
            telemetry.addData("Limelight", "NOT AVAILABLE - Check Ethernet connection");
        }

        telemetry.addLine("");
        telemetry.addLine("==== CONTROLS ====");
        telemetry.addData("Shooter", "%.0f RPM", shooterRPM);
        telemetry.addData("Shooter Direction", shooterReversed ? "REVERSED" : "FORWARD");
        telemetry.addData("Shooter ON", gamepad2.right_trigger > 0.5 ? "YES" : "NO");
        telemetry.addData("Carry", "%.0f%%", carrySpeed*100);
        telemetry.addData("Carry Dir",
                gamepad2.right_bumper ? "FWD" :
                gamepad2.left_bumper ? "REV" : "OFF");
        telemetry.addData("Intake", "%.0f%%", intakeSpeed*100);
        telemetry.addData("Intake Dir",
                gamepad1.right_trigger > 0.1 ? "IN" :
                gamepad1.left_trigger > 0.1 ? "OUT" : "OFF");
       
        telemetry.addLine("");
        telemetry.addLine("==== APRILTAG CONTROLS ====");
        telemetry.addLine("Gamepad2 X: Auto Drive to ANY detected tag");
        telemetry.addLine("Gamepad2 A: Stop Auto Drive");
       
        telemetry.addLine("");
        telemetry.addLine("==== SHOOTER CONTROLS ====");
        telemetry.addLine("Gamepad2 Right Trigger: Run Shooter");
        telemetry.addLine("Gamepad2 Left Trigger: Increase Shooter Power");
        telemetry.addLine("Gamepad2 Up Analog: Toggle Shooter Reverse");
       
        telemetry.addLine("");
        telemetry.addLine("==== INTAKE CONTROLS ====");
        telemetry.addLine("Gamepad1 Right Trigger: Intake IN");
        telemetry.addLine("Gamepad1 Left Trigger: Intake OUT (Reverse)");

        telemetry.update();
    }
}
