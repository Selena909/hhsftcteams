package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp (name = "Field Centric Mecanum", group = "TeleOp")
  // Instantiate your turret mechanism

    @Override
    public void runOpMode() {
        // Declare our motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor intake = hardwareMap.dcMotor.get("slurpMotor");
        DcMotor mainShooter = hardwareMap.dcMotor.get("frontShooter");
        DcMotor backRoller = hardwareMap.dcMotor.get("backShooter");
        GoBildaPinpointDriver odo;

        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); 
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        // --- HARDWARE MAPPING ---
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        // Initialize the turret mechanism
        turretMechanism.init(hardwareMap);

        limelight.pipelineSwitch(-1);
        limelight.start();

        waitForStart();

        if (isStopRequested()) return;

        // Reset the mechanism timer at the start of the loop
        turretMechanism.resetTimer();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; 
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.donotBRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.donotBRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.donotBRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.donotBRAKE);

            if (gamepad1.options) {
                odo.resetPosAndIMU();
            }

            // --- INTAKE CONTROLS ---
            if (gamepad2.left_bumper) {
                intake.setPower(0.8); 
            } else if (gamepad2.x) {
                intake.setPower(-0.5); 
            } else {
                intake.setPower(0);
            }

            // --- SHOOTER CONTROLS ---
            if (gamepad2.right_bumper) {
                mainShooter.setPower(100); 
                backRoller.setPower(0.8);  
            } else {
                mainShooter.setPower(0);
                backRoller.setPower(0);
            }

            double botHeading = odo.getHeading(AngleUnit.degrees);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- AUTO-AIM VIA TURRET MECHANISM ---
            LLResult result = limelight.getLatestResult();
            
            // This runs your PD controller from TurretMechanismTutorial.java
            turretMechanism.update(result);

            // Telemetry for Debugging
            telemetry.addData("Current kP", turretMechanism.getkP());
            telemetry.addData("Current kD", turretMechanism.getkD());
            if (result != null && result.isValid()) {
                telemetry.addData("Limelight Offset", result.getTx());
            }
            telemetry.update();
        }
    }
}
