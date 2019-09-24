package com._604robotics.robotnik.utils;

public class AutonMovement {

	private AutonMovement() {}
	/**
	 * A class that represents a Drive Train.
	 */
	public static class DriveTrainProperties {
		
		public final double encoderResolution;
		public final double trackWidth;
		public final double wheelRadius;
		public final double maxSpeed;
		public final double angularMaxSpeed;
		
		/**
		 * Instantiates a DriveTrainProperties
		 * @param encoderResolution The clicks per full rotation of the encoder
         * @param trackWidth The horisiontial distance between the centers of the robot wheels in inches
         * @param wheelRadius The radius of the wheel in inches
         * @param maxSpeed The max allowed speed of the robot in meters per second
         * @param angularMaxSpeed The max allowed angualr speed of the robot in rotations per second
		 */
		public DriveTrainProperties(double encoderResolution, double trackWidth, double wheelRadius, double maxSpeed, double angularMaxSpeed) {
			this.encoderResolution = encoderResolution;
			this.trackWidth = trackWidth;
			this.wheelRadius = wheelRadius;
			this.maxSpeed = maxSpeed;
			this.angularMaxSpeed = angularMaxSpeed;
		}
		
		public double getDistancePerPulse() {
			return (2 * Math.PI * wheelRadius / encoderResolution);
		}
		
	}
}