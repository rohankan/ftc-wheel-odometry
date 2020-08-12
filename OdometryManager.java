
public class OdometryManager
{
    // Calculation constants.
    private double robotWidth;

    // Location/orientation variables.
    private Point coords;
    private double theta;

    // Encoder classes.
    private EncoderManager leftEncoder;
    private EncoderManager rightEncoder;
    private EncoderManager strafeEncoder;
    double strafeEncoderDistancePerRobotRotation;

    private class EncoderManager
    {
        Encoder encoder;
        double prevDistance = 0;

        /**
         * Handles encoder operations relevant to odometry.
         * @param encoder Encoder instance
         */
        public EncoderManager(Encoder encoder)
        {
            this.encoder = encoder;
        }

        /**
         * Returns distance change since last method call.
         * @return Distance change
         */
        public getDistanceChange()
        {
            double distance = encoder.getDistance();
            double change = distance - prevDistance;
            prevDistance = Distance;
            return change;
        }
    }

    /**
     * Returns current (x, y) location
     * @return Current location
     */
    public Point getLocation()
    {
        return coords;
    }

    /**
     * Returns current orientation in radians
     * @return Current orientation in radians
     */
    public double getOrientation()
    {
        return theta;
    }

    /**
     * @param initialLocation Initial (x, y) location
     * @param initialTheta Initial global orientation
     * @param leftEncoder Left vertical encoder
     * @param rightEncoder Right vertical encoder
     * @param strafeEncoder Bottom strafe/horizontal encoder
     * @param robotWidth Robot width in same units as encoder distances
     * @param strafeEncoderDistancePerRobotRotation Distance the strafe
     * encoder travels when the robot does one full rotation.
     */
    public OdometryManager(Point initialLocation, double initialTheta,
                           Encoder leftEncoder, Encoder rightEncoder,
                           Encoder strafeEncoder, double robotWidth,
                           double strafeEncoderDistancePerRobotRotation)
    {
        this.leftEncoder = EncoderManager(leftEncoder);
        this.rightEncoder = EncoderManager(rightEncoder);
        this.strafeEncoder = EncoderManager(strafeEncoder);
        this.coords = initialLocation;
        this.theta = initialTheta;
        this.robotWidth = robotWidth;
        this.strafeEncoderDistancePerRobotRotation = strafeEncoderDistancePerRobotRotation;
    }

    /**
     * Rotates point about origin by a given angle.
     * @param point Point to rotate
     * @param theta Angle in radians, positive creates  counterclockwise rotation,
     *              negative creates clockwise rotation
     */
    private static Point rotateAboutOrigin(Point point, double theta)
    {
        // Pre-calculating sin and cos values.
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        // Rotating point about origin.
        double x = (point.x * cos) - (point.y * sin);
        double y = (point.x * sin) + (point.y * cos);

        return new Point(x, y);
    }

    /**
     * Updates the x, y, and theta values of the system.
     */
    public void update()
    {
        // Setting d1, or the change in left encoder distance.
        double d1 = leftEncoder.getDistanceChange();

        // Setting d2, or the change in right encoder distance.
        double d2 = rightEncoder.getDistanceChange();

        double dx1;
        double dy1;

        // If we ran the code in the else when d1 == d2,
        // theta would become 0, and the radius would be infinite.
        // Therefore, we have to handle this edge case separately.
        if (d1 == d2) {
            // For d1 to equal d2, the vertical encoders move in
            // a straight line.
            dx1 = 0;
            dy1 = d1;
        } else {
            // Averaging vertical encoder distances.
            double d = (d1 + d2) / 2;

            // Calculating angle change.
            double dTheta = (d2 - d1) / robotWidth;

            // Updating global orientation.
            theta += dTheta;

            // Creating and rotating robot-centric position.
            double r = d / dTheta;
            Point loc = new Point(r, 0);
            Point rot = rotateAboutOrigin(loc, dTheta);

            dx1 = loc.x - rot.x;
            dy1 = loc.y - rot.y;
        }

        // Setting dStrafe, or the change in strafe encoder position.
        double dStrafe = strafeEncoder.getDistanceChange();

        // Correcting for incorrect strafe values
        double turnOffset = dTheta * strafeEncoderDistancePerRobotRotation;
        double turnCorrectedStrafeDistance = dStrafe - turnOffset;

        double dx2 = Math.cos(dTheta) * turnCorrectedStrafeDistance;
        double dy2 = Math.sin(dTheta) * turnCorrectedStrafeDistance;

        // Creating the final robot-centric change in (x, y) position by
        // combining the positional changes derived from the vertical
        // encoder readings and strafe encoder readings.
        Point robotCentricChange = new Point(dx1 + dx2, dy1 + dy2);

        // Rotates the final robot-centric vector by the global orientation
        // about the origin to produce a field-centric vector.
        // NOTE: This program assumes that your field orientation system has
        // been setup like the unit circle; in other words, 0 radians follows
        // the x-axis, while π/2 radians follows the y-axis. Otherwise, strange
        // behavior may occur.
        Point fieldCentricChange = rotateAboutOrigin(robotCentricChange, theta);

        // Updating the global position.
        coords.x += fieldCentricChange.x;
        coords.y += fieldCentricChange.y;
    }
}
