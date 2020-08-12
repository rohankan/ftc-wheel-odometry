
public class Encoder
{
    double distancePerTick;

    /**
     * NOTE: Modify/adapt this class to your own encoder setup.
     * @param distancePerTick Distance (in your choice of units) per tick
     */
    public Encoder(double distancePerTick)
    {
        this.distancePerTick = distancePerTick;
    }

    /**
     * Returns number of ticks.
     * @return Number of ticks
     */
    private double getTicks()
    {
        return 0;  // Dummy value, change to actual implementation.
    }

    /**
     * Returns distance traveled.
     * @return Distance traveled
     */
    public double getDistance()
    {
        return getTicks() * distancePerTick;
    }
}
