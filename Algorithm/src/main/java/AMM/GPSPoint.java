package AMM;

import java.util.List;

public class GPSPoint {
    private final long timestamp;
    private final double latitude;
    private final double longitude;
    private final double direction;
    private final double velocity;

    public List<GPSPoint> deleted_gps_list;
    public PointsSet parent;

    public GPSPoint(long time, double lat, double lon, double dir, double vel){
        timestamp = time;
        latitude = lat;
        longitude = lon;
        direction = dir;
        velocity = vel;
    }

    public long getTimestamp(){
        return timestamp;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getDirection() {
        return direction;
    }
}
