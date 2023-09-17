import AMM.AMM;
import AMM.GPSPoint;
import AMM.Utils;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class Matching {
    private final String city = "Shanghai";

    public static void main(String[] args) {
        Matching match = new Matching();
        AMM algorithm = new AMM("MatchMap/Shanghai.osm.pbf");
        for (int index=0; index<200; index++){
            System.out.println("Index " + index);
            List<GPSPoint> track_list = match.readTrack(index);
            double match_length = algorithm.match(track_list, index);
            double gps_length = match.readGround(index);
            System.out.println("Matching Error " +Math.abs(match_length-gps_length)/gps_length);
        }
    }

    private List<GPSPoint> readTrack(int index) {
        List<GPSPoint> track = new ArrayList<>();
        String file = "MatchData/" + city + "/track/" + index + ".csv";
        BufferedReader reader;
        try {
            reader = new BufferedReader(new FileReader(file));
            reader.readLine();
            String line;
            line = reader.readLine();
            while (line != null) {
                String[] splits = line.split(",");
                long time = Long.parseLong(splits[2])/1000;
                double latitude = Double.parseDouble(splits[3]);
                double longitude = Double.parseDouble(splits[4]);
                double velocity = Double.parseDouble(splits[5]);
                double direction = Double.parseDouble(splits[6]);
                line = reader.readLine();
                track.add(new GPSPoint(time, latitude, longitude, direction, velocity));
            }
            reader.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return filterTracks(track);
    }

    private double readGround(int index){
        List<GPSPoint> track = new ArrayList<>();
        String path = "MatchData/" + city + "/ground/" + index + ".csv";
        File ground = new File("MatchResult/ground.csv");
        BufferedReader reader;
        double distance = 0;
        try {
            reader = new BufferedReader(new FileReader(path));
            reader.readLine();
            String line, ground_str = "lat,lon\n";
            line = reader.readLine();
            double pre_latitude = 0, pre_longitude = 0;
            while (line != null) {
                String[] splits = line.split(",");
                double latitude = Double.parseDouble(splits[2]);
                double longitude = Double.parseDouble(splits[3]);
                line = reader.readLine();
                ground_str += latitude + "," + longitude + "\n";
                if (pre_latitude != 0) distance += Utils.calculateDistance(pre_latitude, pre_longitude, latitude, longitude);
                pre_latitude = latitude;
                pre_longitude = longitude;
            }
            reader.close();
            FileOutputStream fos1 = new FileOutputStream(ground);
            OutputStreamWriter dos1 = new OutputStreamWriter(fos1);
            dos1.write(ground_str);
            dos1.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return distance;
    }

    private List<GPSPoint> filterTracks(List <GPSPoint> track){
        List<GPSPoint> filtered = new ArrayList<>();
        filtered.add(track.get(0));
        for (int i=1; i<track.size(); ++i){
            if (Utils.calculateDistance(track.get(i).getLatitude(), track.get(i).getLongitude(), filtered.get(filtered.size()-1).getLatitude(), filtered.get(filtered.size()-1).getLongitude())>2 * AMM.getRadius()){
                filtered.add(track.get(i));
            }
        }
        return filtered;
    }
}


