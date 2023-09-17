package AMM;

import com.graphhopper.routing.Path;
import com.graphhopper.util.PointList;
import com.graphhopper.util.shapes.GHPoint;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AMM {
    private List<Candidate> matched_list;
    private double match_length;

    private static MapConfig map;

    //parameters
    private static double sigma_z;
    private static double sigma_theta;
    private static double radius;
    private final double score_threshold;
    private static double lambda1;
    private static double lambda2;
    private static double delta_v;

    public AMM(String map_path){
        map = new MapConfig(map_path);
        sigma_z = 20.0;
        sigma_theta = 30.0;
        radius = 50.0;
        score_threshold = 0.6;
        lambda1 = 1.0;
        lambda2 = 2.0;
        delta_v = 5.0;
    }

    public static double getDeltaV() {
        return delta_v;
    }

    public static double getRadius() {
        return radius;
    }

    public static MapConfig getMap() {
        return map;
    }

    public double match(List<GPSPoint> gps_list, int id){
        String gps_str = "id,lat,lon,dir,vel\n", candidate_str = "id,lat,lon\n", path_str = "lat,lon\n";
        List<Double> velocity = new ArrayList<>(); velocity.add(5.0); velocity.add(5.0); velocity.add(5.0); velocity.add(5.0); velocity.add(5.0);
        List<PointsSet> track_list = generateSequence(gps_list);
        PointsSet previous = null;
        int cumulative_delete = 0;//累计删除的异常点数量
        for (int i=0; i < track_list.size(); ++i) {
            PointsSet current = track_list.get(i);
            //写入观测点信息
            gps_str += i + "," + current.getObservation().getLatitude() + "," + current.getObservation().getLongitude() + "," + current.getObservation().getDirection() + "," + current.getObservation().getVelocity() + "\n";
            for(Candidate point : current.getCandidates()) {
                candidate_str += i + "," + point.getSnap().getSnappedPoint().lat + "," + point.getSnap().getSnappedPoint().lon + "\n";
            }
            current.v0 = Utils.setVelocityVector(velocity, current);//设置当前临界速度
            if (current.setScore(previous)) { //将previous赋给con_previous，检验current与previous的连通性，并对三个特征评分
                current.score = Utils.maximizeScore(current.getCandidates(), lambda1, lambda2);//将评分最大化得到总评分
                if (current.score > score_threshold) {
                    previous = current;//如果评分超过阈值，则将当前点设置为previous
                    if (cumulative_delete > 2) current.backward();//如果累计删点数量达到3，则触发回溯机制
                    cumulative_delete = 0;//如果当前时刻点质量正常，则累计数量清零
                }
                else cumulative_delete++; //评分低于阈值，跳过该点，累计删除数量+1
            }
            else cumulative_delete++; //未连通，跳过该点，累计删除数量+1
        }
        //倒序生成匹配路径
        int location = track_list.size()-1;
        List<Path> match_result = null;
        while (location >0 && match_result == null){
            match_result = generatePath(track_list.get(location));
            location--;
        }
        if (match_result == null) return 0;
        for (Path short_path : match_result){
            PointList points = short_path.calcPoints();
            for (GHPoint point : points){
                path_str += point.lat + "," + point.lon + "\n";
            }
        }
        writeFile(gps_str, candidate_str, path_str);
        updateParameters(id);
        return match_length;
    }


    private List<PointsSet> generateSequence(List<GPSPoint> gps_list){
        return map.generateSequence(gps_list);
    }

    private List<Path> generatePath(PointsSet final_point){
        Candidate last_point = null;
        double min_length = Double.MAX_VALUE;
        for (Candidate candidate : final_point.getCandidates()){
            if (candidate.getMinLength() < min_length) last_point = candidate;
        }
        if (last_point == null) {
            return null;
        }
        matched_list = new ArrayList<>();
        matched_list.add(last_point);
        match_length = last_point.min_length;
        List<Path> result = new ArrayList<>();
        while (last_point.best_previous != null){
            result.add(last_point.best_path);
            last_point = last_point.best_previous;
            matched_list.add(last_point);
        }
        Collections.reverse(matched_list);
        Collections.reverse(result);
        return result;
    }

    private void updateParameters(int id){
        double new_sigma_z = 0, new_sigma_theta = 0;
        for (Candidate candidate : matched_list) {
            new_sigma_z += Math.pow(candidate.getSnap().getQueryDistance(), 2.0);
            new_sigma_theta += Math.pow(candidate.delta_theta, 2.0);
        }
        new_sigma_z = Math.sqrt(new_sigma_z / matched_list.size());
        new_sigma_theta = Math.sqrt(new_sigma_theta / matched_list.size());
        sigma_z = (id * sigma_z + new_sigma_z) / (id + 1);
        sigma_theta = (id * sigma_theta + new_sigma_theta) / (id +1);
    }

    private void writeFile(String observation_str, String candidate_str, String path_str){
        File observation = new File("MatchResult/observation.csv");
        File candidate = new File("MatchResult/candidate.csv");
        File path = new File("MatchResult/path.csv");
        try {
            FileOutputStream fos1 = new FileOutputStream(observation);OutputStreamWriter dos1 = new OutputStreamWriter(fos1);dos1.write(observation_str);dos1.close();
            FileOutputStream fos2 = new FileOutputStream(candidate);OutputStreamWriter dos2 = new OutputStreamWriter(fos2);dos2.write(candidate_str);dos2.close();
            FileOutputStream fos3 = new FileOutputStream(path);OutputStreamWriter dos3 = new OutputStreamWriter(fos3);dos3.write(path_str);dos3.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
