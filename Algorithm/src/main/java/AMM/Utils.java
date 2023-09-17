package AMM;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.List;

public class Utils {
    public static double setVelocityVector(List<Double> velocity_list, PointsSet point){
        velocity_list.remove(0);
        velocity_list.add(point.getObservation().getVelocity());
        double average = 0;
        for (double velocity : velocity_list) average += velocity;
        return average / velocity_list.size();
    }

    public static double calculateDistance(double lat_1, double lon_1, double lat_2, double lon_2){
        lat_1 = lat_1 / 180.0 * Math.PI;
        lat_2 = lat_2 / 180.0 * Math.PI;
        lon_1 = lon_1 / 180.0 * Math.PI;
        lon_2 = lon_2 / 180.0 * Math.PI;
        return 6371000.0 * Math.acos(Math.min(Math.cos(lat_1)*Math.cos(lat_2)*Math.cos(lon_2-lon_1)+Math.sin(lat_1)*Math.sin(lat_2),1.0));
    }

    public static double calculateDirection(double lat_1, double lon_1, double lat_2, double lon_2) {
        lat_1 = lat_1 / 180.0 * Math.PI;
        lat_2 = lat_2 / 180.0 * Math.PI;
        lon_1 = lon_1 / 180.0 * Math.PI;
        lon_2 = lon_2 / 180.0 * Math.PI;
        double y = Math.sin(lon_2 - lon_1) * Math.cos(lat_2);
        double x = Math.cos(lat_1) * Math.sin(lat_2) - Math.sin(lat_1) * Math.cos(lat_2) * Math.cos(lon_2 - lon_1);
        double theta = Math.atan2(y, x);
        return (theta * 180.0 / Math.PI + 360.0) % 360;
    }

    public static double calculateVelocityScore(double velocity, double v_0, double delta_v){
        double v_max = v_0 + delta_v;
        double c = Math.pow(v_0 + v_max, 2.0) * 3.0 / (v_0 * 8.0 + v_max * 4.0);
        if (velocity < v_0) return c * 2.0 / (v_0 + v_max);
        else if (velocity < v_max) return c * 2.0 * (v_max - velocity) / (Math.pow(v_max, 2.0) - Math.pow(v_0, 2.0));
        else return 0.0;
    }

    public static double calculateNormalScore(double delta, double sigma){
        double fraction = Math.pow(delta / sigma, 2.0);
        return Math.sqrt(2.0) * Math.exp(- fraction / 2.0);
    }

    public static double maximizeScore(List<Candidate> Candidates, double lambda_1, double lambda_2){
        lambda_1 *= 10.0;
        lambda_2 = Math.exp(-lambda_2*Candidates.size());
        double [][] Score_matrix = new double[3][Candidates.size()];
        for (int i=0; i< Candidates.size(); ++i){
            Score_matrix[0][i] = Candidates.get(i).position_score;
            Score_matrix[1][i] = Candidates.get(i).direction_score;
            Score_matrix[2][i] = Candidates.get(i).velocity_score;
        }
        RealVector Result = Zoutendijk.getResult(Score_matrix, Candidates.size(), 3, lambda_1, lambda_2);
        double [] Result_array = Result.toArray();
        RealVector Weight = Result.getSubVector(0,3);
        double score = 0.0;
        RealMatrix Score = MatrixUtils.createRealMatrix(Score_matrix);
        for (int i=0; i< Candidates.size(); ++i){
            Candidates.get(i).probability = Result_array[3 + i];
            score += Weight.dotProduct(Score.getColumnVector(i)) * Result_array[3 + i];
        }
        return score;
    }
}
