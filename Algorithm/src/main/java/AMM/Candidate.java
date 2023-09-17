package AMM;

import com.graphhopper.routing.Path;
import com.graphhopper.storage.index.Snap;

import java.util.List;

public class Candidate {
    private final Snap snap;

    public PointsSet parent;
    public double probability;
    public double delta_theta = 180.0;

    public double min_length = Double.MAX_VALUE;
    public Candidate best_previous = null;
    public Path best_path = null;

    public double velocity_score;
    public double direction_score;
    public double position_score;

    public Candidate(Snap point){
        snap = point;
    }

    public Snap getSnap() {
        return snap;
    }

    public double getMinLength(){
        return min_length;
    }

    public void setDirectionScore(double sigma_theta) {
        delta_theta = AMM.getMap().getDeltaDirection(snap, parent.getObservation().getDirection());
        direction_score = Utils.calculateNormalScore(delta_theta, sigma_theta);
    }

    public void setPositionScore(double sigma_z){
        double delta_z = snap.getQueryDistance();
        position_score = Utils.calculateNormalScore(delta_z, sigma_z);
    }

    public boolean setVelocityScore(double v0, double delta_v){
        if (parent.con_previous == null){
            velocity_score = Utils.calculateVelocityScore(v0, v0, delta_v);
            min_length = 0;
        }
        else{
            List<Candidate> previous_list = parent.con_previous.getCandidates();
            velocity_score = 0;
            for (Candidate start : previous_list){
                Path path = AMM.getMap().findPath(start.getSnap(), this.getSnap());
                double score = 0;
                if (path.isFound()){
                    double length = path.getDistance();
                    if (length + start.min_length < min_length) {
                        min_length = length + start.min_length;
                        best_previous = start;
                        best_path = path;
                    }
                    double velocity = length / (double) (parent.getObservation().getTimestamp()- parent.con_previous.getObservation().getTimestamp());
                    score = Utils.calculateVelocityScore(velocity, v0, delta_v);
                }
                velocity_score += score * start.probability;
            }
        }
        return velocity_score > 0;
    }

}
