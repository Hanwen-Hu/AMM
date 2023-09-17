package AMM;

import com.graphhopper.routing.Path;

import java.util.List;

public class PointsSet {
    private final GPSPoint observation;
    private final List<Candidate> candidates_list;
    public double v0;

    private double min_path = Double.MAX_VALUE;

    public double score;
    public PointsSet con_previous = null;
    public PointsSet adj_previous = null;
    public Candidate matched_candidate;
    public Path matched_path;


    public PointsSet(GPSPoint gps, List<Candidate> candidates){
        observation = gps;
        observation.parent = this;
        candidates_list = candidates;
        for (Candidate candidate : candidates){
            candidate.parent = this;
        }
    }

    public List<Candidate> getCandidates(){
        return candidates_list;
    }

    public GPSPoint getObservation() {
        return observation;
    }

    //给当前时刻的点打分，同时返回输入的前继节点是否连通
    public boolean setScore(PointsSet previous){
        boolean connected = false;
        this.con_previous = previous;
        for (Candidate current : candidates_list){
            current.setDirectionScore(10.0);
            current.setPositionScore(10.0);
            if (!current.setVelocityScore(v0,AMM.getDeltaV())) continue;
            if (current.getMinLength() < min_path) min_path = current.getMinLength();
            connected = true;
        }
        return connected;
    }

    //从当前时刻开始回溯，直到重新找到路径为止
    //反向通过访问Previous以链表的形式不断回溯
    //统计反向评分的删除点的累计数量，并与正向评分相比
    //取较优的结果
    //路径拼接采用最短路径的原则
    public void backward(){
        if (con_previous == null) return;
        PointsSet prev_last = con_previous.con_previous;
        while (prev_last!= null){
            double new_length = min_path;
            for (Candidate current : candidates_list) {
                for (Candidate past : prev_last.getCandidates()) {
                    Path path = AMM.getMap().findPath(past.getSnap(), current.getSnap());
                    if (path.isFound()) {
                        double length = path.getDistance() + past.getMinLength();
                        if (length < new_length) {
                            new_length = length;
                            current.best_previous = past;
                            con_previous = prev_last;
                        }
                    }
                }
            }
            //如果最短路径没有变短，则退出回溯
            if (new_length < min_path) {
                min_path = new_length;
                prev_last = prev_last.con_previous;
            }
            else break;
        }
    }
}
