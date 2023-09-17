package AMM;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.BidirRoutingAlgorithm;
import com.graphhopper.routing.DijkstraBidirectionRef;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.*;
import com.graphhopper.util.shapes.BBox;
import org.locationtech.jts.geom.Envelope;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.graphhopper.util.DistancePlaneProjection.DIST_PLANE;

public class MapConfig {
    private final LocationIndexTree location_index;
    private final Graph graph;
    private final Weighting weighting;
    private final BooleanEncodedValue subnetwork;
    private final int max_visited_nodes;

    public MapConfig(String path){
        String weight = "fastest";
        GraphHopperConfig config = new GraphHopperConfig();
        config.putObject("graph.flag_encoders", "car");
        config.putObject("graph.location", "MatchCache");
        config.putObject("datareader.file", path);

        List<Profile> Profiles = new ArrayList<>();
        Profiles.add(new Profile("car").setVehicle("car").setWeighting(weight));
        config.setProfiles(Profiles);

        GraphHopper hopper = new GraphHopper().init(config);
        hopper.importOrLoad();

        PMap hints = new PMap();
        hints.putObject("profile", 300000);
        String profileStr = hints.getString("profile", "car");
        Profile profile = hopper.getProfile(profileStr);

        location_index = (LocationIndexTree) hopper.getLocationIndex();
        graph = hopper.getGraphHopperStorage();
        weighting = hopper.createWeighting(profile, hints);
        subnetwork = hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key(profileStr));
        max_visited_nodes = hints.getInt(Parameters.Routing.MAX_VISITED_NODES, Integer.MAX_VALUE);
    }

    public List<PointsSet> generateSequence(List<GPSPoint> Observations){
        List<PointsSet> Points_List = new ArrayList<>();
        for (GPSPoint observation : Observations){
            List<Snap> Snaps = findCandidateSnaps(observation);
            List<Candidate> Candidates = new ArrayList<>();
            for (Snap point : Snaps) Candidates.add(new Candidate(point));
            Points_List.add(new PointsSet(observation, Candidates));
        }
        return Points_List;
    }

    public Path findPath(Snap start, Snap end){
        BidirRoutingAlgorithm router;
        router = new DijkstraBidirectionRef(graph, weighting, TraversalMode.EDGE_BASED) {
            @Override
            protected void initCollections(int size) {
                super.initCollections(50);
            }
        };
        router.setMaxVisitedNodes(max_visited_nodes);
        return router.calcPath(start.getClosestNode(), end.getClosestNode());
    }

    public double getDeltaDirection(Snap snap, double direction){
        double direction1 = Double.MAX_VALUE;
        double direction2 = Double.MAX_VALUE;
        EdgeIteratorState edge = snap.getClosestEdge();
        boolean start_to_end = edge.get(weighting. getFlagEncoder().getAccessEnc());
        boolean end_to_start = edge.getReverse(weighting.getFlagEncoder().getAccessEnc());
        if (start_to_end){
            direction1 = Utils.calculateDirection(edge.fetchWayGeometry(FetchMode.ALL).get(0).lat, edge.fetchWayGeometry(FetchMode.ALL).get(0).lon,
                    edge.fetchWayGeometry(FetchMode.ALL).get(1).lat, edge.fetchWayGeometry(FetchMode.ALL).get(1).lon);
        }
        if (end_to_start){
            direction2 = Utils.calculateDirection(edge.fetchWayGeometry(FetchMode.ALL).get(1).lat, edge.fetchWayGeometry(FetchMode.ALL).get(1).lon,
                    edge.fetchWayGeometry(FetchMode.ALL).get(0).lat, edge.fetchWayGeometry(FetchMode.ALL).get(0).lon);
        }
        double delta_direction;
        delta_direction = Math.min(Math.min(Math.abs(direction1-direction),Math.abs(360.0-Math.abs(direction1-direction))),
                Math.min(Math.abs(direction2-direction), Math.abs(360.0-Math.abs(direction2-direction))));
        return delta_direction;
    }

    private List<Snap> findCandidateSnaps(GPSPoint observation) {
        double lat = observation.getLatitude();
        double lon = observation.getLongitude();
        double rLon = (AMM.getRadius() * 360.0 / DistanceCalcEarth.DIST_EARTH.calcCircumference(lat));
        double rLat = AMM.getRadius() / DistanceCalcEarth.METERS_PER_DEGREE;
        Envelope envelope = new Envelope(lon, lon, lat, lat);
        for (int i = 0; i < 50; i++) {
            envelope.expandBy(rLon, rLat);
            List<Snap> snaps = findCandidateSnapsInBBox(lat, lon, BBox.fromEnvelope(envelope));
            if (!snaps.isEmpty()) {
                return snaps;
            }
        }
        return Collections.emptyList();
    }

    private List<Snap> findCandidateSnapsInBBox(double queryLat, double queryLon, BBox queryShape) {
        EdgeFilter edgeFilter = new DefaultSnapFilter(weighting, subnetwork);
        List<Snap> snaps = new ArrayList<>();
        IntHashSet seenEdges = new IntHashSet();
        IntHashSet seenNodes = new IntHashSet();
        location_index.query(queryShape, edgeId -> {
            EdgeIteratorState edge = graph.getEdgeIteratorStateForKey(edgeId * 2);
            if (seenEdges.add(edgeId) && edgeFilter.accept(edge)) {
                Snap snap = new Snap(queryLat, queryLon);
                location_index.traverseEdge(queryLat, queryLon, edge, (node, normedDist, wayIndex, pos) -> {
                    if (normedDist < snap.getQueryDistance()) {
                        snap.setQueryDistance(normedDist);
                        snap.setClosestNode(node);
                        snap.setWayIndex(wayIndex);
                        snap.setSnappedPosition(pos);
                    }
                });
                double dist = DIST_PLANE.calcDenormalizedDist(snap.getQueryDistance());
                snap.setClosestEdge(edge);
                snap.setQueryDistance(dist);
                if (snap.isValid() && (snap.getSnappedPosition() != Snap.Position.TOWER || seenNodes.add(snap.getClosestNode()))) {
                    snap.calcSnappedPoint(DistanceCalcEarth.DIST_EARTH);
                    if (queryShape.contains(snap.getSnappedPoint().lat, snap.getSnappedPoint().lon)) {
                        snaps.add(snap);
                    }
                }
            }
        });
        return snaps;
    }

}
