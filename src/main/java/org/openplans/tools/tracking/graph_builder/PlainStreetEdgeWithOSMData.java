package org.openplans.tools.tracking.graph_builder;

import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetTraversalPermission;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.vertextype.IntersectionVertex;
import org.opentripplanner.routing.vertextype.TurnVertex;

import com.vividsolutions.jts.geom.LineString;

/**
 * 
 * @author novalis
 *
 */
public class PlainStreetEdgeWithOSMData extends PlainStreetEdge {

    private static final long serialVersionUID = -8417533365831538395L;

    private long way;

    private long fromNode;

    private long toNode;

    public PlainStreetEdgeWithOSMData(long way, long fromNode, long toNode,
            IntersectionVertex startEndpoint, IntersectionVertex endEndpoint, LineString geometry,
            String name, double length, StreetTraversalPermission permissions, boolean back) {

        super(startEndpoint, endEndpoint, geometry, name, length, permissions, back);
        this.way = way;
        this.fromNode = fromNode;
        this.toNode = toNode;
    }

    public long getWay() {
        return way;
    }

    public long getFromNode() {
        return fromNode;
    }

    public long getToNode() {
        return toNode;
    }

    public TurnVertex createTurnVertex(Graph graph) {
        String id = getId();
        TurnVertexWithOSMData tv = new TurnVertexWithOSMData(way, fromNode, toNode,graph, id, getGeometry(), getName(), getLength(), back,
                getNotes());
        tv.setWheelchairNotes(getWheelchairNotes());
        tv.setWheelchairAccessible(isWheelchairAccessible());
//        tv.setCrossable(isCrossable());
        tv.setPermission(getPermission());
        tv.setRoundabout(isRoundabout());
        tv.setBogusName(hasBogusName());
        tv.setNoThruTraffic(isNoThruTraffic());
        tv.setStairs(isStairs());
        return tv;
    }
}
