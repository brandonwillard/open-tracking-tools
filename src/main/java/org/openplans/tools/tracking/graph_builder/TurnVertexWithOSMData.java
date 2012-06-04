package org.openplans.tools.tracking.graph_builder;

import java.util.Set;

import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.patch.Alert;
import org.opentripplanner.routing.vertextype.TurnVertex;

import com.vividsolutions.jts.geom.LineString;

public class TurnVertexWithOSMData extends TurnVertex {

    private static final long serialVersionUID = -2624941463575097067L;

    private long way;

    private long fromNode;

    private long toNode;

    public TurnVertexWithOSMData(long way, long fromNode, long toNode, Graph graph, String id,
            LineString geometry, String name, double length, boolean back, Set<Alert> notes) {
        super(graph, id, geometry, name, length, back, notes);
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

}
