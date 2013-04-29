package org.opentrackingtools.graph.otp;

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

  private final long fromNode;

  private final long toNode;

  private TurnVertexWithOSMData turnVertex;

  private final long way;

  public PlainStreetEdgeWithOSMData(long way, long fromNode,
    long toNode, IntersectionVertex startEndpoint,
    IntersectionVertex endEndpoint, LineString geometry, String name,
    double length, StreetTraversalPermission permissions, boolean back) {

    super(startEndpoint, endEndpoint, geometry, name, length,
        permissions, back);
    this.way = way;
    this.fromNode = fromNode;
    this.toNode = toNode;
  }

  @Override
  public TurnVertex createTurnVertex(Graph graph) {
    final String id = this.getId();
    final TurnVertexWithOSMData tv =
        new TurnVertexWithOSMData(this, this.way, this.fromNode,
            this.toNode, graph, id, this.getGeometry(),
            this.getName(), this.getLength(), this.back,
            this.getNotes());
    tv.setWheelchairNotes(this.getWheelchairNotes());
    tv.setWheelchairAccessible(this.isWheelchairAccessible());
    tv.setPermission(this.getPermission());
    tv.setRoundabout(this.isRoundabout());
    tv.setBogusName(this.hasBogusName());
    tv.setNoThruTraffic(this.isNoThruTraffic());
    tv.setStairs(this.isStairs());
    this.turnVertex = tv;
    return tv;
  }

  public long getFromNode() {
    return this.fromNode;
  }

  public long getToNode() {
    return this.toNode;
  }

  public TurnVertexWithOSMData getTurnVertex() {
    return this.turnVertex;
  }

  public long getWay() {
    return this.way;
  }
}
