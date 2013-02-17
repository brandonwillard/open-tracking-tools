package org.opentrackingtools.graph.otp.impl;

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
public class PlainStreetEdgeWithOSMData extends
    PlainStreetEdge {

  private static final long serialVersionUID =
      -8417533365831538395L;

  private final long way;

  private final long fromNode;

  private final long toNode;

  private TurnVertexWithOSMData turnVertex;

  public PlainStreetEdgeWithOSMData(long way,
    long fromNode, long toNode,
    IntersectionVertex startEndpoint,
    IntersectionVertex endEndpoint, LineString geometry,
    String name, double length,
    StreetTraversalPermission permissions, boolean back) {

    super(startEndpoint, endEndpoint, geometry, name,
        length, permissions, back);
    this.way = way;
    this.fromNode = fromNode;
    this.toNode = toNode;
  }

  @Override
  public TurnVertex createTurnVertex(Graph graph) {
    final String id = getId();
    final TurnVertexWithOSMData tv =
        new TurnVertexWithOSMData(this, way, fromNode,
            toNode, graph, id, getGeometry(), getName(),
            getLength(), back, getNotes());
    tv.setWheelchairNotes(getWheelchairNotes());
    tv.setWheelchairAccessible(isWheelchairAccessible());
    tv.setPermission(getPermission());
    tv.setRoundabout(isRoundabout());
    tv.setBogusName(hasBogusName());
    tv.setNoThruTraffic(isNoThruTraffic());
    tv.setStairs(isStairs());
    turnVertex = tv;
    return tv;
  }

  public long getFromNode() {
    return fromNode;
  }

  public long getToNode() {
    return toNode;
  }

  public TurnVertexWithOSMData getTurnVertex() {
    return turnVertex;
  }

  public long getWay() {
    return way;
  }
}
