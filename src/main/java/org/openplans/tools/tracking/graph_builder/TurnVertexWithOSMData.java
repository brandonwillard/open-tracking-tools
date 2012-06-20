package org.openplans.tools.tracking.graph_builder;

import java.util.Set;

import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.patch.Alert;
import org.opentripplanner.routing.vertextype.TurnVertex;

import com.vividsolutions.jts.geom.LineString;

public class TurnVertexWithOSMData extends TurnVertex {

  private static final long serialVersionUID = -2624941463575097067L;

  private final long way;

  private final long fromNode;

  private final long toNode;

  private final Edge original;

  public TurnVertexWithOSMData(Edge original, long way,
    long fromNode, long toNode, Graph graph, String id,
    LineString geometry, String name, double length, boolean back,
    Set<Alert> notes) {
    super(graph, id, geometry, name, length, back, notes);
    this.original = original;
    this.way = way;
    this.fromNode = fromNode;
    this.toNode = toNode;
  }

  public long getFromNode() {
    return fromNode;
  }

  public Edge getOriginal() {
    return original;
  }

  public long getToNode() {
    return toNode;
  }

  public long getWay() {
    return way;
  }

}
