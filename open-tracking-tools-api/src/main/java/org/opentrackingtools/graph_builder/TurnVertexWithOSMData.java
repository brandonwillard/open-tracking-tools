package org.opentrackingtools.graph_builder;

import java.util.Set;

import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.edgetype.TurnEdge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.patch.Alert;
import org.opentripplanner.routing.vertextype.StreetVertex;
import org.opentripplanner.routing.vertextype.TurnVertex;

import com.vividsolutions.jts.geom.LineString;

public class TurnVertexWithOSMData extends TurnVertex {

  private static final long serialVersionUID =
      -2624941463575097067L;

  private final long way;

  private final long fromNode;

  private final long toNode;

  private final StreetEdge original;

  public TurnVertexWithOSMData(StreetEdge original,
    long way, long fromNode, long toNode, Graph graph,
    String id, LineString geometry, String name,
    double length, boolean back, Set<Alert> notes) {
    super(graph, id, geometry, name, length, back, notes);
    this.original = original;
    this.way = way;
    this.fromNode = fromNode;
    this.toNode = toNode;
  }

  public long getFromNode() {
    return fromNode;
  }

  public StreetEdge getOriginal() {
    return original;
  }

  public long getToNode() {
    return toNode;
  }

  public long getWay() {
    return way;
  }

  @Override
  public TurnEdge makeTurnEdge(StreetVertex out) {
    if (out instanceof TurnVertex) {
      return new SimpleTurnEdge(this, out);
    }
    return new SimpleTurnEdge(this, out);
  }

}
