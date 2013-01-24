package org.openplans.tools.tracking.server.shared;

import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;


import com.vividsolutions.jts.geom.Geometry;

public class OsmSegmentWithVelocity extends OsmSegment {

  private final Double velocity;

  public OsmSegmentWithVelocity(InferredEdge inferredEdge, Double mean) {
    super(inferredEdge);
    this.velocity = mean;
  }

  public OsmSegmentWithVelocity(String i, Geometry g, String name,
    Double mean) {
    super(i, g, name);
    this.velocity = mean;
  }

  @JsonSerialize
  public Double getVelocity() {
    return velocity;
  }

}
