package org.openplans.tools.tracking.server.shared;

import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.opentrackingtools.impl.graph.InferredEdge;


import com.vividsolutions.jts.geom.Geometry;

public class OsmSegmentWithVelocity extends OsmSegment {

  private final Double velocity;

  public OsmSegmentWithVelocity(InferredEdge edge, Double mean) {
    super(edge);
    this.velocity = mean;
  }

  public OsmSegmentWithVelocity(Integer i, Geometry g, String name,
    Double mean) {
    super(i, g, name);
    this.velocity = mean;
  }

  @JsonSerialize
  public Double getVelocity() {
    return velocity;
  }

}