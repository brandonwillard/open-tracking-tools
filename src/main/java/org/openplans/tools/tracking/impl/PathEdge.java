package org.openplans.tools.tracking.impl;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

public class PathEdge {

  private final InferredEdge edge;
  private final Double distToStartOfEdge;
  
  private PathEdge(InferredEdge edge) {
    this.edge = edge;
    this.distToStartOfEdge = null;
  }
  
  public PathEdge(InferredEdge edge, double distToStartOfEdge) {
    this.edge = edge;
    this.distToStartOfEdge = distToStartOfEdge;
  }

  public InferredEdge getEdge() {
    return edge;
  }

  public double getDistToStartOfEdge() {
    return distToStartOfEdge;
  }

  @Override
  public String toString() {
    return "PathEdge [edge=" + edge + ", distToStartOfEdge="
        + distToStartOfEdge + "]";
  }

  private static PathEdge emptyPathEdge = new PathEdge(InferredGraph.getEmptyEdge());

  public static PathEdge getEmptyPathEdge() {
    return emptyPathEdge;
  }
  
}
