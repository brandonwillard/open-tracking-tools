package org.openplans.tools.tracking.impl;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;

public class PathEdge {

  private final InferredEdge edge;
  private final Double distToStartOfEdge;
  
  private PathEdge(InferredEdge edge) {
    this.edge = edge;
    this.distToStartOfEdge = null;
  }
  
  private PathEdge(InferredEdge edge, double distToStartOfEdge) {
    Preconditions.checkArgument(edge != InferredGraph.getEmptyEdge());
    this.edge = edge;
    this.distToStartOfEdge = distToStartOfEdge;
  }

  public InferredEdge getInferredEdge() {
    return edge;
  }

  public Double getDistToStartOfEdge() {
    return distToStartOfEdge;
  }

  @Override
  public String toString() {
    return "PathEdge [edge=" + edge + ", distToStartOfEdge="
        + distToStartOfEdge + "]";
  }

  public static PathEdge getEdge(InferredEdge infEdge) {
    PathEdge edge;
    if (infEdge == InferredGraph.getEmptyEdge()) {
      edge = PathEdge.getEmptyPathEdge(); 
    } else {
      edge = new PathEdge(infEdge, 0d);
    }
    return edge;
  }
  
  public static PathEdge getEdge(InferredEdge infEdge, double distToStart) {
    PathEdge edge;
    if (infEdge == InferredGraph.getEmptyEdge()) {
      edge = PathEdge.getEmptyPathEdge(); 
    } else {
      edge = new PathEdge(infEdge, distToStart);
    }
    return edge;
  }
  
  private static PathEdge emptyPathEdge = new PathEdge(InferredGraph.getEmptyEdge());

  public static PathEdge getEmptyPathEdge() {
    return emptyPathEdge;
  }
  
}
