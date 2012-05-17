package org.openplans.tools.tracking.impl;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;

public class InferredPath {

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;
  
  public InferredPath(ImmutableList<PathEdge> edges, double totalPathDistance) {
    this.edges = edges;
    this.totalPathDistance = totalPathDistance;
  }

  public InferredPath(ImmutableList<PathEdge> path) {
    this.edges = path;
    this.totalPathDistance = null;
  }

  public ImmutableList<PathEdge> getEdges() {
    return edges;
  }

  public double getTotalPathDistance() {
    return totalPathDistance;
  }

  @Override
  public String toString() {
    return "InferredPath [edges=" + edges + ", totalPathDistance="
        + totalPathDistance + "]";
  }

  private static InferredPath emptyPath = new InferredPath(ImmutableList.of(PathEdge.getEmptyPathEdge()));
  
  public static InferredPath emptyPath() {
    return emptyPath;
  }
  
  /**
   * Returns the edge that covers the given distance, or
   * null.
   * @param distance
   * @return
   */
  public PathEdge getEdge(double distance) {
    Preconditions.checkArgument(this != emptyPath);
    Preconditions.checkArgument(distance >= 0d);
    // TODO pre-compute/improve this
    for (PathEdge edge : this.edges) {
      if (edge.getDistToStartOfEdge() <= distance
          && distance < edge.getDistToStartOfEdge() + edge.getEdge().getLength()) {
        return edge;
      }
    }
    
   return null;
  }
  
}
