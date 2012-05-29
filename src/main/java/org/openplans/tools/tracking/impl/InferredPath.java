package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled and
 * the direction (by sign) 
 * @author bwillard
 *
 */
public class InferredPath {

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;
  
  public InferredPath(ImmutableList<PathEdge> edges, double totalPathDistance) {
    Preconditions.checkArgument(edges.size() > 1);
    // TODO remove/revise these sanity checks
    PathEdge prevEdge = null;
    for (PathEdge edge : edges) {
      if (prevEdge != null) {
        Preconditions.checkArgument(!edge.equals(prevEdge));
        if (prevEdge != PathEdge.getEmptyPathEdge()
            && edge != PathEdge.getEmptyPathEdge()) {
          final Vector start = edge.getDistToStartOfEdge() >= 0 ?
              edge.getInferredEdge().getStartPoint() : 
                edge.getInferredEdge().getEndPoint();
          final Vector end = edge.getDistToStartOfEdge() >= 0 ?
              prevEdge.getInferredEdge().getEndPoint() : 
                prevEdge.getInferredEdge().getStartPoint();
          final double dist = start.euclideanDistance(end);
              
          Preconditions.checkArgument(dist < 5d);
        }
      }
      prevEdge = edge;
    }
    this.edges = edges;
    this.totalPathDistance = totalPathDistance;
  }
  
  public InferredPath(ImmutableList<PathEdge> edges) {
    Preconditions.checkArgument(edges.size() > 0);
    this.edges = edges;
    final PathEdge lastEdge = Iterables.getLast(edges);
    final double direction = lastEdge.getDistToStartOfEdge() > 0 ? 1d : -1d;
    this.totalPathDistance = lastEdge.getDistToStartOfEdge() + direction * lastEdge.getInferredEdge().getLength();
  }

  private InferredPath() {
    this.edges = ImmutableList.of(PathEdge.getEmptyPathEdge());
    this.totalPathDistance = null;
  }

  public InferredPath(InferredEdge inferredEdge) {
    Preconditions.checkArgument(inferredEdge != InferredGraph.getEmptyEdge());
    this.edges = ImmutableList.of(PathEdge.getEdge(inferredEdge, 0d));
    this.totalPathDistance = inferredEdge.getLength();
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

  private static InferredPath emptyPath = new InferredPath();
  
  public static InferredPath getEmptyPath() {
    return emptyPath;
  }
  
  /**
   * Returns the edge that covers the given distance, or
   * null.
   * @param distance
   * @return
   */
  public PathEdge getEdgeAtDistance(double distance) {
    Preconditions.checkArgument(this != emptyPath);
    Preconditions.checkArgument(distance >= 0d);
    // TODO pre-compute/improve this
    for (PathEdge edge : this.edges) {
      if (edge.getDistToStartOfEdge() <= distance
          && distance < edge.getDistToStartOfEdge() + edge.getInferredEdge().getLength()) {
        return edge;
      }
    }
    
   return null;
  }
  
}
