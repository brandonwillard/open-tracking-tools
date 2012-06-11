package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.Map;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled
 * and the direction (by sign)
 * 
 * @author bwillard
 * 
 */
public class InferredPath {

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;

  private static InferredPath emptyPath = new InferredPath();

  private InferredPath() {
    this.edges = ImmutableList.of(PathEdge.getEmptyPathEdge());
    this.totalPathDistance = null;
  }

  public InferredPath(ImmutableList<PathEdge> edges) {
    Preconditions.checkArgument(edges.size() > 0);
    this.edges = edges;
    final PathEdge lastEdge = Iterables.getLast(edges);
    final double direction = lastEdge.getDistToStartOfEdge() >= 0 ? 1d
        : -1d;
    this.totalPathDistance = lastEdge.getDistToStartOfEdge()
        + direction * lastEdge.getInferredEdge().getLength();
  }

  public InferredPath(ImmutableList<PathEdge> edges,
    double totalPathDistance) {
    Preconditions.checkArgument(edges.size() >= 1);
    this.edges = edges;
    this.totalPathDistance = totalPathDistance;
  }

  public InferredPath(InferredEdge inferredEdge) {
    Preconditions.checkArgument(inferredEdge != InferredGraph
        .getEmptyEdge());
    this.edges = ImmutableList.of(PathEdge.getEdge(inferredEdge, 0d));
    this.totalPathDistance = inferredEdge.getLength();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    InferredPath other = (InferredPath) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  /**
   * Returns the edge that covers the given distance, or null.
   * 
   * @param distance
   * @return
   */
  public PathEdge getEdgeAtDistance(double distance) {
    Preconditions.checkArgument(this != emptyPath);
    Preconditions.checkArgument(distance >= 0d);
    // TODO pre-compute/improve this
    for (final PathEdge edge : this.edges) {
      if (edge.getDistToStartOfEdge() <= distance
          && distance < edge.getDistToStartOfEdge()
              + edge.getInferredEdge().getLength()) {
        return edge;
      }
    }

    return null;
  }

  public ImmutableList<PathEdge> getEdges() {
    return edges;
  }

  /**
   * XXX: the state must have a prior predictive mean.
   * 
   * @param obs
   * @param state
   * @return
   */
  public InferredPathEntry getPredictiveLogLikelihood(
    Observation obs, VehicleState state) {

    final MultivariateGaussian beliefPrediction = state.getBelief();
    final StandardRoadTrackingFilter filter = state
        .getMovementFilter();

    /*-
     * Compute predictive dist. over path
     * Note that this path should always start with the edge that
     * this state is currently on.
     */
    PathEdge prevEdge = PathEdge.getEdge(state.getInferredEdge());
    double pathLogLik = Double.NEGATIVE_INFINITY;
    final Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBeliefAndLogLikelihood = Maps
        .newHashMap();

    for (final PathEdge edge : this.getEdges()) {

      /*
       * If we're going off-road, then pass the edge we used to be on.
       */
      final MultivariateGaussian edgeBelief = beliefPrediction
          .clone();
      if (edge == PathEdge.getEmptyPathEdge()) {
        filter.predict(edgeBelief, edge, prevEdge);
      } else {
        edge.predict(edgeBelief);
      }

      // TODO should we use cumulative transition?
      double localLogLik = state.getEdgeTransitionDist()
          .predictiveLogLikelihood(
              prevEdge.getInferredEdge(), edge.getInferredEdge());
      localLogLik += filter.logLikelihood(
          obs.getProjectedPoint(), edgeBelief, edge);

      Preconditions.checkArgument(!Double.isNaN(localLogLik));

      edgeToPredictiveBeliefAndLogLikelihood.put(
          edge, new DefaultWeightedValue<MultivariateGaussian>(
              edgeBelief.clone(), localLogLik));

      /*
       * Add likelihood for this edge to the path total
       */
      pathLogLik = LogMath.add(pathLogLik, localLogLik);
      prevEdge = edge;
    }

    return new InferredPathEntry(
        this, edgeToPredictiveBeliefAndLogLikelihood, filter,
        pathLogLik);
  }

  public double getTotalPathDistance() {
    return totalPathDistance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result
        + ((edges == null) ? 0 : edges.hashCode());
    return result;
  }

  @Override
  public String toString() {
    return "InferredPath [edges=" + edges + ", totalPathDistance="
        + totalPathDistance + "]";
  }

  public static InferredPath getEmptyPath() {
    return emptyPath;
  }

}
