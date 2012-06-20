package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.List;
import java.util.Map;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled
 * and the direction (by sign)
 * 
 * @author bwillard
 * 
 */
public class InferredPath implements Comparable<InferredPath> {

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;
  public List<Integer> edgeIds = Lists.newArrayList();

  private static InferredPath emptyPath = new InferredPath();

  private InferredPath() {
    this.edges = ImmutableList.of(PathEdge.getEmptyPathEdge());
    this.totalPathDistance = null;
  }

  private InferredPath(ImmutableList<PathEdge> edges) {
    Preconditions.checkArgument(edges.size() > 0);
    this.edges = edges;

    PathEdge lastEdge = null;
    for (final PathEdge edge : edges) {
      if (!edge.isEmptyEdge()) {
        lastEdge = edge;
      }
      edgeIds.add(edge.getInferredEdge().getEdgeId());
    }
    this.totalPathDistance = lastEdge.getDistToStartOfEdge()
        + lastEdge.getInferredEdge().getLength();
  }

  private InferredPath(InferredEdge inferredEdge) {
    Preconditions.checkArgument(!inferredEdge.isEmptyEdge());
    this.edges = ImmutableList.of(PathEdge.getEdge(inferredEdge, 0d));
    this.totalPathDistance = inferredEdge.getLength();
  }

  @Override
  public int compareTo(InferredPath o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.edges.toArray(), o.edges.toArray());
    return comparator.toComparison();
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
    final InferredPath other = (InferredPath) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  public boolean isEmptyPath() {
    return this == emptyPath;
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

    /*-
     * A prior predictive is created for every path, since, in some instances,
     * we need to project onto an edge and then predict movement.
     */
    final MultivariateGaussian beliefPrediction = state.getBelief()
        .clone();
    final StandardRoadTrackingFilter filter = state
        .getMovementFilter();

    filter.predict(beliefPrediction, this.getEdges().get(0), 
        PathEdge.getEdge(state.getInferredEdge()));
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
      final double edgePredMarginalLogLik;
      if (edge.isEmptyEdge()) {
        filter.predict(edgeBelief, edge, prevEdge);
        // TODO meh?
        edgePredMarginalLogLik = 0d;
      } else {
        edge.predict(edgeBelief, obs);
        edgePredMarginalLogLik = edge
            .marginalPredictiveLogLikelihood(beliefPrediction);
      }

      final double edgePredTransLogLik = state
          .getEdgeTransitionDist().predictiveLogLikelihood(
              prevEdge.getInferredEdge(), edge.getInferredEdge());

      final double localPosVelPredLogLik = filter.logLikelihood(
          obs.getProjectedPoint(), edgeBelief, edge);

      final double localLogLik = edgePredMarginalLogLik
          + edgePredTransLogLik + localPosVelPredLogLik;

      Preconditions.checkArgument(!Double.isNaN(localLogLik));

      /*
       * We're only going to deal with the terminating edge for now.
       */
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
        this, beliefPrediction, edgeToPredictiveBeliefAndLogLikelihood, filter,
        pathLogLik);
  }

  public Double getTotalPathDistance() {
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
    if (this == emptyPath)
      return "InferredPath [empty path]";
    else
      return "InferredPath [edges=" + edgeIds
          + ", totalPathDistance=" + totalPathDistance + "]";
  }

  public static InferredPath getEmptyPath() {
    return emptyPath;
  }

  public static InferredPath getInferredPath(InferredEdge inferredEdge) {
    if (inferredEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(inferredEdge);
  }

  public static InferredPath getInferredPath(List<PathEdge> edges) {
    return new InferredPath(ImmutableList.copyOf(edges));
  }

  public static InferredPath getInferredPath(PathEdge pathEdge) {
    if (pathEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(ImmutableList.of(pathEdge));
  }

}
