package org.openplans.tools.tracking.impl;

import java.util.Map;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;

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
    Preconditions.checkArgument(edges.size() >= 1);
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
    final double direction = lastEdge.getDistToStartOfEdge() >= 0 ? 1d : -1d;
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

  public InferredPathEntry getPredictiveLogLikelihood(Observation obs, VehicleState state) {
    
    /*-
     * Produce the distance prediction.
     * Note: here the road beliefs start at 0
     */
    final MultivariateGaussian beliefPrediction = state.getBelief().clone();
    final PathEdge firstEdge = this.getEdges().get(0);
    final StandardRoadTrackingFilter filter = state.getMovementFilter();
    filter.predict(beliefPrediction, firstEdge, PathEdge.getEdge(state.getEdge(), 0d));
    
    /*-
     * Compute predictive dist. over path
     * Note that this path should always start with the edge that
     * this state is currently on.
     */
    PathEdge prevEdge = PathEdge.getEdge(state.getInferredEdge());
    double pathLogLik = Double.NEGATIVE_INFINITY;
    final Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> 
      edgeToPredictiveBeliefAndLogLikelihood = Maps.newHashMap();
    
    for (final PathEdge edge : this.getEdges()) {

      /*
       * If we're going off-road, then pass the edge we used to be on.
       */
      final MultivariateGaussian edgeBelief = beliefPrediction.clone();
      if (edge == PathEdge.getEmptyPathEdge()) {
        filter.predict(edgeBelief, edge, prevEdge);
      } else {
        edge.predict(edgeBelief);
      }

      // TODO should we use cumulative transition?
      double localLogLik = state.getEdgeTransitionDist()
          .predictiveLogLikelihood(prevEdge.getInferredEdge(),
              edge.getInferredEdge());
      localLogLik += filter.logLikelihood(obs.getProjectedPoint(),
          edgeBelief, edge);

      Preconditions.checkArgument(!Double.isNaN(localLogLik));

      edgeToPredictiveBeliefAndLogLikelihood.put(edge,
          new DefaultWeightedValue<MultivariateGaussian>(
              edgeBelief.clone(), localLogLik));

      /*
       * Add likelihood for this edge to the path total
       */
      pathLogLik = LogMath.add(pathLogLik, localLogLik);
      prevEdge = edge;
    }

    return new InferredPathEntry(this, edgeToPredictiveBeliefAndLogLikelihood, 
        filter, pathLogLik);
  }
  
}
