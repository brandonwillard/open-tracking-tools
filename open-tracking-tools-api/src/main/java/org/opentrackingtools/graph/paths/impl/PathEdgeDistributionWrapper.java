package org.opentrackingtools.graph.paths.impl;

import java.util.List;
import java.util.Map;

import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.EdgePredictiveResults;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.statistics.distributions.impl.DefaultCountedDataDistribution;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;

public class PathEdgeDistributionWrapper implements
    Comparable<PathEdgeDistributionWrapper> {

  private final AbstractRoadTrackingFilter filter;

  /*
   * This maps edge's to their conditional prior predictive location/velocity states.
   */
  private final Map<PathEdge, EdgePredictiveResults> edgeToPredictiveBelief;

  private final InferredPath path;

  private final double totalLogLikelihood;

  private final DefaultCountedDataDistribution<PathEdge> pathEdgeDistribution;

  public PathEdgeDistributionWrapper(
    InferredPath path,
    Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik,
    AbstractRoadTrackingFilter filter,
    DefaultCountedDataDistribution<PathEdge> pathEdgeDistribution,
    double totalLogLikelihood) {
    Preconditions.checkArgument(!Double
        .isNaN(totalLogLikelihood));
    this.totalLogLikelihood = totalLogLikelihood;
    this.path = path;
    this.filter = filter;
    this.edgeToPredictiveBelief = edgeToPreBeliefAndLogLik;
    this.pathEdgeDistribution = pathEdgeDistribution;
  }

  @Override
  public int compareTo(PathEdgeDistributionWrapper o) {
    return ComparisonChain.start()
        .compare(this.path, o.path).result();
  }

  @Override
  public boolean equals(Object object) {
    if (object instanceof PathEdgeDistributionWrapper) {
      if (!super.equals(object))
        return false;
      final PathEdgeDistributionWrapper that =
          (PathEdgeDistributionWrapper) object;
      return Objects.equal(this.path, that.path);
    }
    return false;
  }

  public Map<PathEdge, EdgePredictiveResults>
      getEdgeToPredictiveBelief() {
    return edgeToPredictiveBelief;
  }

  public AbstractRoadTrackingFilter getFilter() {
    return filter;
  }

  public InferredPath getPath() {
    return this.path;
  }

  public double getTotalLogLikelihood() {
    return totalLogLikelihood;
  }

  public DefaultCountedDataDistribution<PathEdge>
      getPathEdgeDisribution() {
    return this.pathEdgeDistribution;
  }

  @Override
  public int hashCode() {
    return Objects.hashCode(super.hashCode(), path);
  }

  @Override
  public String toString() {
    return "InferredPathEntry [path=" + path
        + ", totalLogLikelihood=" + totalLogLikelihood
        + "]";
  }

}
