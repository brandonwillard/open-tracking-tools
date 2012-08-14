package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.util.Pair;

import java.util.List;
import java.util.Map;

import org.openplans.tools.tracking.impl.graph.paths.InferredPath.EdgePredictiveResults;
import org.openplans.tools.tracking.impl.statistics.WrappedWeightedValue;
import org.openplans.tools.tracking.impl.statistics.filters.StandardRoadTrackingFilter;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;

public class InferredPathEntry implements
    Comparable<InferredPathEntry> {

  private final StandardRoadTrackingFilter filter;

  /*
   * This maps edge's to their conditional prior predictive location/velocity states.
   */
  private final Map<Pair<PathEdge, Boolean>, EdgePredictiveResults> edgeToPredictiveBelief;

  private final InferredPath path;

  private final double totalLogLikelihood;

  private final List<WrappedWeightedValue<PathEdge>> weightedPathEdges;

  public InferredPathEntry(
    InferredPath path,
    Map<Pair<PathEdge, Boolean>, EdgePredictiveResults> edgeToPreBeliefAndLogLik,
    StandardRoadTrackingFilter filter,
    List<WrappedWeightedValue<PathEdge>> weightedPathEdges,
    double totalLogLikelihood) {
    Preconditions.checkArgument(!Double.isNaN(totalLogLikelihood));
    this.totalLogLikelihood = totalLogLikelihood;
    this.path = path;
    this.filter = filter;
    this.edgeToPredictiveBelief = edgeToPreBeliefAndLogLik;
    this.weightedPathEdges = weightedPathEdges;
  }

  @Override
  public int compareTo(InferredPathEntry o) {
    return ComparisonChain.start().compare(this.path, o.path)
        .result();
  }

  @Override
  public boolean equals(Object object) {
    if (object instanceof InferredPathEntry) {
      if (!super.equals(object))
        return false;
      final InferredPathEntry that = (InferredPathEntry) object;
      return Objects.equal(this.path, that.path);
    }
    return false;
  }

  public Map<Pair<PathEdge, Boolean>, EdgePredictiveResults>
      getEdgeToPredictiveBelief() {
    return edgeToPredictiveBelief;
  }

  public StandardRoadTrackingFilter getFilter() {
    return filter;
  }

  public InferredPath getPath() {
    return this.path;
  }

  public double getTotalLogLikelihood() {
    return totalLogLikelihood;
  }

  public List<WrappedWeightedValue<PathEdge>> getWeightedPathEdges() {
    return this.weightedPathEdges;
  }

  @Override
  public int hashCode() {
    return Objects.hashCode(super.hashCode(), path);
  }

  @Override
  public String toString() {
    return "InferredPathEntry [path=" + path
        + ", totalLogLikelihood=" + totalLogLikelihood + "]";
  }

}
