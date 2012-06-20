package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.Map;

import com.google.common.base.Objects;
import com.google.common.collect.ComparisonChain;

public class InferredPathEntry implements
    Comparable<InferredPathEntry> {
  private final StandardRoadTrackingFilter filter;
  private final Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBelief;
  private final InferredPath path;
  private final double totalLogLikelihood;

  public InferredPathEntry(
    InferredPath path,
    Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBeliefAndLogLikelihood,
    StandardRoadTrackingFilter filter, double totalLogLikelihood) {
    this.totalLogLikelihood = totalLogLikelihood;
    this.path = path;
    this.filter = filter;
    this.edgeToPredictiveBelief = edgeToPredictiveBeliefAndLogLikelihood;
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

  public Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> getEdgeToPredictiveBelief() {
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
