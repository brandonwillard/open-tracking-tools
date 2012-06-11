package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.Map;

public class InferredPathEntry {
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
    final InferredPathEntry other = (InferredPathEntry) obj;
    if (path == null) {
      if (other.path != null) {
        return false;
      }
    } else if (!path.equals(other.path)) {
      return false;
    }
    return true;
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
    final int prime = 31;
    int result = 1;
    result = prime * result + ((path == null) ? 0 : path.hashCode());
    return result;
  }

  @Override
  public String toString() {
    return "InferredPathEntry [path=" + path
        + ", totalLogLikelihood=" + totalLogLikelihood + "]";
  }

}
