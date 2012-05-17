package org.openplans.tools.tracking.impl;

import java.util.Map;
import java.util.Map.Entry;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.collect.ImmutableList;

public class InferredPathEntry {
  private final StandardRoadTrackingFilter filter;
  private final Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBelief;
  private final InferredPath path;

  public InferredPathEntry(InferredPath path,
    Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBeliefAndLogLikelihood,
    StandardRoadTrackingFilter filter) {
    this.path = path;
    this.filter = filter;
    this.edgeToPredictiveBelief = edgeToPredictiveBeliefAndLogLikelihood;
  }

  public StandardRoadTrackingFilter getFilter() {
    return filter;
  }

  public Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> getEdgeToPredictiveBelief() {
    return edgeToPredictiveBelief;
  }

  @Override
  public String toString() {
    return "InferredPathEntry [filter=" + filter + ", edgeToPredictiveBelief="
        + edgeToPredictiveBelief + "]";
  }

  public InferredPath getPath() {
    return this.path;
  }

}
