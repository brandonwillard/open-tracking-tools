package org.openplans.tools.tracking.impl;

import java.util.Map;
import java.util.Map.Entry;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.collect.ImmutableList;

public class InferredPathEntry {
  private final StandardRoadTrackingFilter filter;
  private final Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBelief;
  private final InferredPath path;
  private final double totalLogLikelihood;

  public InferredPathEntry(InferredPath path,
    Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> edgeToPredictiveBeliefAndLogLikelihood,
    StandardRoadTrackingFilter filter, double totalLogLikelihood) {
    this.totalLogLikelihood = totalLogLikelihood;
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

  public InferredPath getPath() {
    return this.path;
  }

  public double getTotalLogLikelihood() {
    return totalLogLikelihood;
  }

  @Override
  public String toString() {
    return "InferredPathEntry [path=" + path + ", totalLogLikelihood="
        + totalLogLikelihood + "]";
  }

  public DataDistribution<PathEdge> getNormalizedPathDistribution() {
    final DataDistribution<PathEdge> pathEdgeDist = new DefaultDataDistribution<PathEdge>();

    /*-
     * Normalize to avoid zero probs.
     */
    double totalLikelihood = Double.NEGATIVE_INFINITY;
    for (final DefaultWeightedValue<MultivariateGaussian> weight : this 
        .getEdgeToPredictiveBelief().values()) {
      totalLikelihood = LogMath.add(weight.getWeight(), totalLikelihood);
    }
    
    for (final PathEdge edge : this.getPath().getEdges()) {
      final double weight = this.getEdgeToPredictiveBelief().get(edge)
          .getWeight()
          - totalLikelihood;
      pathEdgeDist.set(edge, Math.exp(weight));
    }
    
    return pathEdgeDist;
  }

}
