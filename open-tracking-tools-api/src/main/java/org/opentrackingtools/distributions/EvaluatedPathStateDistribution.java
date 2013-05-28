package org.opentrackingtools.distributions;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opentrackingtools.paths.Path;

public class EvaluatedPathStateDistribution extends PathStateDistribution {
  
  protected double obsLogLikelihood = Double.NEGATIVE_INFINITY;
  protected double edgeLogLikelihood = Double.NEGATIVE_INFINITY;
  protected double edgeTransitionLogLikelihood = Double.NEGATIVE_INFINITY;

  public EvaluatedPathStateDistribution(Path path, MultivariateGaussian dist) {
    super(path, dist);
  }

  public EvaluatedPathStateDistribution(
      PathStateDistribution pathStateDistribution) {
    super(pathStateDistribution);
  }

  public double getObsLogLikelihood() {
    return obsLogLikelihood;
  }

  public void setObsLogLikelihood(double obsLogLikelihood) {
    this.obsLogLikelihood = obsLogLikelihood;
  }

  public double getEdgeLogLikelihood() {
    return edgeLogLikelihood;
  }

  public void setEdgeLogLikelihood(double edgeLogLikelihood) {
    this.edgeLogLikelihood = edgeLogLikelihood;
  }

  public double getEdgeTransitionLogLikelihood() {
    return edgeTransitionLogLikelihood;
  }

  public void setEdgeTransitionLogLikelihood(double edgeTransitionLogLikelihood) {
    this.edgeTransitionLogLikelihood = edgeTransitionLogLikelihood;
  }

}
