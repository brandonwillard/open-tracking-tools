package org.opentrackingtools.statistics.distributions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.statistics.estimators.vehicles.impl.AbstractRoadTrackingEstimator;

public interface PathStateDistribution extends PathState, Cloneable {

  public abstract Matrix getCovariance();

  public abstract MultivariateGaussian getGlobalStateBelief();

  public abstract MultivariateGaussian getLocalStateBelief();

  public abstract MultivariateGaussian getRawStateBelief();

  public abstract double logLikelihood(Vector obs,
    AbstractRoadTrackingEstimator filter);

  /**
   * Returns the log likelihood for the
   * {@link AbstractRoadTrackingEstimator#getObservationBelief(MultivariateGaussian, SimplePathEdge)
   * observation belief} .
   * 
   * @param obs
   * @param belief
   * @return
   */
  public abstract double priorPredictiveLogLikelihood(Vector obs,
    AbstractRoadTrackingEstimator filter);

  public abstract MultivariateGaussian getGroundBelief();

  public abstract PathStateDistribution getTruncatedPathStateBelief();

  @Override
  public abstract PathStateDistribution clone();
}