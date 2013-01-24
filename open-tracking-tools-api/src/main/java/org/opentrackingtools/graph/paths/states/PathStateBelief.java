package org.opentrackingtools.graph.paths.states;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;

public interface PathStateBelief extends PathState, Cloneable {

  public abstract Matrix getCovariance();

  public abstract MultivariateGaussian getGlobalStateBelief();

  public abstract MultivariateGaussian getLocalStateBelief();

  public abstract MultivariateGaussian getRawStateBelief();

  public abstract double logLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter);

  /**
   * Returns the log likelihood for the
   * {@link AbstractRoadTrackingFilter#getObservationBelief(MultivariateGaussian, SimplePathEdge)
   * observation belief} .
   * 
   * @param obs
   * @param belief
   * @return
   */
  public abstract double priorPredictiveLogLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter);

  public abstract MultivariateGaussian getGroundBelief();

  public abstract PathStateBelief getTruncatedPathStateBelief();

  public abstract PathStateBelief clone();
}