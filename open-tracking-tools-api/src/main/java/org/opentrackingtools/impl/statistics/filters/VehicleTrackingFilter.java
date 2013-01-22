package org.opentrackingtools.impl.statistics.filters;

import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.Random;

public interface VehicleTrackingFilter<ObservationType, ParameterType>
    extends ParticleFilter<ObservationType, ParameterType> {

  FilterInformation getFilterInformation(
    ObservationType observation);

  double getLastProcessedTime();

  @Override
  Random getRandom();

}
