package org.opentrackingtools.statistics.filters.vehicles;

import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.Random;

import org.opentrackingtools.statistics.filters.vehicles.impl.FilterInformation;

public interface VehicleTrackingFilter<ObservationType, ParameterType>
    extends ParticleFilter<ObservationType, ParameterType> {

  FilterInformation getFilterInformation(
    ObservationType observation);

  double getLastProcessedTime();

  @Override
  Random getRandom();

}
