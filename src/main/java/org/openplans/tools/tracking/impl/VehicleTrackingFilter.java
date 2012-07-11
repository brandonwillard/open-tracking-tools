package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.Random;

import org.openplans.tools.tracking.impl.statistics.FilterInformation;

public interface VehicleTrackingFilter<ObservationType, ParameterType>
    extends ParticleFilter<ObservationType, ParameterType> {

  FilterInformation getFilterInformation(ObservationType observation);

  @Override
  Random getRandom();

}
