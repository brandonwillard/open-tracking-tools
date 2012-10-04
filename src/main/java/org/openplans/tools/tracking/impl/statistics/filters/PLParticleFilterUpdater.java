package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.Random;

public interface PLParticleFilterUpdater<OBS, STATE> extends
    ParticleFilter.Updater<OBS, STATE> {

  public Random getRandom();

  public void setRandom(Random rng);
}
