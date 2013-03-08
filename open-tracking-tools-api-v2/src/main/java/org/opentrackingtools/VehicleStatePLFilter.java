package org.opentrackingtools;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.distributions.DefaultCountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.updater.VehicleTrackingPLFilterUpdater;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.model.WrappedWeightedValue;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public class VehicleStatePLFilter<O extends GpsObservation>
    extends AbstractParticleFilter<O, VehicleState<O>> {

  private static final long serialVersionUID = -8257075186193062150L;

  private InferenceGraph inferredGraph;
  private Boolean isDebug;

  public VehicleStatePLFilter(O obs, InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters, Boolean isDebug, Random rng) {
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleTrackingPLFilterUpdater<O>(obs, inferredGraph, parameters, rng));
  }

  @Override
  public void update(DataDistribution<VehicleState<O>> target, O obs) {

    /*
     * Compute predictive distributions, and create a distribution out of those and
     * their likelihoods for the new observation.
     */
    final DefaultCountedDataDistribution<VehicleState<O>> resampleDist =
        new DefaultCountedDataDistribution<VehicleState<O>>(true);

    for (final VehicleState<O> state : target.getDomain()) {

      final VehicleState<O> predictedState = new VehicleState<O>(state);
      predictedState.setObservation(obs);
      this.updater.update(predictedState);

      final double predictiveLogLikelihood =
          this.getUpdater().computeLogLikelihood(predictedState, obs);

      resampleDist.increment(predictedState, predictiveLogLikelihood);
    }

    final Random rng = getRandom();

    Preconditions.checkState(!resampleDist.isEmpty());

    /*
     * Resample the predictive distributions.  Now we're dealing with the "best" states.
     */
    final ArrayList<VehicleState<O>> smoothedStates =
        resampleDist.sample(rng, getNumParticles());

    target.clear();

    /*
     * Propagate/smooth the best states. 
     */
    for (final VehicleState<O> state : smoothedStates) {

      target.increment(state, 0d);

    }

  }

}