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
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.model.WrappedWeightedValue;

import com.google.common.base.Preconditions;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public class VehicleStatePLFilter<O extends GpsObservation, V extends VehicleState<O>>
    extends AbstractParticleFilter<O, V> {

  private long seed;
  private static final long serialVersionUID = -8257075186193062150L;

  private InferenceGraph inferredGraph;
  private Boolean isDebug;

  public VehicleStatePLFilter(O obs, InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    ParticleFilter.Updater<O, V> updater, Boolean isDebug, Random rng) {

    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(updater);
  }

  @Override
  public void update(DataDistribution<V> target, O obs) {

    /*
     * Compute predictive distributions, and create a distribution out of those and
     * their likelihoods for the new observation.
     */
    final DefaultCountedDataDistribution<V> resampleDist =
        new DefaultCountedDataDistribution<V>(true);

    for (final V state : target.getDomain()) {

      final V predictedState =
          (V) state.getBayesianEstimatorPredictor()
              .createPredictiveDistribution(state);

      final double predictiveLogLikelihood =
          predictedState.getProbabilityFunction().logEvaluate(obs);

      resampleDist.increment(predictedState, predictiveLogLikelihood);
    }

    final Random rng = getRandom();

    Preconditions.checkState(!resampleDist.isEmpty());

    /*
     * Resample the predictive distributions.  Now we're dealing with the "best" states.
     */
    final ArrayList<V> smoothedStates =
        resampleDist.sample(rng, getNumParticles());

    target.clear();

    /*
     * Propagate/smooth the best states. 
     */
    for (final V state : smoothedStates) {

      this.getUpdater().update(state);

      target.increment(state, 0d);

    }

  }

}