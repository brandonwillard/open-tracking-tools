package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public class VehicleTrackingFilter extends
    AbstractParticleFilter<Observation, VehicleState> {

  private static final long serialVersionUID = -8257075186193062150L;

  /*
   * Populate this when you want a generalized graph. TODO Otherwise, one is
   * created for each particle.
   */
  private final InferredGraph inferredGraph;

  private double prevTime = 0;

  public VehicleTrackingFilter(Observation obs,
    InferredGraph inferredGraph, InitialParameters parameters) {
    this.setNumParticles(50);
    this.inferredGraph = inferredGraph;
    this.setUpdater(new VehicleTrackingFilterUpdater(
        obs, this.inferredGraph, parameters));
  }

  @Override
  public Random getRandom() {
    final VehicleTrackingFilterUpdater updater = (VehicleTrackingFilterUpdater) this
        .getUpdater();
    return updater.getThreadRandom().get();
  }

  /**
   * Note: this skips observations with a time delta of zero or less.
   */
  @Override
  public void update(DataDistribution<VehicleState> target,
    Observation obs) {

    final double timeDiff = prevTime == 0 ? 0 : (obs.getTimestamp()
        .getTime() - prevTime) / 1000;
    prevTime = obs.getTimestamp().getTime();

    if (timeDiff <= 0)
      return;
    // Preconditions.checkArgument(timeDiff > 0, "timeDiff=" + timeDiff);

    final Multimap<VehicleState, DefaultWeightedValue<InferredPathEntry>> stateToPaths = HashMultimap
        .create();
    final Set<InferredPathEntry> evaluatedPaths = Sets.newHashSet();

    /*
     * Resample based on predictive likelihood to get a smoothed sample
     */
    final List<DefaultWeightedValue<VehicleState>> resampler = Lists
        .newArrayList();
    for (final VehicleState state : target.getDomain()) {

      state.getBelief().clone();

      /*
       * Take the prediction step
       */
      state.getMovementFilter().predict(
          state.getBelief(),
          PathEdge.getEdge(state.getInferredEdge()),
          PathEdge.getEdge(state.getInferredEdge(), 0d));

      final Set<InferredPath> instStateTransitions = inferredGraph
          .getPaths(state, obs.getObsCoords());

      state.getMovementFilter().setCurrentTimeDiff(timeDiff);
      double totalLogLik = Double.NEGATIVE_INFINITY;

      for (final InferredPath path : instStateTransitions) {
        final InferredPathEntry infPath = path
            .getPredictiveLogLikelihood(obs, state);
        evaluatedPaths.add(infPath);
        totalLogLik = LogMath.add(
            totalLogLik, infPath.getTotalLogLikelihood());
        stateToPaths.put(
            state, new DefaultWeightedValue<InferredPathEntry>(
                infPath, infPath.getTotalLogLikelihood()));
      }

      resampler.add(new DefaultWeightedValue<VehicleState>(
          state, totalLogLik));
    }

    final Random rng = getRandom();

    @SuppressWarnings("unchecked")
    final DataDistribution<VehicleState> resampleDist = StatisticsUtil
        .getLogNormalizedDistribution(resampler);
    final ArrayList<? extends VehicleState> smoothedStates = resampleDist
        .sample(rng, getNumParticles());

    // TODO FIXME this really sucks, but due to the interface we have to
    // alter the argument map
    target.clear();

    /*
     * Propagate states
     */
    for (final VehicleState state : smoothedStates) {

      /*-
       * First, the inst. state. TODO unfortunately, there isn't much of
       * anything to sample yet.
       */
      final DataDistribution<InferredPathEntry> instStateDist = StatisticsUtil
          .getLogNormalizedDistribution(stateToPaths.get(state));
      final InferredPathEntry pathEntry = instStateDist.sample(rng);

      /*-
       * Now, if you need to, propagate/sample a predictive location state. 
       * TODO don't need to now, but will when estimating state covariance/precision
       * parameters
       */

      /*
       * State suffient stats are next (e.g. kalman params)
       */

      final StandardRoadTrackingFilter filter = pathEntry.getFilter();
      final EdgeTransitionDistributions edgeTransDist = state
          .getEdgeTransitionDist().clone();

      /*
       * Sample the edge we're on.
       */
      final PathEdge sampledEdge;
      if (pathEntry.getPath().getEdges().size() > 1) {
        final DataDistribution<PathEdge> pathEdgeDist = StatisticsUtil
            .getLogNormalizedDistribution(pathEntry
                .getEdgeToPredictiveBelief());
        sampledEdge = pathEdgeDist.sample(rng);
      } else {
        sampledEdge = pathEntry.getPath().getEdges().get(0);
      }
      final MultivariateGaussian sampledBelief = pathEntry
          .getEdgeToPredictiveBelief().get(sampledEdge).getValue()
          .clone();

      @SuppressWarnings("unchecked")
      final FilterInformation info = new FilterInformation(
          pathEntry.getPath(), evaluatedPaths);
      
      final VehicleState newState = new VehicleState(
          this.inferredGraph, obs, filter, sampledBelief,
          edgeTransDist, sampledEdge, state, info);
      
      target.set(newState, 1d / getNumParticles());

      /*-
       * Propagate sufficient stats (can be done off-line) Just the edge
       * transitions for now.
       * TODO FIXME must make sure the point is projected onto an edge
       */
      filter.measure(
          sampledBelief, obs.getProjectedPoint(), sampledEdge);
      
      InferredEdge prevEdge = pathEntry.getPath().getEdges().get(0)
          .getInferredEdge();
      for (final PathEdge edge : pathEntry.getPath().getEdges()) {
        if (prevEdge != null)
          edgeTransDist.update(prevEdge, edge.getInferredEdge());
        
        if (edge != PathEdge.getEmptyPathEdge()) {
          edge.getInferredEdge().getVelocityEstimator().update(
              edge.getInferredEdge().getVelocityPrecisionDist(), 
              sampledBelief.getMean().getElement(1));
        }
        
        prevEdge = edge.getInferredEdge();
      }

    }

  }

}
