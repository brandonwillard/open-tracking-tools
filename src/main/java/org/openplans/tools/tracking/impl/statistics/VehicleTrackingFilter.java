package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Multimap;
import com.google.common.collect.Sets;

public class VehicleTrackingFilter extends
    AbstractParticleFilter<Observation, VehicleState> {

  private static final long serialVersionUID = -8257075186193062150L;

  /*
   * Populate this when you want a generalized graph. TODO Otherwise, one is
   * created for each particle.
   */
  private final OtpGraph inferredGraph;

  private double prevTime = 0;

  private DataDistribution<VehicleState> previousResampleDist;

  private final Map<Observation, FilterInformation> filterInfo = Maps
      .newHashMap();

  private final boolean isDebug;

  private final Observation initialObservation;

  public VehicleTrackingFilter(Observation obs,
		  OtpGraph inferredGraph, InitialParameters parameters,
    boolean isDebug) {
    this.isDebug = isDebug;
    this.setNumParticles(50);
    this.inferredGraph = inferredGraph;
    this.setUpdater(new VehicleTrackingFilterUpdater(
        obs, this.inferredGraph, parameters));
    this.initialObservation = obs;
  }

  @Override
  public DataDistribution<VehicleState> createInitialLearnedObject() {
    final DataDistribution<VehicleState> dist = super
        .createInitialLearnedObject();
    if (isDebug) {
      final Set<InferredPath> evaledPaths = Sets.newHashSet();
      for (final VehicleState state : dist.getDomain()) {
        // TODO FIXME provide real info here
        evaledPaths
            .add(state.getPath());
      }
      this.filterInfo.put(initialObservation, new FilterInformation(
          evaledPaths, dist));
    }
    return dist;
  }

  public FilterInformation getFilterInformation(Observation obs) {
    return this.filterInfo.get(obs);
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
    final Set<InferredPath> evaluatedPaths = Sets.newHashSet();

    /*
     * Resample based on predictive likelihood to get a smoothed sample
     */
    final List<DefaultWeightedValue<VehicleState>> resampler = Lists
        .newArrayList();
    for (final VehicleState state : target.getDomain()) {

      final Set<InferredPath> instStateTransitions = inferredGraph
          .getPaths(state, obs.getObsCoords());

      state.getMovementFilter().setCurrentTimeDiff(timeDiff);
      double totalLogLik = Double.NEGATIVE_INFINITY;

      for (final InferredPath path : instStateTransitions) {
        final InferredPathEntry infPath = path
            .getPredictiveLogLikelihood(obs, state);

        if (isDebug)
          evaluatedPaths.add(path);

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

    final DataDistribution<VehicleState> resampleDist = StatisticsUtil
        .getLogNormalizedDistribution(resampler);
    @SuppressWarnings("unchecked")
    final ArrayList<? extends VehicleState> smoothedStates = resampleDist
        .sample(rng, getNumParticles());

    target.clear();
    if (isDebug)
      this.filterInfo.put(obs, new FilterInformation(
          evaluatedPaths, resampleDist));

    /*
     * Propagate states
     */
    for (final VehicleState state : smoothedStates) {

      final VehicleState newState = state.clone();
      final DataDistribution<InferredPathEntry> instStateDist = StatisticsUtil
          .getLogNormalizedDistribution(Lists
              .newArrayList(stateToPaths.get(newState)));
      final InferredPathEntry sampledPathEntry = instStateDist
          .sample(rng);

      /*-
       * Now, if you need to, propagate/sample a predictive location state. 
       * TODO don't need to now, but will when estimating state covariance/precision
       * parameters
       */

      /*
       * State suffient stats are next (e.g. kalman params)
       */

      /*
       * Sample the edge we're on.
       */
      final PathEdge sampledEdge;
      if (sampledPathEntry.getPath().getEdges().size() > 1) {
        final DataDistribution<PathEdge> pathEdgeDist = StatisticsUtil
            .getLogNormalizedDistribution(sampledPathEntry
                .getEdgeToPredictiveBelief());
        sampledEdge = pathEdgeDist.sample(rng);
      } else {
        sampledEdge = sampledPathEntry.getPath().getEdges().get(0);
      }
      final MultivariateGaussian sampledBelief = sampledPathEntry
          .getEdgeToPredictiveBelief().get(sampledEdge).getValue()
          .clone();

      /*-
       * Propagate sufficient stats (can be done off-line) Just the edge
       * transitions for now.
       */
      final StandardRoadTrackingFilter updatedFilter = sampledPathEntry
          .getFilter().clone();
      updatedFilter.measure(
          sampledBelief, obs.getProjectedPoint(), sampledEdge);

      InferredEdge prevEdge = sampledPathEntry.getPath().getEdges()
          .get(0).getInferredEdge();
      final EdgeTransitionDistributions updatedEdgeTransDist = newState
          .getEdgeTransitionDist().clone();
      for (final PathEdge edge : sampledPathEntry.getPath()
          .getEdges()) {
        if (prevEdge != null)
          updatedEdgeTransDist.update(
              prevEdge, edge.getInferredEdge());

        if (!edge.isEmptyEdge()) {
          edge
              .getInferredEdge()
              .getVelocityEstimator()
              .update(
                  edge.getInferredEdge().getVelocityPrecisionDist(),
                  sampledBelief.getMean().getElement(1));
        }

        prevEdge = edge.getInferredEdge();
      }

      final VehicleState newTransState = new VehicleState(
          this.inferredGraph, obs, updatedFilter, sampledBelief,
          updatedEdgeTransDist, sampledEdge,
          sampledPathEntry.getPath(), state);

      target.set(newTransState, 1d / smoothedStates.size());

    }
  }

}
