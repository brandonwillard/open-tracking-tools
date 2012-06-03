package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.ArrayList;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;

public class VehicleTrackingFilter extends
    AbstractParticleFilter<Observation, VehicleState> {

  private static final long serialVersionUID = -8257075186193062150L;

  /*
   * Populate this when you want a generalized graph. TODO Otherwise, one is
   * created for each particle.
   */
  private final InferredGraph inferredGraph;

  private double prevTime = 0;

  public VehicleTrackingFilter(Observation obs, InferredGraph inferredGraph,
    InitialParameters parameters) {
    this.setNumParticles(50);
    this.inferredGraph = inferredGraph;
    this.setUpdater(new VehicleTrackingFilterUpdater(obs, this.inferredGraph,
        parameters));
  }
  
  @Override
  public Random getRandom() {
    VehicleTrackingFilterUpdater updater = (VehicleTrackingFilterUpdater) this.getUpdater();
    return updater.getThreadRandom().get();
  }

  /**
   * Note: this skips observations with a time delta of zero or less.
   */
  @Override
  public void update(DataDistribution<VehicleState> target, Observation obs) {

    final double timeDiff = prevTime == 0 ? 0
        : (obs.getTimestamp().getTime() - prevTime) / 1000;
    prevTime = obs.getTimestamp().getTime();

    if (timeDiff <= 0)
      return;
    // Preconditions.checkArgument(timeDiff > 0, "timeDiff=" + timeDiff);

    final Map<VehicleState, DataDistribution<InferredPathEntry>> instStateTransMap = Maps
        .newHashMap();

    /*
     * Resample based on predictive likelihood to get a smoothed sample
     */
    final DataDistribution<VehicleState> resampler = new DefaultDataDistribution<VehicleState>(
        target.size());
    for (final VehicleState state : target.getDomain()) {

      /*
       * Get all possible inst. state transitions. TODO FIXME Currently there is
       * only one: the "shortest-path" between edges.
       */
      final DataDistribution<InferredPathEntry> allInstStateTransitions = new DefaultDataDistribution<InferredPathEntry>();
      final Set<InferredPath> instStateTransitions = inferredGraph.getPaths(
          state, obs.getObsCoords());


      state.getMovementFilter().setCurrentTimeDiff(timeDiff);
      final StandardRoadTrackingFilter filter = state.getMovementFilter()
          .clone();


      double totalLogLik = Double.NEGATIVE_INFINITY;
      for (final InferredPath path : instStateTransitions) {

        /*-
         * Produce the distance prediction.
         * Note: here the road beliefs start at 0
         */
        final MultivariateGaussian beliefPrediction = state.getBelief().clone();
        final PathEdge firstEdge = path.getEdges().get(0);
        filter.predict(beliefPrediction, firstEdge, PathEdge.getEdge(state.getEdge(), 0d));
        
        /*-
         * Compute predictive dist. over path
         * Note that this path should always start with the edge that
         * this state is currently on.
         */
        PathEdge prevEdge = PathEdge.getEdge(state.getInferredEdge());
        double pathLogLik = Double.NEGATIVE_INFINITY;
        final Map<PathEdge, DefaultWeightedValue<MultivariateGaussian>> 
          edgeToPredictiveBeliefAndLogLikelihood = Maps.newHashMap();
        
        for (final PathEdge edge : path.getEdges()) {

          /*
           * If we're going off-road, then pass the edge we used to be on.
           */
          final MultivariateGaussian edgeBelief = beliefPrediction.clone();
          if (edge == PathEdge.getEmptyPathEdge()) {
            filter.predict(edgeBelief, edge, prevEdge);
          } else {
            edge.predict(edgeBelief);
          }

          // TODO should we use cumulative transition?
          double localLogLik = state.getEdgeTransitionDist()
              .predictiveLogLikelihood(prevEdge.getInferredEdge(),
                  edge.getInferredEdge());
          localLogLik += filter.logLikelihood(obs.getProjectedPoint(),
              edgeBelief, edge);

          Preconditions.checkArgument(!Double.isNaN(localLogLik));

          edgeToPredictiveBeliefAndLogLikelihood.put(edge,
              new DefaultWeightedValue<MultivariateGaussian>(
                  edgeBelief.clone(), localLogLik));

          /*
           * Add likelihood for this edge to the path total
           */
          pathLogLik = LogMath.add(pathLogLik, localLogLik);
          prevEdge = edge;
        }

        /*
         * Add likelihood for this path to the source state's total
         */
        totalLogLik = LogMath.add(totalLogLik, pathLogLik);

        final InferredPathEntry infPath = new InferredPathEntry(path,
            edgeToPredictiveBeliefAndLogLikelihood, filter);

        allInstStateTransitions.increment(infPath, Math.exp(pathLogLik));
      }
      resampler.increment(state, Math.exp(totalLogLik));
      instStateTransMap.put(state, allInstStateTransitions);
    }

    final Random rng = getRandom();

    @SuppressWarnings("unchecked")
    final ArrayList<? extends VehicleState> smoothedStates = resampler.sample(
        rng, getNumParticles());

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
      final DataDistribution<InferredPathEntry> instStateDist = instStateTransMap
          .get(state);
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
        final DataDistribution<PathEdge> pathEdgeDist = new DefaultDataDistribution<PathEdge>();

        /*-
         * Normalize to avoid zero probs.
         * TODO put this in PathEntry
         */
        double totalLikelihood = Double.NEGATIVE_INFINITY;
        for (final DefaultWeightedValue<MultivariateGaussian> weight : pathEntry
            .getEdgeToPredictiveBelief().values()) {
          totalLikelihood = LogMath.add(weight.getWeight(), totalLikelihood);
        }
        for (final PathEdge edge : pathEntry.getPath().getEdges()) {
          final double weight = pathEntry.getEdgeToPredictiveBelief().get(edge)
              .getWeight()
              - totalLikelihood;
          pathEdgeDist.set(edge, Math.exp(weight));
        }

        sampledEdge = pathEdgeDist.sample(rng);
      } else {
        sampledEdge = pathEntry.getPath().getEdges().get(0);
      }
      final MultivariateGaussian sampledBelief = pathEntry
          .getEdgeToPredictiveBelief().get(sampledEdge).getValue().clone();

      final VehicleState newState = new VehicleState(this.inferredGraph, obs,
          filter, sampledBelief, edgeTransDist, sampledEdge,
          pathEntry.getPath(), state);
      target.set(newState, 1d / getNumParticles());

      /*-
       * Propagate sufficient stats (can be done off-line) Just the edge
       * transitions for now.
       * TODO FIXME must make sure the point is projected onto an edge
       */
      filter.measure(sampledBelief, obs.getProjectedPoint(), sampledEdge);
      InferredEdge prevEdge = pathEntry.getPath().getEdges().get(0)
          .getInferredEdge();
      for (final PathEdge edge : pathEntry.getPath().getEdges()) {
        if (prevEdge != null)
          edgeTransDist.update(prevEdge, edge.getInferredEdge());
        prevEdge = edge.getInferredEdge();
      }

    }

  }

}
