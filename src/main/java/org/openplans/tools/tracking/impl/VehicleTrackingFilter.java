package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Maps;
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

  public VehicleTrackingFilter(Observation obs, InferredGraph inferredGraph) {
    this.setNumParticles(50);
    this.inferredGraph = inferredGraph;
    this.setUpdater(new VehicleTrackingFilterUpdater(obs, this.inferredGraph));
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

    final Map<VehicleState, DataDistribution<InferredPath>> instStateTransMap = Maps
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
      final DataDistribution<InferredPath> allInstStateTransitions = new DefaultDataDistribution<InferredPath>();
      final Set<ImmutableList<InferredEdge>> instStateTransitions = Sets
          .newHashSet();
      final ImmutableList<InferredEdge> thePath = inferredGraph.getPath(state,
          obs.getObsCoords());
      instStateTransitions.add(thePath);
      double totalLik = 0d;
      for (final ImmutableList<InferredEdge> path : instStateTransitions) {
        final MultivariateGaussian belief = state.getMovementBelief().clone();
        final Standard2DTrackingFilter filter = state.getMovementFilter()
            .clone();
        filter.predict(belief);
        InferredEdge prevEdge = null;
        double localLogLik = 0d;
        /*
         * Compute predictive dist. over path
         */
        for (final InferredEdge edge : path) {
          filter.constrainMotion(edge.getAngle());
          filter.predict(belief);
          localLogLik += state.getEdgeTransitionDist().predictiveLikelihood(prevEdge, edge);

          prevEdge = edge;
        }
        localLogLik = filter.predictiveLikelihood(obs.getProjectedPoint(), belief);
        totalLik += Math.exp(localLogLik);

        final InferredPath infPath = new InferredPath(path, belief, filter);
        allInstStateTransitions.increment(infPath, Math.exp(localLogLik));
      }
      resampler.increment(state, Math.exp(totalLik));
      instStateTransMap.put(state, allInstStateTransitions);
    }
    final Random rng = VehicleTrackingFilterUpdater.getThreadRandom().get();

    @SuppressWarnings("unchecked")
    final List<VehicleState> smoothedStates = (List<VehicleState>) resampler
        .sample(rng, getNumParticles());

    // TODO FIXME this really sucks, but due to the interface we have to
    // alter the argument map
    target.clear();

    /*
     * Propagate states
     */
    for (final VehicleState state : smoothedStates) {

      /*
       * First, the inst. state. TODO unfortunately, there isn't much of
       * anything to sample yet.
       */
      final DataDistribution<InferredPath> instStateDist = instStateTransMap
          .get(state);
      final InferredPath path = instStateDist.sample(rng);

      /*
       * Now, if you need to, propagate/sample a predictive location state. TODO
       * don't need to now, but will when estimating state covariance/precision
       * parameters
       */

      /*
       * State suffient stats are next (e.g. kalman params)
       */
      final MultivariateGaussian belief = path.getBelief();
      final Standard2DTrackingFilter filter = path.getFilter();
      final EdgeTransitionDistributions edgeTransDist = state
          .getEdgeTransitionDist().clone();

      final VehicleState newState = new VehicleState(obs, filter, belief,
          edgeTransDist, path.getPath());
      target.set(newState, 1d / getNumParticles());

      /*
       * Propagate sufficient stats (can be done off-line) Just the edge
       * transitions for now.
       */
      filter.update(belief, obs.getProjectedPoint());
      InferredEdge prevEdge = null;
      for (final InferredEdge edge : path.getPath()) {
        if (prevEdge != null)
          edgeTransDist.update(prevEdge, edge);
        prevEdge = edge;
      }

    }

  }

}
