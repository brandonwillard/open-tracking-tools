package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.base.Objects;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;

public class VehicleTrackingFilterUpdater implements
    ParticleFilter.Updater<Observation, VehicleState> {

  private static final long serialVersionUID = 2884138088944317656L;

  private final Observation initialObservation;
  private final InferredGraph inferredGraph;

  private static ThreadLocal<Random> threadRandom = new ThreadLocal<Random>() {

    @Override
    public Random get() {
      return super.get();
    }

    @Override
    protected Random initialValue() {
      // TODO add an option for seeding?
      return new Random();
    }

  };

  public VehicleTrackingFilterUpdater(Observation obs,
    InferredGraph inferredGraph) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
  }

  @Override
  public VehicleTrackingFilterUpdater clone() {
    return null;
  }

  /**
   * Evaluate the "point-wise" likelihood, i.e. we don't evaluate a path.
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
    return particle.getProbabilityFunction().logEvaluate(
        new EdgeLocation(observation));
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(int numParticles) {
    /*
     * Create initial distributions for all snapped edges
     */
    
    final SnappedEdges initialEdges = inferredGraph.getNarratedGraph()
        .snapToGraph(null, initialObservation.getObsCoords());
    final DataDistribution<VehicleState> initialDist = new DefaultDataDistribution<VehicleState>(
        numParticles);

    /*
     * TODO FIXME should sample precisions and whatnot
     */
    if (initialEdges.getSnappedEdges().isEmpty()) {
      
      final InferredEdge edge = InferredGraph.getEmptyEdge();
      final VehicleState state = new VehicleState(initialObservation,
          Lists.newArrayList(edge));

      final EdgeLocation edgeLoc = new EdgeLocation(edge,
          initialObservation.getObsPoint(),
          initialObservation.getProjectedPoint(), Lists.newArrayList(edge));
      final double lik = state.getProbabilityFunction().evaluate(edgeLoc);
      initialDist.set(state, lik);
      
    } else {
      for (final Edge nativeEdge : initialEdges.getSnappedEdges()) {
        final InferredEdge edge = inferredGraph.getEdge(nativeEdge);
        final VehicleState state = new VehicleState(initialObservation,
            Lists.newArrayList(edge));
  
        final EdgeLocation edgeLoc = new EdgeLocation(edge,
            initialObservation.getObsPoint(),
            initialObservation.getProjectedPoint(), Lists.newArrayList(edge));
        final double lik = state.getProbabilityFunction().evaluate(edgeLoc);
  
        initialDist.increment(state, lik);
      }
    }

    /*
     * Free-motion
     */
    final VehicleState state = new VehicleState(initialObservation);
    final double lik = state.getProbabilityFunction().evaluate(
        new EdgeLocation(initialObservation));
    initialDist.increment(state, lik);

    final DataDistribution<VehicleState> retDist = new DefaultDataDistribution<VehicleState>(
        initialDist.sample(threadRandom.get(), numParticles));

    return retDist;
  }

  public Observation getInitialObservation() {
    return initialObservation;
  }

  @Deprecated
  @Override
  public VehicleState update(VehicleState previousParameter) {
    /*
     * This method is supposed to sample a proposal state. Start by getting
     * a/the path between edges or free movement. TODO caching?
     */
    final ImmutableList<InferredEdge> path = inferredGraph.getPath(
        previousParameter, previousParameter.getObservation().getObsCoords());

    final Standard2DTrackingFilter filter = previousParameter
        .getMovementFilter().clone();
    final MultivariateGaussian belief = previousParameter.getMovementBelief()
        .clone();
    // InferredEdge prevEdge = null;
    for (final InferredEdge edge : path) {

      /*
       * Do something special? Like sample
       */
      // if (edge == InferredGraph.getEmptyEdge()) {
      //
      // } else {
      //
      // }

      filter.constrainMotion(edge.getAngle());
      filter.predict(belief);

      // prevEdge = edge;
    }

    return new VehicleState(previousParameter.getObservation(), filter, belief,
        previousParameter.getEdgeTransitionDist(), path);
  }

  public static ThreadLocal<Random> getThreadRandom() {
    return threadRandom;
  }

}