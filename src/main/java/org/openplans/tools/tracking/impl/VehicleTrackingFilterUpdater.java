package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Random;
import java.util.Set;

import jj2000.j2k.NotImplementedError;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.base.Objects;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;

public class VehicleTrackingFilterUpdater implements
    ParticleFilter.Updater<Observation, VehicleState> {

  private static final long serialVersionUID = 2884138088944317656L;

  private final Observation initialObservation;
  private final InferredGraph inferredGraph;

  private final InitialParameters parameters;

  private ThreadLocal<Random> threadRandom;

  private static class UpdaterThreadLocal extends ThreadLocal<Random> {
  
    long seed;
    
    public UpdaterThreadLocal(long seed) {
      super();
      this.seed = seed;
    }
    
    @Override
    public Random get() {
      return super.get();
    }

    @Override
    protected Random initialValue() {
      Random rng = new Random();
      if (this.seed == 0l) {
        this.seed = rng.nextLong();
      }
      rng.setSeed(this.seed);
      
      return rng;
    }

    public long getSeed() {
      return seed;
    }

  }
    
  public VehicleTrackingFilterUpdater(Observation obs,
    InferredGraph inferredGraph, InitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    this.threadRandom = new UpdaterThreadLocal(parameters.getSeed());
  }

  @Override
  public VehicleTrackingFilterUpdater clone() {
    throw new RuntimeException("not implemented");
  }

  /**
   * Evaluate the "point-wise" likelihood, i.e. we don't evaluate a path.
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
    return particle.getProbabilityFunction().logEvaluate(
        new VehicleStateConditionalParams(PathEdge.getEdge(particle.getInferredEdge(), 0d), 
            observation.getProjectedPoint(), 0d));
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(int numParticles) {
    /*
     * Create initial distributions for all snapped edges
     */
    
    final List<StreetEdge> initialEdges = inferredGraph.getNarratedGraph()
        .snapToGraph(null, initialObservation.getObsCoords());
    final DataDistribution<VehicleState> initialDist = new DefaultDataDistribution<VehicleState>(
        numParticles);

    if (initialEdges.isEmpty()) {
      
      final VehicleState state = new VehicleState(this.inferredGraph, initialObservation,
          InferredGraph.getEmptyEdge(), parameters);

      final VehicleStateConditionalParams params = 
          new VehicleStateConditionalParams(initialObservation.getProjectedPoint());
      final double lik = state.getProbabilityFunction().evaluate(params);
      initialDist.set(state, lik);
      
    } else {
      for (final Edge nativeEdge : initialEdges) {
        final InferredEdge edge = inferredGraph.getInferredEdge(nativeEdge);
        final PathEdge pathEdge = PathEdge.getEdge(edge, 0d);
        final VehicleState state = new VehicleState(this.inferredGraph, initialObservation, 
            pathEdge.getInferredEdge(), parameters);
  
        final VehicleStateConditionalParams edgeLoc = new VehicleStateConditionalParams(pathEdge,
            initialObservation.getProjectedPoint());
        final double lik = state.getProbabilityFunction().evaluate(edgeLoc);
  
        initialDist.increment(state, lik);
      }
    }

    /*
     * Free-motion
     */
    final VehicleState state = new VehicleState(this.inferredGraph, initialObservation, 
        InferredGraph.getEmptyEdge(), parameters);
    final double lik = state.getProbabilityFunction().evaluate(
        new VehicleStateConditionalParams(initialObservation.getProjectedPoint()));
    initialDist.increment(state, lik);

    final DataDistribution<VehicleState> retDist = new DefaultDataDistribution<VehicleState>(
        initialDist.sample(threadRandom.get(), numParticles));

    return retDist;
  }

  public Observation getInitialObservation() {
    return initialObservation;
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedError();
  }

  public ThreadLocal<Random> getThreadRandom() {
    return threadRandom;
  }

}