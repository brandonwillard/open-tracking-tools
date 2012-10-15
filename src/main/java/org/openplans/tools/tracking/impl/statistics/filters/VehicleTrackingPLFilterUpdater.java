package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.apache.commons.lang.NotImplementedException;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.Sets;

public class VehicleTrackingPLFilterUpdater implements
    PLParticleFilterUpdater<Observation, VehicleState> {

  private static class UpdaterThreadLocal extends ThreadLocal<Random> {

    private long seed;

    public UpdaterThreadLocal(long seed) {
      super();
      this.seed = seed;
    }

    @Override
    public Random get() {
      return super.get();
    }

    public long getSeed() {
      return seed;
    }

    @Override
    protected Random initialValue() {
      final Random rng = new Random();
      if (this.seed == 0l) {
        this.seed = rng.nextLong();
      }
      rng.setSeed(this.seed);

      return rng;
    }

    public void setSeed(long nextLong) {
      this.seed = nextLong;
    }

  }

  private static final long serialVersionUID = 2884138088944317656L;
  private final Observation initialObservation;

  private final OtpGraph inferredGraph;

  private final VehicleStateInitialParameters parameters;

  private Random random;

  public VehicleTrackingPLFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    this.random = new Random(parameters.getSeed());
  }

  public VehicleTrackingPLFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    @Nonnull Random rng) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
    this.random = rng;
  }

  @Override
  public VehicleTrackingPLFilterUpdater clone() {
    throw new NotImplementedException();
  }

  /**
   * Evaluate the "point-wise" likelihood, i.e. we don't evaluate a path.
   */
  @Override
  public double computeLogLikelihood(VehicleState particle,
    Observation observation) {
    throw new NotImplementedException();
    //    return particle.getProbabilityFunction().logEvaluate(
    //        new VehicleStateConditionalParams(PathEdge.getEdge(
    //            particle.getInferredEdge(), 0d), observation
    //            .getProjectedPoint(), 0d));
  }

  @Override
  public DataDistribution<VehicleState> createInitialParticles(
    int numParticles) {

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>();

    for (int i = 0; i < numParticles; i++) {
      final StandardRoadTrackingFilter tmpTrackingFilter =
          new StandardRoadTrackingFilter(parameters.getObsCov(),
              parameters.getOffRoadStateCov(),
              parameters.getOnRoadStateCov(),
              parameters.getInitialObsFreq());
      final MultivariateGaussian tmpInitialBelief =
          tmpTrackingFilter.createInitialLearnedObject();
      final Vector xyPoint = initialObservation.getProjectedPoint();
      tmpInitialBelief.setMean(VectorFactory.getDefault().copyArray(
          new double[] { xyPoint.getElement(0), 0d,
              xyPoint.getElement(1), 0d }));
      final List<StreetEdge> initialEdges =
          inferredGraph.getNearbyEdges(tmpInitialBelief,
              tmpTrackingFilter);
  
      final DataDistribution<VehicleState> initialDist =
          new DefaultDataDistribution<VehicleState>(numParticles);
  
      final Set<InferredPathEntry> evaluatedPaths = Sets.newHashSet();
      if (!initialEdges.isEmpty()) {
        for (final Edge nativeEdge : initialEdges) {
          final InferredEdge edge =
              inferredGraph.getInferredEdge(nativeEdge);
          final PathEdge pathEdge = PathEdge.getEdge(edge, 0d, false);
          final InferredPath path =
              InferredPath.getInferredPath(pathEdge);
          evaluatedPaths.add(new InferredPathEntry(path, null, null,
              null, Double.NEGATIVE_INFINITY));
  
          final StandardRoadTrackingFilter trackingFilter =
              new StandardRoadTrackingFilter(
                  parameters.getObsCov(),
                  parameters.getOffRoadStateCov(),
                  parameters.getOnRoadStateCov(),
                  parameters.getInitialObsFreq());
  
          final OnOffEdgeTransDirMulti edgeTransDist =
              new OnOffEdgeTransDirMulti(inferredGraph,
                  parameters.getOnTransitionProbs(),
                  parameters.getOffTransitionProbs());
  
          final VehicleState state =
              new VehicleState(this.inferredGraph, initialObservation,
                  pathEdge.getInferredEdge(), trackingFilter,
                  edgeTransDist, this.random);
  
          /*
           * Sample an initial prior for the transition probabilities
           */
          final Vector edgeDriorParams =
              state.getEdgeTransitionDist()
                  .getEdgeMotionTransProbPrior()
                  .sample(this.random);
          final Vector freeDriorParams =
              state.getEdgeTransitionDist()
                  .getFreeMotionTransProbPrior()
                  .sample(this.random);
          state.getEdgeTransitionDist().getEdgeMotionTransPrior()
              .setParameters(edgeDriorParams);
          state.getEdgeTransitionDist().getFreeMotionTransPrior()
              .setParameters(freeDriorParams);
  
          final double lik =
              state.getProbabilityFunction().evaluate(
                  initialObservation);
  
          initialDist.increment(state, lik);
        }
      }
  
      /*
       * Free-motion
       */
      final StandardRoadTrackingFilter trackingFilter =
          new StandardRoadTrackingFilter(parameters.getObsCov(),
              parameters.getOffRoadStateCov(),
              parameters.getOnRoadStateCov(),
              parameters.getInitialObsFreq());
  
      final OnOffEdgeTransDirMulti edgeTransDist =
          new OnOffEdgeTransDirMulti(inferredGraph,
              parameters.getOnTransitionProbs(),
              parameters.getOffTransitionProbs());
  
      final VehicleState state =
          new VehicleState(this.inferredGraph, initialObservation,
              InferredEdge.getEmptyEdge(), trackingFilter,
              edgeTransDist, this.random);
      /*
       * Sample an initial prior for the transition probabilities
       */
      final Vector edgeDriorParams =
          state.getEdgeTransitionDist().getEdgeMotionTransProbPrior()
              .sample(this.random);
      final Vector freeDriorParams =
          state.getEdgeTransitionDist().getFreeMotionTransProbPrior()
              .sample(this.random);
      state.getEdgeTransitionDist().getEdgeMotionTransPrior()
          .setParameters(edgeDriorParams);
      state.getEdgeTransitionDist().getFreeMotionTransPrior()
          .setParameters(freeDriorParams);
  
      final double lik =
          state.getProbabilityFunction().evaluate(initialObservation);
  
      initialDist.increment(state, lik);
      
      retDist.increment(initialDist.sample(random));
    }

    return retDist;
  }

  public Observation getInitialObservation() {
    return initialObservation;
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedException();
  }

  @Override
  public Random getRandom() {
    return this.random;
  }

  @Override
  public void setRandom(Random rng) {
    this.random = rng;
  }

}