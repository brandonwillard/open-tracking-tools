package org.openplans.tools.tracking.impl.statistics.filters.particle_learning;

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

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.filters.AbstractVehicleTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPathSamplerFilterUpdater;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

import com.google.common.collect.Sets;

public abstract class AbstractVTParticleFilterUpdater 
  implements ParticleFilter.Updater<Observation, VehicleState> {
  
  private static final long serialVersionUID = 7567157323292175525L;

  protected Observation initialObservation;

  protected OtpGraph inferredGraph;

  protected VehicleStateInitialParameters parameters;

  protected Random random;

  public long seed;

  public AbstractVTParticleFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
  }

  @Override
  public AbstractVTParticleFilterUpdater clone() {
    try {
      final AbstractVTParticleFilterUpdater clone =
          (AbstractVTParticleFilterUpdater) super.clone();
      clone.seed = seed;
      clone.inferredGraph = inferredGraph;
      clone.initialObservation = initialObservation;
      clone.parameters = clone.parameters.clone();
      clone.random = random;
      return clone;
    } catch (final CloneNotSupportedException e) {
      e.printStackTrace();
    }
    return null;
  }

  @Nonnull
  @Override
  public DataDistribution<VehicleState> createInitialParticles(
    int numParticles) {

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>();

    for (int i = 0; i < numParticles; i++) {
      final AbstractRoadTrackingFilter<?> tmpTrackingFilter =
          this.createRoadTrackingFilter();
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

          final AbstractRoadTrackingFilter<?> trackingFilter =
              this.createRoadTrackingFilter();

          final OnOffEdgeTransDirMulti edgeTransDist =
              new OnOffEdgeTransDirMulti(inferredGraph,
                  parameters.getOnTransitionProbs(),
                  parameters.getOffTransitionProbs());

          final MultivariateGaussian initialBelief =
              trackingFilter.getRoadFilter()
                  .createInitialLearnedObject();
    
          final double lengthLocation =
              edge.getLengthIndexedLine().project(
                  initialObservation.getObsPoint());
    
          final Vector stateSmpl =
              trackingFilter.sampleStateTransDist(initialBelief.getMean(),
                  this.random);
          initialBelief.setMean(stateSmpl);
          initialBelief.getMean().setElement(0, lengthLocation);
    
          final PathStateBelief pathStateBelief = PathStateBelief.getPathStateBelief(
              path, initialBelief);
          
          final VehicleState state =
              new VehicleState(this.inferredGraph,
                  initialObservation, trackingFilter, 
                  pathStateBelief, edgeTransDist, null);

          /*
           * Sample an initial prior for the transition probabilities
           */
          final Vector edgePriorParams =
              state.getEdgeTransitionDist()
                  .getEdgeMotionTransProbPrior().sample(this.random);
          final Vector freeDriorParams =
              state.getEdgeTransitionDist()
                  .getFreeMotionTransProbPrior().sample(this.random);
          state.getEdgeTransitionDist().getEdgeMotionTransPrior()
              .setParameters(edgePriorParams);
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
      final AbstractRoadTrackingFilter<?> trackingFilter =
          this.createRoadTrackingFilter();

      final OnOffEdgeTransDirMulti edgeTransDist =
          new OnOffEdgeTransDirMulti(inferredGraph,
              parameters.getOnTransitionProbs(),
              parameters.getOffTransitionProbs());
      
      final MultivariateGaussian initialBelief = trackingFilter
          .getGroundFilter().createInitialLearnedObject();
      
      final Vector stateSmpl =
          trackingFilter.sampleStateTransDist(initialBelief.getMean(), this.random);

      initialBelief.setMean(stateSmpl);
      initialBelief.getMean().setElement(0, xyPoint.getElement(0));
      initialBelief.getMean().setElement(2, xyPoint.getElement(1));
      
      final PathStateBelief pathStateBelief = PathStateBelief.getPathStateBelief(
          InferredPath.getEmptyPath(), initialBelief);

      final VehicleState state =
          new VehicleState(this.inferredGraph, initialObservation,
              trackingFilter, pathStateBelief, edgeTransDist, null);
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

      retDist.increment(initialDist.sample(this.random));
    }

    return retDist;
  }

  @Nonnull
  abstract protected AbstractRoadTrackingFilter<?> createRoadTrackingFilter();

  public Random getRandom() {
    return random;
  }

  public void setRandom(Random random) {
    this.random = random;
  }

  public long getSeed() {
    return seed;
  }

  public void setSeed(long seed) {
    this.seed = seed;
  }

  @Nonnull
  public Observation getInitialObservation() {
    return initialObservation;
  }

  @Nonnull
  public OtpGraph getInferredGraph() {
    return inferredGraph;
  }

  @Nonnull
  public VehicleStateInitialParameters getParameters() {
    return parameters;
  }

}
