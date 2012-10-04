package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Random;
import java.util.Set;

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

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class VehicleTrackingPathSamplerFilterUpdater implements
    PLParticleFilterUpdater<Observation, VehicleState> {

  private static final long serialVersionUID = 2884138088944317656L;

  private final Observation initialObservation;

  private final OtpGraph inferredGraph;

  private final VehicleStateInitialParameters parameters;

  private Random random;

  public VehicleTrackingPathSamplerFilterUpdater(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
    this.initialObservation = obs;
    this.inferredGraph = inferredGraph;
    this.parameters = parameters;
  }

  @Override
  public VehicleTrackingPathSamplerFilterUpdater clone() {
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

    final StandardRoadTrackingFilter tmpTrackingFilter =
        new StandardRoadTrackingFilter(parameters.getObsVariance(),
            parameters.getOffRoadStateVariance(),
            parameters.getOnRoadStateVariance(),
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
                parameters.getObsVariance(),
                parameters.getOffRoadStateVariance(),
                parameters.getOnRoadStateVariance(),
                parameters.getInitialObsFreq());

        final OnOffEdgeTransDirMulti edgeTransDist =
            new OnOffEdgeTransDirMulti(inferredGraph,
                parameters.getOnTransitionProbs(),
                parameters.getOffTransitionProbs());

        final VehicleState state =
            new VehicleState(this.inferredGraph, initialObservation,
                pathEdge.getInferredEdge(), trackingFilter,
                edgeTransDist, this.random);

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
        new StandardRoadTrackingFilter(parameters.getObsVariance(),
            parameters.getOffRoadStateVariance(),
            parameters.getOnRoadStateVariance(),
            parameters.getInitialObsFreq());

    final OnOffEdgeTransDirMulti edgeTransDist =
        new OnOffEdgeTransDirMulti(inferredGraph,
            parameters.getOnTransitionProbs(),
            parameters.getOffTransitionProbs());

    final VehicleState state =
        new VehicleState(this.inferredGraph, initialObservation,
            InferredEdge.getEmptyEdge(), trackingFilter,
            edgeTransDist, this.random);

    final double lik =
        state.getProbabilityFunction().evaluate(initialObservation);

    initialDist.increment(state, lik);

    final DataDistribution<VehicleState> retDist =
        new DefaultCountedDataDistribution<VehicleState>(
            initialDist.sample(this.random, numParticles));

    return retDist;
  }

  public Observation getInitialObservation() {
    return initialObservation;
  }

  @Override
  public Random getRandom() {
    return random;
  }

  @Override
  public void setRandom(Random rng) {
    this.random = rng;
  }

  /**
   * Sample a path for the given parameters.
   * Note: the passed belief will be the resulting predictive belief.
   * 
   * @param edgeTransDist
   * @param belief
   * @param startEdge
   * @param movementFilter
   * @return
   */
  public InferredPath traverseEdge(
    OnOffEdgeTransDirMulti edgeTransDist,
    final MultivariateGaussian belief, PathEdge startEdge,
    AbstractRoadTrackingFilter movementFilter) {

    /*
     * We project the road path
     */

    /*
     * TODO FIXME there should probably be a flag for this debug stuff
     */
    final Random rng = this.random;
    rng.setSeed(rng.nextLong());

    PathEdge currentEdge = startEdge;
    PathEdge previousEdge = null;
    final MultivariateGaussian newBelief = belief.clone();

    final List<PathEdge> currentPath = Lists.newArrayList();

    double distTraveled = 0d;
    Double totalDistToTravel = null;
    while (totalDistToTravel == null ||
    // the following case is when we're truly on the edge
        Math.abs(totalDistToTravel) > Math.abs(distTraveled)) {

      final List<InferredEdge> transferEdges = Lists.newArrayList();
      if (currentEdge.getInferredEdge() == InferredEdge
          .getEmptyEdge()) {
        final Vector projLocation =
            AbstractRoadTrackingFilter.getOg().times(
                newBelief.getMean());
        for (final StreetEdge edge : this.inferredGraph
            .getNearbyEdges(projLocation,
                movementFilter.getObservationErrorAbsRadius())) {
          transferEdges.add(this.inferredGraph.getInferredEdge(edge));
        }
      } else {
        if (totalDistToTravel == null) {
          transferEdges.add(startEdge.getInferredEdge());
        } else {
          if (newBelief.getMean().getElement(0) < 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
          } else if (newBelief.getMean().getElement(0) > 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getOutgoingTransferableEdges());
          } else {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getOutgoingTransferableEdges());
          }
          // Make sure we don't move back and forth
          transferEdges.remove(currentEdge.getInferredEdge());
        }
      }

      final InferredEdge sampledEdge =
          edgeTransDist.sample(rng, transferEdges,
              currentEdge == null ? startEdge.getInferredEdge()
                  : currentEdge.getInferredEdge());

      if (sampledEdge == InferredEdge.getEmptyEdge()) {

        if (totalDistToTravel == null) {
          /*
           * Off-road, so just return/add the empty path and be done
           */
          movementFilter.predict(newBelief,
              PathEdge.getEmptyPathEdge(), startEdge);
        } else {
          /*
           * This belief should/could extend past the length of the current
           * edge, so that the converted ground coordinates emulate driving
           * off of a road (most of the time, perhaps).
           */
          AbstractRoadTrackingFilter.convertToGroundBelief(newBelief,
              currentEdge, true);
        }

        /*
         * Add some transition noise.
         */
        final Vector transStateSample =
            AbstractRoadTrackingFilter.sampleMovementBelief(rng,
                newBelief.getMean(), movementFilter);
        newBelief.setMean(transStateSample);

        currentEdge = PathEdge.getEmptyPathEdge();
        currentPath.add(PathEdge.getEmptyPathEdge());
        break;
      }

      double direction =
          newBelief.getMean().getElement(0) >= 0d ? 1d : -1d;
      final PathEdge sampledPathEdge =
          PathEdge.getEdge(sampledEdge, previousEdge == null
              || previousEdge.isEmptyEdge() ? 0d : direction
              * previousEdge.getInferredEdge().getLength()
              + previousEdge.getDistToStartOfEdge(), direction < 0d);

      if (sampledPathEdge == null) {
        /*-
         * We have nowhere else to go, but we're not moving off of an edge, so 
         * we call this a stop.
         */
        newBelief.getMean().setElement(0,
            direction * currentEdge.getInferredEdge().getLength());
        newBelief.getMean().setElement(1, 0d);
        break;
      }

      if (totalDistToTravel == null) {
        /*
         * Predict the movement, i.e. distance and direction to travel. The mean
         * of this belief should be set to the true value, so the prediction is
         * exact.
         */

        /*
         * Since we might be just transferring onto an edge, check
         * first.
         */
        final PathEdge initialEdge =
            startEdge.isEmptyEdge() ? sampledPathEdge : startEdge;
        currentEdge = initialEdge;

        if (newBelief.getInputDimensionality() == 4) {
          AbstractRoadTrackingFilter.convertToRoadBelief(newBelief,
              InferredPath.getInferredPath(initialEdge));
        }

        double previousLocation = newBelief.getMean().getElement(0);
        movementFilter.predict(newBelief, initialEdge, initialEdge);

        final Vector transStateSample =
            AbstractRoadTrackingFilter.sampleMovementBelief(rng,
                newBelief.getMean(), movementFilter);
        newBelief.setMean(transStateSample);
        totalDistToTravel =
            newBelief.getMean().getElement(0) - previousLocation;

        double newLocation = newBelief.getMean().getElement(0);
        final double L = initialEdge.getInferredEdge().getLength();

        /*
         * Adjust reference locations to be the same, wrt the new
         * location's direction.
         */
        if (newLocation < 0d && previousLocation > 0d) {
          previousLocation = -L + previousLocation;
          newLocation = previousLocation + totalDistToTravel;
          newBelief.getMean().setElement(0, newLocation);
        } else if (newLocation >= 0d && previousLocation < 0d) {
          previousLocation = L + previousLocation;
          newLocation = previousLocation + totalDistToTravel;
          newBelief.getMean().setElement(0, newLocation);
        }

        /*
         * Get the distance we've covered to move off of this edge, if we
         * have moved off.
         */
        direction = totalDistToTravel >= 0d ? 1d : -1d;
        if (L < Math.abs(newLocation)) {
          final double r = L - Math.abs(previousLocation);
          distTraveled += r * direction;
        } else {
          distTraveled += totalDistToTravel;
        }
      } else {
        /*
         * Continue along edges
         */
        distTraveled +=
            direction * sampledPathEdge.getInferredEdge().getLength();
        currentEdge = sampledPathEdge;
      }
      previousEdge = currentEdge;
      currentPath.add(currentEdge);
    }

    assert (Iterables.getLast(currentPath).isEmptyEdge() || Iterables
        .getLast(currentPath).isOnEdge(
            newBelief.getMean().getElement(0)));

    belief.setMean(newBelief.getMean());
    belief.setCovariance(newBelief.getCovariance());
    return InferredPath.getInferredPath(currentPath,
        (totalDistToTravel != null && totalDistToTravel < 0d) ? true
            : false);
  }

  @Override
  public VehicleState update(VehicleState previousParameter) {
    throw new NotImplementedException();
  }

  public VehicleState update(VehicleState previousState,
    Observation obs) {
    final MultivariateGaussian currentLocBelief =
        previousState.getBelief().clone();
    final PathEdge currentPathEdge =
        PathEdge.getEdge(previousState.getInferredEdge());

    /*
     * TODO FIXME again, a flag for this debug?
     */
    final Random rng = this.random;
    rng.setSeed(rng.nextLong());

    /*
     * Nothing to update in this filter, besides the sampled path
     * and state.
     */
    final OnOffEdgeTransDirMulti newTransDist =
        previousState.getEdgeTransitionDist().clone();
    final AbstractRoadTrackingFilter predictedFilter =
        previousState.getMovementFilter().clone();

    final MultivariateGaussian predictiveBelief = currentLocBelief.clone();
    final InferredPath newPath =
        traverseEdge(newTransDist, predictiveBelief, currentPathEdge,
            predictedFilter);
    
    final PathEdge newPathEdge =
        Iterables.getLast(newPath.getEdges());
    /*
     * We don't really need to keep the gaussian distributions,
     * however, we do, so we must construct a valid posterior state now.
     */
    final MultivariateGaussian newBelief = predictiveBelief.clone();
    if (newPathEdge.isEmptyEdge()) {
      newBelief.setCovariance(previousState.getMovementFilter().getGroundFilter().getModelCovariance());
    } else {
      newBelief.setCovariance(previousState.getMovementFilter().getRoadFilter().getModelCovariance());
    }

    final VehicleState newState =
        new VehicleState(this.inferredGraph, obs, predictedFilter,
            newBelief, newTransDist, newPath,
            previousState);

    return newState;
  }

}