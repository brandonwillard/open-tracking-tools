package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.List;

import javax.annotation.Nonnull;

import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.SimpleSingularValueDecomposition;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.SvdMatrix;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineSegment;

/**
 * This estimator provides a predictive motion state distribution, conditional
 * on a path and one of its edges, that's centered on the path edge with a
 * variance proportional to its length.
 * 
 * @author bwillard
 * 
 */
public class PathStateEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<PathState, Vector, MultivariateGaussian>,
    IncrementalLearner<Vector, PathStateDistribution> {

  /**
   * 
   */
  private static final long serialVersionUID = 4407545320542773361L;

  /**
   * Returns the predictive distribution for the given edge.
   * 
   * @param roadDistribution
   * @param edge
   * @return
   */
  public static TruncatedRoadGaussian getPathEdgePredictive(
    MultivariateGaussian roadDistribution, Matrix measurementError, 
    PathEdge edge, Coordinate obs, Double startDistance, double deltaTime) {
    final Matrix Or = MotionStateEstimatorPredictor.getOr();
    double edgeLength;
    final double distToStartOfEdge;
    /*
     * When we're on the initial edge, we create a 
     * sub-edge that starts at the original distance.
     * This is done to avoid pulling our prior behind
     * the start point (when the prior is after the middle
     * distance of its edge).
     */
    if (startDistance != null && edge.getDistToStartOfEdge() == 0) {
      edgeLength = edge.getLength() - startDistance;
      /*
       * Zero edge length can happen, and does, so we
       * need to provide a non-zero distance over which
       * error can exist.
       */
      if (edgeLength < 10d) {
        distToStartOfEdge = Math.max(startDistance - 10d, 10d);
        edgeLength = edge.getLength() - distToStartOfEdge;
      } else {
        distToStartOfEdge = startDistance;
      }
    } else {
      edgeLength = edge.getLength();
      distToStartOfEdge = edge.getDistToStartOfEdge();
    }

    final double S =
        Or.times(roadDistribution.getCovariance())
            .times(Or.transpose()).getElement(0, 0)
            + Math.pow(edgeLength / Math.sqrt(12), 2);
    final Matrix W =
        roadDistribution.getCovariance().times(Or.transpose())
            .scale(1 / S);
    final Matrix R =
        roadDistribution.getCovariance().minus(
            W.times(W.transpose()).scale(S));

    final double mean;
    if (edgeLength < 1e-5) {
      mean = distToStartOfEdge;
    } else {
      mean = (distToStartOfEdge + (distToStartOfEdge + edgeLength)) / 2d;
//          lineSegment.segmentFraction(obs) * edgeLength
//              + distToStartOfEdge;
    }

    final Vector beliefMean = roadDistribution.getMean();
    final double e = mean - Or.times(beliefMean).getElement(0);
    /*
     * Since we know the dependency between distance/velocity,
     * we can make sure it's kept.
     */
    W.setElement(0, 0, 1d);
    W.setElement(1, 0, 1d / deltaTime);
    final Vector a = beliefMean.plus(W.getColumn(0).scale(e));

    if (a.getElement(1) < 0d) {
      a.setElement(1, 0d);
    }

    final double afterEndDist =
        -Math.min(distToStartOfEdge + edgeLength - a.getElement(0),
            0d);
    final double beforeBeginDist =
        -Math.min(a.getElement(0) - distToStartOfEdge, 0d);
    if (afterEndDist > 0d) {
      if (afterEndDist > 1d) {
        return null;
      } else {
        a.setElement(0, distToStartOfEdge + edgeLength - 1e-5);
      }
    } else if (beforeBeginDist > 0d) {
      if (beforeBeginDist > 1d) {
        return null;
      } else {
        a.setElement(0, distToStartOfEdge + 1e-5);
      }
    }

    SvdMatrix covarResult = new SvdMatrix(R);
    //    if (!covarResult.isSymmetric()) {
    /*
     * Get closest estimate...
     * FIXME above, use a method without the subtraction.
     */
    final AbstractSingularValueDecomposition svd =
        covarResult.getSvd();
    covarResult =
        new SvdMatrix(new SimpleSingularValueDecomposition(
            svd.getU(), svd.getS(), svd.getU().transpose()));
    //    }

    final TruncatedRoadGaussian result =
        new TruncatedRoadGaussian(a, covarResult);

    return result;

  }

  /**
   * Truncated normal mixing component.
   * 
   * @param beliefPrediction
   * @return
   */
  public static double marginalPredictiveLogLikInternal(
    MultivariateGaussian roadDistribution, Matrix stateTransCov, 
    PathEdge edge, Coordinate obs, Double startDistance, double deltaTime) {
    final Matrix Or = MotionStateEstimatorPredictor.getOr();
    final double direction = edge.isBackward() ? -1d : 1d;
    final double thisStartDistance =
        Math.abs(edge.getDistToStartOfEdge());
    final double thisEndDistance =
        edge.getLength() + thisStartDistance;

    final double var =
        Or.times(roadDistribution.getCovariance())
            .times(Or.transpose()).getElement(0, 0);

    final double mean =
        direction
            * Or.times(roadDistribution.getMean()).getElement(0);

    final double t1 =
      LogMath.subtract(
          StatisticsUtil.normalCdf(thisEndDistance, mean, Math.sqrt(var), true),
          StatisticsUtil.normalCdf(thisStartDistance, mean, Math.sqrt(var), true));
    return t1;
  }

  protected VehicleStateDistribution<? extends GpsObservation> currentState;
  protected double currentTimeDiff;

  protected Path path;

  protected PathStateDistribution priorPathStateDistribution;

  public PathStateEstimatorPredictor(
    @Nonnull VehicleStateDistribution<? extends GpsObservation> currentState,
    @Nonnull Path path, double timeDiff) {
    this.currentTimeDiff = timeDiff;
    this.priorPathStateDistribution = null;
    this.path = path;
    this.currentState = currentState;
  }

  /**
   * This estimator takes as conditional parameters the current/previous vehicle
   * state and a path state distribution.
   * 
   * @param currentState
   * @param path
   *          state dist
   * @param prevPathState
   */
  public PathStateEstimatorPredictor(
    @Nonnull VehicleStateDistribution<? extends GpsObservation> currentState,
    @Nonnull PathStateDistribution priorPathStateDistribution,
    double timeDiff) {
    this.currentTimeDiff = timeDiff;
    this.priorPathStateDistribution = priorPathStateDistribution;
    this.path = priorPathStateDistribution.getPathState().getPath();
    this.currentState = currentState;
  }

  @Override
  public PathStateDistribution createInitialLearnedObject() {
    // TODO Auto-generated method stub
    return null;
  }

  /**
   * This method returns the predictive distribution over the conditional path.
   * Note that the prior must be converted to this path's distance and
   * direction, and the current vehicle state must have a value for its path
   * state parameter.
   * 
   * @param prior
   * @return
   */
  @Override
  public PathStateMixtureDensityModel createPredictiveDistribution(
    MultivariateGaussian prior) {

    final List<PathStateDistribution> distributions =
        Lists.newArrayList();
    final List<Double> weights = Lists.newArrayList();
    double totalWeight = Double.NEGATIVE_INFINITY;
    if (this.path.isNullPath()) {
      /*
       * We're conditioned on no path, so go/stay off-road
       */
      MultivariateGaussian groundDistribution;
      if (prior.getInputDimensionality() == 4) {
        groundDistribution = prior.clone();
      } else {
        /*
         * In this situation we require that the current vehicle state tell us
         * which path edge we were on.
         * There must be a valid path edge in the current vehicle state, as well,
         * otherwise this situation wouldn't be consistent (e.g. how is it that
         * our prior is on a road?).
         */
        final PathState prevPathState =
            this.currentState.getPathStateParam().getValue();
        groundDistribution =
            PathUtils.getGroundBeliefFromRoad(prior.clone(),
                prevPathState.getEdge(), true, true);
      }
      distributions.add(new PathStateDistribution(this.path,
          groundDistribution));
      weights.add(0d);
      totalWeight = 0d;
    } else {
      if (prior.getInputDimensionality() == 4) {
        final MultivariateGaussian roadDistribution =
            PathUtils.getRoadBeliefFromGround(prior,
                Iterables.getOnlyElement(this.path.getPathEdges()),
                true, this.currentState.getPathStateParam()
                    .getValue().getMotionState(),
                this.currentTimeDiff);
        /*
         * If we're going on-road, then no need to do the rest.
         * Also, don't consider moving backward (remember, half-normal 
         * velocities).
         */
        if (roadDistribution.getMean().getElement(1) > 0d) {
          distributions.add(new PathStateDistribution(this.path,
              roadDistribution));
          weights.add(0d);
          totalWeight = 0d;
        }
      } else {
        final MultivariateGaussian roadDistribution = prior;
        /*
         * This offset allows us to normalize the marginal
         * likelihood when there are non-zero likelihood regions 
         * (i.e. truncated velocity < 0).
         */
        //        double startOffset = 0d;
        for (final PathEdge edge : this.path.getPathEdges()) {
          final TruncatedRoadGaussian edgeResult =
              PathStateEstimatorPredictor.getPathEdgePredictive(
                  roadDistribution, this.currentState.getOnRoadModelCovarianceParam().getValue(), 
                  edge, this.currentState.getObservation().getObsProjected(), null,
                  this.currentTimeDiff);

          final PathStateDistribution prediction =
              new PathStateDistribution(this.path.getPathTo(edge),
                  edgeResult);

          //          if (prediction == null) {
          //            startOffset += edge.getLength();
          //            break;
          //          }
          distributions.add(prediction);
          final double edgeWeight =
              roadDistribution.getProbabilityFunction().logEvaluate(
                  prediction.getMean());
          //              this.marginalPredictiveLogLikInternal(
          //                    this.path, roadDistribution, edge, startOffset);
          totalWeight = LogMath.add(totalWeight, edgeWeight);
          weights.add(edgeWeight);
        }
      }
    }

    final double[] nativeWeights = new double[weights.size()];
    int i = 0;
    for (final Double weight : weights) {
      nativeWeights[i] = weight - totalWeight;
      i++;
    }

    final PathStateMixtureDensityModel result =
        new PathStateMixtureDensityModel(distributions, nativeWeights);

    Preconditions
        .checkState(result.getDistributionCount() == 0
            || Math.abs(Math.exp(result.getPriorWeightSum()) - 1d) < 1e-5);

    return result;
  }

  @Override
  public MultivariateGaussian learn(
    Collection<? extends PathState> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void update(PathStateDistribution target,
    Iterable<? extends Vector> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void update(PathStateDistribution posterior, Vector data) {
    Preconditions.checkArgument(posterior.getPathState().getPath()
        .equals(this.path));
    if (posterior.getPathState().isOnRoad()) {
      /*
       * Clamp the projected obs
       */
      if (!this.path.isOnPath(posterior.getMean().getElement(0))) {
        final Vector mean = posterior.getMean().clone();
        mean.setElement(0, posterior.getPathState().getPath()
            .clampToPath(posterior.getMean().getElement(0)));
        posterior.setMean(mean);
      }

      /*
       * We don't want to move behind the current path state after an update, 
       * since moving backward doesn't happen here.
       */
      final PathStateDistribution currentPathDist =
          this.priorPathStateDistribution;
      if (currentPathDist.getMotionDistribution().getMean()
          .getElement(0) < posterior.getMean().getElement(0)) {
        final Vector newMean =
            currentPathDist.getMotionDistribution().getMean().clone();
        newMean.setElement(1, posterior.getMean().getElement(1));
        posterior.setMean(newMean);
      }
    }
  }

}
