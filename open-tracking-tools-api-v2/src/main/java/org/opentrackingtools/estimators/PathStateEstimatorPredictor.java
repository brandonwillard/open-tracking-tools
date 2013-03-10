package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.annotation.Nonnull;

import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.primitives.Doubles;

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
  protected VehicleState<? extends GpsObservation> currentState;
  protected Path path;

  /**
   * This estimator takes as conditional parameters the current/previous vehicle
   * state and a path.
   * 
   * @param currentState
   * @param path
   * @param prevPathState
   */
  public PathStateEstimatorPredictor(
    @Nonnull VehicleState<? extends GpsObservation> currentState,
    @Nonnull Path path) {
    this.path = path;
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
  public PathStateMixtureDensityModel<PathStateDistribution>
      createPredictiveDistribution(MultivariateGaussian prior) {

    final List<PathStateDistribution> distributions =
        Lists.newArrayList();
    final List<Double> weights = Lists.newArrayList();
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
                prevPathState.getEdge(), true);
      }
      distributions.add(new PathStateDistribution(this.path,
          groundDistribution));
      weights.add(1d);
    } else {
      MultivariateGaussian roadDistribution;
      if (prior.getInputDimensionality() == 4) {
        roadDistribution =
            PathUtils.getRoadBeliefFromGround(prior,
                Iterables.getFirst(this.path.getPathEdges(), null),
                true);
      } else {
        roadDistribution = prior;
      }

      for (final PathEdge<?> edge : this.path.getPathEdges()) {

        final Matrix Or = MotionStateEstimatorPredictor.getOr();
        final double S =
            Or.times(roadDistribution.getCovariance())
                .times(Or.transpose()).getElement(0, 0)
                // + 1d;
                + Math.pow(edge.getLength() / Math.sqrt(12), 2);
        final Matrix W =
            roadDistribution.getCovariance().times(Or.transpose())
                .scale(1 / S);
        final Matrix R =
            roadDistribution.getCovariance().minus(
                W.times(W.transpose()).scale(S));

        final double direction = edge.isBackward() ? -1d : 1d;
        final double mean =
            (edge.getDistToStartOfEdge() + (edge
                .getDistToStartOfEdge() + direction
                * edge.getLength())) / 2d;

        final Vector beliefMean = roadDistribution.getMean();
        final double e = mean - Or.times(beliefMean).getElement(0);
        final Vector a = beliefMean.plus(W.getColumn(0).scale(e));

        assert StatisticsUtil
            .isPosSemiDefinite((gov.sandia.cognition.math.matrix.mtj.DenseMatrix) R);

        final PathStateDistribution prediction =
            new PathStateDistribution(this.path,
                new MultivariateGaussian(a, R));

        distributions.add(prediction);
        weights.add(this.marginalPredictiveLogLikInternal(this.path,
            prediction.getMotionStateDistribution(), edge));
      }
    }
    final PathStateMixtureDensityModel<PathStateDistribution> result =
        new PathStateMixtureDensityModel<PathStateDistribution>(
            distributions);
    result.setDistributions((ArrayList) distributions);
    result.setPriorWeights(Doubles.toArray(weights));

    return result;
  }

  /**
   * 
   * @param prior
   * @param startPathState
   * @return
   */
  private PathStateDistribution getPathStateDistributionFromGround(
    MultivariateGaussian prior, PathState startPathState) {

    Preconditions.checkArgument(prior.getInputDimensionality() == 4
        && startPathState.isOnRoad());

    final double direction =
        (startPathState.getEdge().isBackward() ? -1d : 1d);
    final double velocity =
        direction
            * VectorFactory
                .getDenseDefault()
                .copyVector(
                    MotionStateEstimatorPredictor.getVg().times(
                        prior.getMean())).norm2();

    final double distanceMovedFromStartOfPath =
        Math.signum(velocity)
            * MotionStateEstimatorPredictor.Og.times(prior.getMean())
                .euclideanDistance(
                    MotionStateEstimatorPredictor.Og
                        .times(startPathState.getGroundState()))
            + startPathState.getEdge().getDistToStartOfEdge();

    final MultivariateGaussian distribution =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(distanceMovedFromStartOfPath, velocity),
            prior.getCovariance());

    return new PathStateDistribution(this.path, distribution);
  }

  @Override
  public MultivariateGaussian learn(
    Collection<? extends PathState> data) {
    // TODO Auto-generated method stub
    return null;
  }

  /**
   * Truncated normal mixing component.
   * 
   * @param beliefPrediction
   * @return
   */
  protected double marginalPredictiveLogLikInternal(Path path,
    MultivariateGaussian beliefPrediction, PathEdge<?> currentEdge) {

    final double direction = currentEdge.isBackward() ? -1d : 1d;
    final double thisStartDistance =
        Math.abs(currentEdge.getDistToStartOfEdge());
    final double thisEndDistance =
        currentEdge.getLength() + thisStartDistance;

    final Matrix Or = MotionStateEstimatorPredictor.getOr();

    final double var =
        Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0);

    final double mean =
        direction
            * Or.times(beliefPrediction.getMean()).getElement(0);

    final double t1 =
        UnivariateGaussian.CDF.evaluate(thisEndDistance, mean, var)
            - UnivariateGaussian.CDF.evaluate(thisStartDistance,
                mean, var);

    final double Z =
        UnivariateGaussian.CDF.evaluate(
            Math.abs(path.getTotalPathDistance()), mean, var)
            - UnivariateGaussian.CDF.evaluate(0, mean, var);

    return Math.log(t1) - Math.log(Z);
  }

  @Override
  public void update(PathStateDistribution target,
    Iterable<? extends Vector> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void update(PathStateDistribution prior, Vector data) {
    if (prior.getPathState().isOnRoad()) {
      /*
       * Clamp the projected obs
       */
      if (!this.path.isOnPath(prior.getMean().getElement(0))) {
        prior.getMean().setElement(
            0,
            prior.getPathState().getPath()
                .clampToPath(prior.getMean().getElement(0)));
      }
    }
  }

}
