package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.UnivariateStatisticsUtil;
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
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
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
  protected VehicleStateDistribution<? extends GpsObservation> currentState;
  protected Path path;
  protected PathStateDistribution priorPathStateDistribution;

  /**
   * This estimator takes as conditional parameters the current/previous vehicle
   * state and a path state distribution.
   * 
   * @param currentState
   * @param path state dist
   * @param prevPathState
   */
  public PathStateEstimatorPredictor(
    @Nonnull VehicleStateDistribution<? extends GpsObservation> currentState,
    @Nonnull PathStateDistribution priorPathStateDistribution) {
    this.priorPathStateDistribution = priorPathStateDistribution;
    this.path = priorPathStateDistribution.getPathState().getPath();
    this.currentState = currentState;
  }
  
  public PathStateEstimatorPredictor(
    @Nonnull VehicleStateDistribution<? extends GpsObservation> currentState,
    @Nonnull Path path) {
    this.priorPathStateDistribution = null;
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
  public PathStateMixtureDensityModel
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
                prevPathState.getEdge(), true, true);
      }
      distributions.add(new PathStateDistribution(this.path,
          groundDistribution));
      weights.add(0d);
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

      for (final PathEdge edge : this.path.getPathEdges()) {
        PathStateDistribution prediction = getPathEdgePredictive(roadDistribution, edge);
        distributions.add(prediction);
        weights.add(
            this.path.getPathEdges().size() == 1 ? 0d :
              this.marginalPredictiveLogLikInternal(this.path, roadDistribution, edge));
      }
    }
    
    final PathStateMixtureDensityModel result =
        new PathStateMixtureDensityModel(distributions, Doubles.toArray(weights));

    Preconditions.checkState(
        Math.abs(Math.exp(result.getPriorWeightSum()) - 1d) < 1e-5);
    
    return result;
  }
  
  /**
   * Returns the predictive distribution for the given edge.
   * 
   * @param roadDistribution
   * @param edge
   * @return
   */
  public PathStateDistribution getPathEdgePredictive(MultivariateGaussian roadDistribution, PathEdge edge) {
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
      //roadDistribution.getCovariance();

    final double direction = edge.isBackward() ? -1d : 1d;
    final double mean =
        (edge.getDistToStartOfEdge() + (edge
            .getDistToStartOfEdge() + direction
            * edge.getLength())) / 2d;

    final Vector beliefMean = roadDistribution.getMean();
    final double e = mean - Or.times(beliefMean).getElement(0);
    Vector a = beliefMean.plus(W.getColumn(0).scale(e));
//    roadDistribution.getMean();
    
    /*
     * Truncate our new mean so that it's forced onto the edge.
     * Otherwise, it would be possible for a mixture element
     * to not correspond to its edge.
     */
    a = edge.clampToEdge(a);

    assert StatisticsUtil
        .isPosSemiDefinite((gov.sandia.cognition.math.matrix.mtj.DenseMatrix) R);

    final PathStateDistribution prediction =
        new PathStateDistribution(this.path.getPathTo(edge),
            new TruncatedRoadGaussian(a, R));
    
    return prediction;
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
  public double marginalPredictiveLogLikInternal(Path path,
    MultivariateGaussian beliefPrediction, PathEdge currentEdge) {

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
      LogMath.subtract(
          StatisticsUtil.normalCdf(thisEndDistance, mean, Math.sqrt(var), true),
          StatisticsUtil.normalCdf(thisStartDistance, mean, Math.sqrt(var), true));
//        UnivariateGaussian.CDF.evaluate(thisEndDistance, mean, var)
//            - UnivariateGaussian.CDF.evaluate(thisStartDistance,
//                mean, var);

    final double Z =
      LogMath.subtract(
          StatisticsUtil.normalCdf(Math.abs(path.getTotalPathDistance()), mean, Math.sqrt(var), true),
          StatisticsUtil.normalCdf(0d, mean, Math.sqrt(var), true));
//        UnivariateGaussian.CDF.evaluate(
//            Math.abs(path.getTotalPathDistance()), mean, var)
//            - UnivariateGaussian.CDF.evaluate(0, mean, var);

//    return Math.log(t1) - Math.log(Z);
    return t1 - Z;
  }

  @Override
  public void update(PathStateDistribution target,
    Iterable<? extends Vector> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void update(PathStateDistribution posterior, Vector data) {
    Preconditions.checkArgument(posterior.getPathState().getPath().equals(this.path));
    if (posterior.getPathState().isOnRoad()) {
      /*
       * Clamp the projected obs
       */
      if (!this.path.isOnPath(posterior.getMean().getElement(0))) {
        final Vector mean = posterior.getMean().clone();
        mean.setElement(
            0,
            posterior.getPathState().getPath()
                .clampToPath(posterior.getMean().getElement(0)));
        posterior.setMean(mean);
      }
      
      /*
       * We don't want to move behind the current path state after an update, 
       * since moving backward doesn't happen here.
       */
      PathStateDistribution currentPathDist = this.priorPathStateDistribution;
      if (currentPathDist.getMotionDistribution().getMean().getElement(0) < 
          posterior.getMean().getElement(0)) {
        final Vector newMean = currentPathDist.getMotionDistribution().getMean().clone();
        newMean.setElement(1, posterior.getMean().getElement(1));
        posterior.setMean(newMean);
      }
    }
  }

}
