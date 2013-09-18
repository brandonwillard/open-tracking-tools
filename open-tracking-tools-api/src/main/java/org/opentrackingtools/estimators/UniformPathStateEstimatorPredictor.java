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
 * This implementation is for a uniform distribution over the length of the edge.
 * 
 * @author bwillard
 * 
 */
public class UniformPathStateEstimatorPredictor extends
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

    double edgeLength;
    final double distToStartOfEdge;
    if (startDistance != null && edge.getDistToStartOfEdge() == 0) {
      edgeLength = edge.getLength() - startDistance;
      /*
       * Zero edge length can happen, and does, so we
       * need to provide a non-zero distance over which
       * error can exist.
       */
      if (edgeLength < 10d) {
        edgeLength = Math.max(startDistance - 10d, 10d);
        distToStartOfEdge = startDistance;
      } else {
        distToStartOfEdge = startDistance;
      }
    } else {
      edgeLength = edge.getLength();
      distToStartOfEdge = edge.getDistToStartOfEdge();
    }


    final Vector a = roadDistribution.getMean();
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

    SvdMatrix covarResult = new SvdMatrix(roadDistribution.getCovariance());
    final AbstractSingularValueDecomposition svd =
        covarResult.getSvd();
    covarResult =
        new SvdMatrix(new SimpleSingularValueDecomposition(
            svd.getU(), svd.getS(), svd.getU().transpose()));

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
    return 0d;
  }

  protected VehicleStateDistribution<? extends GpsObservation> currentState;
  protected double currentTimeDiff;

  protected Path path;

  protected PathStateDistribution priorPathStateDistribution;

  public UniformPathStateEstimatorPredictor(
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
  public UniformPathStateEstimatorPredictor(
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
   * TODO what to do about this?
   * 
   * @param prior
   * @return
   */
  @Override
  public PathStateMixtureDensityModel createPredictiveDistribution(
    MultivariateGaussian prior) {
    return null;
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
    // TODO Auto-generated method stub
  }

}
