package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.ComplexNumber;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.CholeskyDecompositionMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Map.Entry;
import java.util.Random;

import no.uib.cipr.matrix.DenseCholesky;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public abstract class AbstractRoadTrackingFilter extends
    AbstractCloneableSerializable {

  /**
   * 
   */
  private static final long serialVersionUID = -3818533301279461087L;

  /**
   * Motion model of the underlying system.
   */
  protected LinearDynamicalSystem groundModel;

  protected AdjKalmanFilter groundFilter;

  protected LinearDynamicalSystem roadModel;
  protected AdjKalmanFilter roadFilter;

  protected Matrix Qr;
  protected Matrix Qg;

  protected Matrix onRoadStateVariance;
  protected Matrix offRoadStateVariance;
  protected Matrix obsVariance;

  /*
   * Observation matrix
   */
  protected static Matrix Og;
  protected static Matrix Or;
  protected static Matrix U;

  static {
    Og = MatrixFactory.getDefault().createMatrix(2, 4);
    Og.setElement(0, 0, 1);
    Og.setElement(1, 2, 1);

    U = MatrixFactory.getDefault().createMatrix(4, 4);
    U.setElement(0, 0, 1);
    U.setElement(1, 2, 1);
    U.setElement(2, 1, 1);
    U.setElement(3, 3, 1);

    Or = MatrixFactory.getDefault().createMatrix(1, 2);
    Or.setElement(0, 0, 1);
  }

  protected double currentTimeDiff = 0d;

  protected double prevTimeDiff = 0d;

  protected final static Vector zeros2D = VectorFactory.getDefault()
      .copyValues(0, 0);

  @Override
  public AbstractRoadTrackingFilter clone() {
    final AbstractRoadTrackingFilter clone =
        (AbstractRoadTrackingFilter) super.clone();
    clone.currentTimeDiff = this.currentTimeDiff;
    clone.groundFilter = this.groundFilter.clone();
    clone.groundModel = this.groundModel.clone();
    clone.obsVariance = this.obsVariance.clone();
    clone.offRoadStateVariance = this.offRoadStateVariance.clone();
    clone.onRoadStateVariance = this.onRoadStateVariance.clone();
    clone.prevTimeDiff = this.prevTimeDiff;
    clone.Qg = this.Qg.clone();
    clone.Qr = this.Qr.clone();
    clone.roadFilter = this.roadFilter.clone();
    clone.roadModel = this.roadModel.clone();
    return clone;
  }

  public MultivariateGaussian createInitialLearnedObject() {
    return new MultivariateGaussian(groundFilter.getModel()
        .getState(), groundFilter.getModelCovariance());
  }

  public Matrix getCovarianceFactor(boolean isRoad) {
    return getCovarianceFactor(this.currentTimeDiff, isRoad);
  }

  public double getCurrentTimeDiff() {
    return currentTimeDiff;
  }

  public AdjKalmanFilter getGroundFilter() {
    return groundFilter;
  }

  public LinearDynamicalSystem getGroundModel() {
    return groundModel;
  }

  public MultivariateGaussian getObservationBelief(
    final MultivariateGaussian belief, PathEdge edge) {
    final MultivariateGaussian projBelief = belief.clone();
    if (projBelief.getInputDimensionality() == 2) {
      convertToGroundBelief(projBelief, edge);
    }

    final Matrix Q =
        Og.times(projBelief.getCovariance()).times(Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final MultivariateGaussian res =
        new MultivariateGaussian(Og.times(projBelief.getMean()), Q);
    return res;
  }

  public double getObservationErrorAbsRadius() {
    final double varDistance =
        1.98d * Math.sqrt(this.getObsVariance().normFrobenius()
            / Math.sqrt(2));
    return varDistance;
  }

  public Matrix getObsVariance() {
    return obsVariance;
  }

  public Matrix getOffRoadStateVariance() {
    return offRoadStateVariance;
  }

  public Matrix getOnRoadStateVariance() {
    return onRoadStateVariance;
  }

  public double getPrevTimeDiff() {
    return prevTimeDiff;
  }

  public Matrix getQg() {
    return Qg;
  }

  public Matrix getQr() {
    return Qr;
  }

  public AdjKalmanFilter getRoadFilter() {
    return roadFilter;
  }

  public LinearDynamicalSystem getRoadModel() {
    return roadModel;
  }

  /**
   * @param obs
   * @param belief
   * @return
   */
  public double priorPredictiveLogLikelihood(Vector obs,
    MultivariateGaussian belief, PathEdge edge) {
    final MultivariateGaussian projBelief = belief.clone();
    if (projBelief.getInputDimensionality() == 2) {
      convertToGroundBelief(projBelief, edge);
    }

    final Matrix Q =
        Og.times(projBelief.getCovariance()).times(Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final double result =
        StatisticsUtil.logEvaluateNormal(obs,
            Og.times(projBelief.getMean()), Q);
    return result;
  }

  public double logLikelihood(Vector obs,
    MultivariateGaussian belief, PathEdge edge) {
    final MultivariateGaussian projBelief = belief.clone();
    if (projBelief.getInputDimensionality() == 2) {
      convertToGroundBelief(projBelief, edge);
    }

    final double result =
        StatisticsUtil.logEvaluateNormal(obs,
            Og.times(projBelief.getMean()),
            this.groundFilter.getMeasurementCovariance());
    return result;
  }

  /**
   * Updates the road-coordinates prior predictive belief to the posterior for
   * the given observation, edge and path distance to the start of the edge.
   * 
   * @param belief
   * @param observation
   * @param edge
   */
  public void measure(MultivariateGaussian belief,
    Vector observation, InferredPath path) {

    if (belief.getInputDimensionality() == 2) {
      /*
       * Convert road-coordinates prior predictive to ground-coordinates
       */
      final MultivariateGaussian updatedBelief = belief.clone();
      final PathEdge startEdge =
          path.getEdgeForDistance(belief.getMean().getElement(0),
              true);
      convertToGroundBelief(updatedBelief, startEdge);

      //      // TODO FIXME debug. remove
      //      final MultivariateGaussian debugBelief = updatedBelief.clone();

      this.groundFilter.measure(updatedBelief, observation);

      //      // TODO FIXME debug. remove
      //      if (Og.times(debugBelief.getMean()).euclideanDistance(
      //          observation) < Og.times(updatedBelief.getMean())
      //          .euclideanDistance(observation)) {
      //        assert false;
      //      }

      /*
       * Convert back to road-coordinates
       */
      convertToRoadBelief(updatedBelief, path, startEdge);

      final MultivariateGaussian debugBelief2 = updatedBelief.clone();
      convertToGroundBelief(debugBelief2, startEdge);

      //      // TODO FIXME debug. remove
      //      if (Og.times(debugBelief.getMean()).euclideanDistance(
      //          observation)
      //          - Og.times(debugBelief2.getMean()).euclideanDistance(
      //              observation) <= -5) {
      //        assert false;
      //      }

      belief.setMean(updatedBelief.getMean());
      belief.setCovariance(updatedBelief.getCovariance());
    } else {
      Preconditions
          .checkArgument(belief.getInputDimensionality() == 4);
      this.groundFilter.measure(belief, observation);
    }

  }

  /**
   * Pass it a road-coordinates prior predictive belief distribution, edge and
   * path starting distance, and it will update the prior predictive
   * distribution for that edge and path. Otherwise, project free-movement onto
   * an edge or predict free movement.
   * 
   * @param startOfEdgeDist
   */
  public void predict(MultivariateGaussian currentBelief,
    PathEdge newEdge, PathEdge currentEdge) {
    Preconditions.checkArgument(currentBelief
        .getInputDimensionality() == 2
        || currentBelief.getInputDimensionality() == 4);
    Preconditions.checkNotNull(newEdge);

    if (newEdge.isEmptyEdge()) {
      if (currentBelief.getInputDimensionality() == 4) {
        /*-
         * Predict free-movement
         */
        groundFilter.predict(currentBelief);
      } else {
        /*-
         * Going off-road
         */
        Preconditions.checkNotNull(currentEdge);
        convertToGroundBelief(currentBelief, currentEdge);
        groundFilter.predict(currentBelief);
      }
    } else {
      Preconditions.checkNotNull(currentEdge);
      Preconditions.checkArgument(currentEdge.isEmptyEdge()
          || currentEdge.getDistToStartOfEdge() == 0d);
      if (currentBelief.getInputDimensionality() == 4) {
        /*-
         * Predict movement onto a path/edge.
         * Currently, this just consists of projecting onto
         * the edge
         * 
         */
        Preconditions
            .checkArgument(newEdge.getDistToStartOfEdge() == 0d);
        convertToRoadBelief(currentBelief,
            InferredPath.getInferredPath(newEdge));
      } else {
      }
      roadFilter.predict(currentBelief);
    }

  }

  /**
   * Use this sampling method to beat the inherent degeneracy of our state
   * covariance.
   * 
   * @param vehicleState
   * @return
   */
  public Vector sampleStateTransition(Vector mean, InferredPath path,
    Random rng) {
    final boolean isRoad = mean.getDimensionality() == 2;
    final Matrix Q = isRoad ? this.getQr() : this.getQg();
    final Matrix covSqrt;
    if (!isRoad) {
      covSqrt =
          CholeskyDecompositionMTJ.create(
              DenseMatrixFactoryMTJ.INSTANCE.copyMatrix(Q)).getR();
    } else {
      covSqrt = DenseMatrixFactoryMTJ.INSTANCE.copyMatrix(Q);
      covSqrt.setElement(0, 0, Math.sqrt(Q.getElement(0, 0)));
    }

    final Vector underlyingSample =
        MultivariateGaussian.sample(VectorFactory.getDefault()
            .createVector(isRoad ? 1 : 2), covSqrt, rng);
    final Matrix Gamma = this.getCovarianceFactor(isRoad);
    final Vector thisStateSample =
        Gamma.times(underlyingSample).plus(mean);

    if (isRoad && !path.isOnPath(thisStateSample.getElement(0))) {

      final double dir = path.getIsBackward() ? -1d : 1d;
      final LengthIndexedLine lil =
          new LengthIndexedLine(path.getGeometry());
      final double clampedIndex =
          dir * lil.clampIndex(dir * thisStateSample.getElement(0));
      thisStateSample.setElement(0, clampedIndex);

    }

    return thisStateSample;
  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    if (currentTimeDiff != prevTimeDiff) {
      groundFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, Qg, false));
      roadFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, Qr, true));

      groundModel.setA(createStateTransitionMatrix(currentTimeDiff,
          false));
      roadModel.setA(createStateTransitionMatrix(currentTimeDiff,
          true));
    }
    this.prevTimeDiff = this.currentTimeDiff;
    this.currentTimeDiff = currentTimeDiff;
  }

  @Override
  public String toString() {
    return "StandardRoadTrackingFilter [" + "groundFilterCov="
        + groundFilter.getModelCovariance() + ", roadFilterCov="
        + roadFilter.getModelCovariance() + ", onRoadStateVariance="
        + onRoadStateVariance + ", offRoadStateVariance="
        + offRoadStateVariance + ", obsVariance=" + obsVariance
        + ", currentTimeDiff=" + currentTimeDiff + "]";
  }

  public static boolean checkPosDef(DenseMatrix covar) {
    final EigenDecompositionRightMTJ decomp =
        EigenDecompositionRightMTJ.create(covar);
    for (final ComplexNumber eigenVal : decomp.getEigenValues()) {
      if (eigenVal.getRealPart() < 0)
        return false;
    }
    return true;
  }

  public static void convertToGroundBelief(
    MultivariateGaussian belief, PathEdge edge) {
    convertToGroundBelief(belief, edge, false);
  }

  public static void convertToGroundBelief(
    MultivariateGaussian belief, PathEdge edge,
    boolean allowExtensions) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2
        || belief.getInputDimensionality() == 4);

    if (belief.getInputDimensionality() == 4)
      return;

    Preconditions.checkArgument(!edge.isEmptyEdge());

    final Vector positiveMean;
    if (belief.getMean().getElement(0) < 0d
        || (belief.getMean().getElement(0) == 0d && edge.isBackward() == Boolean.TRUE)) {
      /*
       * We're going all positive here, since we should've been using
       * the reversed geometry if negative.
       */
      final Vector posMeanTmp = belief.getMean().clone();

      double posLocation =
          Math.max(
              0d,
              belief.getMean().getElement(0)
                  + edge.getInferredEdge().getLength()
                  + Math.abs(edge.getDistToStartOfEdge()));

      /*
       * In cases where large negative movements past an edge are made,
       * we need to adjust the excess to be positive.
       */
      if (allowExtensions && posLocation < 0d) {
        posLocation =
            edge.getInferredEdge().getLength()
                + Math.abs(belief.getMean().getElement(0));
      }

      posMeanTmp.setElement(0, posLocation);
      positiveMean = posMeanTmp;
    } else if (belief.getMean().getElement(0) > 0d
        || (belief.getMean().getElement(0) == 0d && edge.isBackward() == Boolean.FALSE)) {

      positiveMean = belief.getMean().clone();

      assert edge.getDistToStartOfEdge() >= 0d;

      positiveMean.setElement(
          0,
          Math.max(
              0d,
              positiveMean.getElement(0)
                  - edge.getDistToStartOfEdge()));
    } else {
      throw new IllegalStateException();
    }

    assert positiveMean.getElement(0) >= 0d
        && (allowExtensions || positiveMean.getElement(0) <= edge
            .getInferredEdge().getLength() + 1e-4);

    final Geometry geom = edge.getInferredEdge().getGeometry();
    final Entry<LineSegment, Double> segmentDist =
        getSegmentAndDistanceToStart(geom, positiveMean.getElement(0));
    final double absTotalPathDistanceToStartOfSegment =
        Math.abs(segmentDist.getValue());
    final double absTotalPathDistanceToEndOfSegment =
        absTotalPathDistanceToStartOfSegment
            + segmentDist.getKey().getLength();
    final Entry<Matrix, Vector> projPair =
        AbstractRoadTrackingFilter.posVelProjectionPair(
            segmentDist.getKey(),
            absTotalPathDistanceToStartOfSegment);

    if (!allowExtensions) {
      //    assert (Math.abs(belief.getMean().getElement(0)) <= absTotalPathDistanceToEndOfSegment 
      //        && Math.abs(belief.getMean().getElement(0)) >= absTotalPathDistanceToStartOfSegment);

      /*
       * Truncate, to keep it on the edge.
       */
      if (positiveMean.getElement(0) > absTotalPathDistanceToEndOfSegment) {
        positiveMean
            .setElement(0, absTotalPathDistanceToEndOfSegment);
      } else if (positiveMean.getElement(0) < absTotalPathDistanceToStartOfSegment) {
        positiveMean.setElement(0,
            absTotalPathDistanceToStartOfSegment);
      }
    }

    final Matrix C = belief.getCovariance();
    final Vector projMean =
        projPair.getKey().times(positiveMean)
            .plus(projPair.getValue());
    final Matrix projCov =
        projPair.getKey().times(C)
            .times(projPair.getKey().transpose());

    belief.setMean(projMean);
    belief.setCovariance(projCov);
  }

  public static void convertToRoadBelief(MultivariateGaussian belief,
    InferredPath path) {
    convertToRoadBelief(belief, path, null);
  }

  public static void convertToRoadBelief(MultivariateGaussian belief,
    InferredPath path, PathEdge edge) {

    // TODO FIXME XXX make sure this works with the direction of motion.
    Preconditions.checkArgument(belief.getInputDimensionality() == 2
        || belief.getInputDimensionality() == 4);
    Preconditions.checkArgument(edge == null
        || path.getEdges().contains(edge));

    if (belief.getInputDimensionality() == 2)
      return;

    Preconditions.checkArgument(!path.isEmptyPath());

    final Vector m = belief.getMean().clone();
    final Matrix C = belief.getCovariance().clone();

    /*
     * We snap to the line and find the segment of interest.
     * When we're given a non-null distanceThreshold, we attempt
     * to snap a location that is between the start of the edge 
     * for the distance threshold and the actual distance threshold.
     */
    final Geometry geom = path.getGeometry();
    final LinearLocation lineLocation;
    final Coordinate currentPos =
        GeoUtils.makeCoordinate(Og.times(m));
    final double distanceToStartOfSegmentOnGeometry;
    final LineSegment pathLineSegment;
    if (edge != null) {
      final Geometry edgeGeom =
          path.getIsBackward() ? edge.getInferredEdge().getGeometry()
              .reverse() : edge.getInferredEdge().getGeometry();
      final LocationIndexedLine locIndex =
          new LocationIndexedLine(edgeGeom);
      final LinearLocation tmpLocation = locIndex.project(currentPos);
      final LengthIndexedLine tmpLengthIdx =
          new LengthIndexedLine(edgeGeom);
      final double lengthOnEdge =
          tmpLengthIdx.indexOf(tmpLocation.getCoordinate(edgeGeom));

      /*
       * Due to some really weird behavior with indexAfter in JTS,
       * we're doing this for the edge first, then computing the distance
       * up to it on the path and using that with the full path geom.
       */
      pathLineSegment = tmpLocation.getSegment(edgeGeom);
      final LengthIndexedLine lengthIndex =
          new LengthIndexedLine(edgeGeom);
      distanceToStartOfSegmentOnGeometry =
          Math.abs(edge.getDistToStartOfEdge())
              + lengthIndex.indexOf(pathLineSegment.p0);

      lineLocation =
          LengthLocationMap.getLocation(geom,
              Math.abs(edge.getDistToStartOfEdge()) + lengthOnEdge);
    } else {
      final LocationIndexedLine locIndex =
          new LocationIndexedLine(geom);
      lineLocation = locIndex.project(currentPos);
      /*
       * Get the segment we're projected onto, and the distance offset
       * of the path.
       */
      pathLineSegment = lineLocation.getSegment(geom);
      final LengthIndexedLine lengthIndex =
          new LengthIndexedLine(geom);
      distanceToStartOfSegmentOnGeometry =
          lengthIndex.indexOf(pathLineSegment.p0);
    }

    final Coordinate pointOnLine = lineLocation.getCoordinate(geom);
    m.setElement(0, pointOnLine.x);
    m.setElement(2, pointOnLine.y);

    final Entry<Matrix, Vector> projPair =
        AbstractRoadTrackingFilter.posVelProjectionPair(
            pathLineSegment, distanceToStartOfSegmentOnGeometry);

    final Vector projMean =
        projPair.getKey().transpose()
            .times(m.minus(projPair.getValue()));

    /*
     * Since this projection was working wrt. positive movement,
     * if we're on a path moving backward, we need to flip the signs.
     * otherwise, predictions will advance in the opposite direction!
     */
    if (path.getIsBackward()) {
      //      final double distanceToEndOfSegmentOnGeometry = 
      //          lengthIndex.indexOf(lineSegment.p1);
      //      projMean.setElement(0, distanceToEndOfSegmentOnGeometry 
      //          - projMean.getElement(0));
      projMean.scaleEquals(-1d);
    }

    // this should be done, when needed, elsewhere
    //    normalizeBelief(projMean,
    //        path.getEdgeForDistance(projMean.getElement(0), true));

    final Matrix projCov =
        projPair.getKey().transpose().times(C)
            .times(projPair.getKey());

    belief.setMean(projMean);
    belief.setCovariance(projCov);
  }

  /**
   * Creates either a diagonal matrix with diag = xa0Variance, yaVariance, or
   * the aforementioned matrix rotated by the x-axis angle angle, with
   * xa0Variance, yaVariance the perpendicular, parallel variances.
   * 
   * @param timeDiff
   * @param yaVariance
   * @param xa0Variance
   * @param angle
   * @return
   */
  protected static Matrix createStateCovarianceMatrix(
    double timeDiff, Matrix Q, boolean isRoad) {

    final Matrix A_half = getCovarianceFactor(timeDiff, isRoad);
    final Matrix A = A_half.times(Q).times(A_half.transpose());

    return A;
  }

  protected static Matrix createStateTransitionMatrix(
    double timeDiff, boolean isRoad) {

    final int dim;
    if (isRoad) {
      dim = 2;
    } else {
      dim = 4;
    }
    final Matrix Gct =
        MatrixFactory.getDefault().createIdentity(dim, dim);
    Gct.setElement(0, 1, timeDiff);
    if (dim > 2)
      Gct.setElement(2, 3, timeDiff);

    return Gct;
  }

  public static Matrix getCovarianceFactor(double timeDiff,
    boolean isRoad) {

    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half =
        MatrixFactory.getDefault().createMatrix(dim * 2, dim);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }

    return A_half;
  }

  public static Matrix getGroundObservationMatrix() {
    return Og;
  }

  public static Matrix getOg() {
    return Og;
  }

  public static Matrix getOr() {
    return Or;
  }

  public static Matrix getRoadObservationMatrix() {
    return Or;
  }

  protected static Matrix getRotatedCovarianceMatrix(
    double aVariance, double a0Variance, double angle) {

    final Matrix rotationMatrix =
        MatrixFactory.getDefault().createIdentity(2, 2);
    rotationMatrix.setElement(0, 0, Math.cos(angle));
    rotationMatrix.setElement(0, 1, -Math.sin(angle));
    rotationMatrix.setElement(1, 0, Math.sin(angle));
    rotationMatrix.setElement(1, 1, Math.cos(angle));

    final Matrix temp =
        MatrixFactory.getDefault().createDiagonal(
            VectorFactory.getDefault().copyArray(
                new double[] { a0Variance, aVariance }));
    return rotationMatrix.times(temp).times(
        rotationMatrix.transpose());
  }

  /**
   * Returns the lineSegment in the geometry of the edge and the
   * distance-to-start of the segment on the entire path. The line segment is in
   * the direction of the edge's geometry, and the distance-to-start has the
   * same sign as the direction of movement.
   * 
   * @param edge
   * @param distanceAlong
   * @return
   */
  public static Entry<LineSegment, Double>
      getSegmentAndDistanceToStart(Geometry geometry,
        double distAlongGeometry) {

    Preconditions.checkArgument(distAlongGeometry >= 0d);
    final LengthIndexedLine lengthIdxLine =
        new LengthIndexedLine(geometry);

    final LinearLocation lineLocation =
        LengthLocationMap.getLocation(geometry, distAlongGeometry);
    final LineSegment lineSegment = lineLocation.getSegment(geometry);
    final Coordinate startOfSegmentCoord = lineSegment.p0;
    final double positiveDistToStartOfSegmentOnGeometry =
        lengthIdxLine.indexOf(startOfSegmentCoord);

    double distanceToStartOfSegmentOnPath;
    distanceToStartOfSegmentOnPath =
        positiveDistToStartOfSegmentOnGeometry;

    return Maps.immutableEntry(lineSegment,
        distanceToStartOfSegmentOnPath);
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  public static Matrix getU() {
    return U;
  }

  public static Vector getZeros2d() {
    return zeros2D;
  }

  public static void invertProjection(MultivariateGaussian dist,
    InferredPath path) {
    Preconditions.checkArgument(!path.isEmptyPath());
    Preconditions.checkArgument(dist.getInputDimensionality() == 2
        || dist.getInputDimensionality() == 4);

    if (dist.getInputDimensionality() == 2) {
      /*
       * Convert to ground-coordinates
       */
      convertToGroundBelief(dist,
          path.getEdgeForDistance(dist.getMean().getElement(0), true));
    } else {
      /*
       * Convert to road-coordinates
       */
      convertToRoadBelief(dist, path);

    }
  }

  /**
   * Note: it's very important that the position be "normalized" relative to the
   * edge w.r.t. the velocity. That way, when we predict the next location, the
   * start position isn't relative to the wrong end of the edge, biasing our
   * measurements by the length of the edge in the wrong direction. E.g. {35,
   * -1} -> {-5, -1} for 30sec time diff., then relative to the origin edge we
   * will mistakenly evaluate a loop-around. Later, when evaluating likelihoods
   * on paths/path-edges, we'll need to re-adjust locally when path directions
   * don't line up.
   */
  public static void normalizeBelief(Vector mean, PathEdge edge) {

    Preconditions.checkArgument(edge.isOnEdge(mean.getElement(0)));

    final double desiredDirection;
    if (edge.getDistToStartOfEdge() == 0d) {
      desiredDirection = Math.signum(mean.getElement(1));
      /*
       * When the velocity is zero there's not way to normalize, other
       * than pure convention.
       */
      if (desiredDirection == 0d)
        return;
    } else {
      desiredDirection = Math.signum(edge.getDistToStartOfEdge());
    }

    if (Math.signum(mean.getElement(0)) != desiredDirection) {
      final double totalPosLength =
          edge.getInferredEdge().getLength()
              + Math.abs(edge.getDistToStartOfEdge());
      double newPosLocation =
          totalPosLength - Math.abs(mean.getElement(0));
      if (newPosLocation < 0d)
        newPosLocation = 0d;
      else if (newPosLocation > totalPosLength)
        newPosLocation = totalPosLength;

      final double newLocation = desiredDirection * newPosLocation;
      assert Double.compare(Math.abs(newLocation), totalPosLength) <= 0;
      assert edge.isOnEdge(newLocation);

      mean.setElement(0, newLocation);
    }
  }

  /**
   * TODO FIXME associate these values with segments? Returns the matrix and
   * offset vector for projection onto the given edge. distEnd is the distance
   * from the start of the path to the end of the given edge. NOTE: These
   * results are only in the positive direction. Convert on your end.
   */
  static protected Entry<Matrix, Vector> posVelProjectionPair(
    LineSegment lineSegment, double distToStartOfLine) {

    final Vector start = GeoUtils.getVector(lineSegment.p0);
    final Vector end = GeoUtils.getVector(lineSegment.p1);

    final double length = start.euclideanDistance(end);

    final double distToStart = Math.abs(distToStartOfLine);

    final Vector P1 = end.minus(start).scale(1 / length);
    final Vector s1 = start.minus(P1.scale(distToStart));

    final Matrix P = MatrixFactory.getDefault().createMatrix(4, 2);
    P.setColumn(0, P1.stack(zeros2D));
    P.setColumn(1, zeros2D.stack(P1));

    final Vector a = s1.stack(zeros2D);

    return Maps.immutableEntry(U.times(P), U.times(a));
  }

  /**
   * Use this sampling method to beat the inherent degeneracy of our state
   * covariance.
   * 
   * @param vehicleState
   * @return
   */
  public static Vector sampleMovementBelief(Random rng, Vector mean,
    AbstractRoadTrackingFilter filter) {
    final boolean isRoad = mean.getDimensionality() == 2;
    final Matrix Q = isRoad ? filter.getQr() : filter.getQg();

    final Matrix covSqrt =
        CholeskyDecompositionMTJ.create(
            DenseMatrixFactoryMTJ.INSTANCE.copyMatrix(Q)).getR();
    final Vector underlyingSample =
        MultivariateGaussian.sample(VectorFactory.getDefault()
            .createVector(covSqrt.getNumRows()), covSqrt, rng);
    final Matrix Gamma = filter.getCovarianceFactor(isRoad);
    final Vector thisStateSample =
        Gamma.times(underlyingSample).plus(mean);
    return thisStateSample;
  }

  public void setQr(Matrix qr) {
    assert DenseCholesky.factorize(
        ((AbstractMTJMatrix) qr).getInternalMatrix()).isSPD(); 
    Qr = qr;
  }

  public void setQg(Matrix qg) {
    assert DenseCholesky.factorize(
        ((AbstractMTJMatrix) qg).getInternalMatrix()).isSPD(); 
    Qg = qg;
  }

  public void setOnRoadStateVariance(Matrix onRoadStateVariance) {
    assert DenseCholesky.factorize(
        ((AbstractMTJMatrix) onRoadStateVariance).getInternalMatrix()).isSPD(); 
    this.onRoadStateVariance = onRoadStateVariance;
  }

  public void setOffRoadStateVariance(Matrix offRoadStateVariance) {
    assert DenseCholesky.factorize(
        ((AbstractMTJMatrix) offRoadStateVariance).getInternalMatrix()).isSPD(); 
    this.offRoadStateVariance = offRoadStateVariance;
  }

  public void setObsVariance(Matrix obsVariance) {
    assert DenseCholesky.factorize(
        ((AbstractMTJMatrix) obsVariance).getInternalMatrix()).isSPD(); 
    this.obsVariance = obsVariance;
  }

  public abstract void updateSufficientStatistics(Observation obs,
    VehicleState state, MultivariateGaussian sampledBelief,
    InferredPath sampledPath, Random rng);

}
