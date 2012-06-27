package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.math.ComplexNumber;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.CholeskyDecompositionMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.KalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.CloneableSerializable;

import java.util.Map.Entry;
import java.util.Random;

import javax.crypto.spec.PSource;

import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
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

public class StandardRoadTrackingFilter implements
    CloneableSerializable {

  /**
   * 
   */
  private static final long serialVersionUID = -3818533301279461087L;

  /**
   * Motion model of the underlying system.
   */
  private final LinearDynamicalSystem groundModel;
  private final KalmanFilter groundFilter;

  private final LinearDynamicalSystem roadModel;
  private final KalmanFilter roadFilter;

  private final Matrix Qr;
  private final Matrix Qg;

  private final Matrix onRoadStateVariance;
  private final Matrix offRoadStateVariance;
  private final Matrix obsVariance;

  /*
   * Observation matrix
   */
  private static Matrix Og;
  private static Matrix Or;
  private static Matrix U;

  /**
   * Standard 2D tracking model with the following state equation: {@latex[ D_
   * x_t = G x_ t-1} + A \epsilon_t} Also, when angle != null, a constraint
   * matrix is created for the state covariance, with perpendicular variance
   * a0Variance. aVariance doubles as both the x and y variances for
   * free-motion.
   * 
   * @param gVariance
   * @param aVariance
   * @param a0Variance
   * @param angle
   */
  public StandardRoadTrackingFilter(Vector obsVariance,
    Vector offRoadStateVariance, Vector onRoadStateVariance) {

    this.obsVariance = MatrixFactory.getDefault().createDiagonal(
        obsVariance);
    this.offRoadStateVariance = MatrixFactory.getDefault()
        .createDiagonal(offRoadStateVariance);
    this.onRoadStateVariance = MatrixFactory.getDefault()
        .createDiagonal(onRoadStateVariance);

    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel = new LinearDynamicalSystem(
        0, 2);
    final Matrix roadG = createStateTransitionMatrix(
        currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setB(MatrixFactory.getDefault().createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;

    this.Qr = MatrixFactory.getDefault().createDiagonal(
        onRoadStateVariance);
    this.roadFilter = new KalmanFilter(
        roadModel, createStateCovarianceMatrix(1d, Qr, true),
        this.obsVariance);

    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel = new LinearDynamicalSystem(
        0, 4);

    final Matrix groundGct = createStateTransitionMatrix(
        currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setB(MatrixFactory.getDefault().createIdentity(4, 4));
    groundModel.setC(Og);

    this.groundModel = groundModel;

    this.Qg = MatrixFactory.getDefault().createDiagonal(
        offRoadStateVariance);
    this.groundFilter = new KalmanFilter(
        groundModel, createStateCovarianceMatrix(1d, Qg, false),
        this.obsVariance);

  }

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

  private double currentTimeDiff = 1d;

  private double prevTimeDiff = 1d;

  private final static Vector zeros2D = VectorFactory.getDefault()
      .copyValues(0, 0);

  @Override
  public StandardRoadTrackingFilter clone() {
    StandardRoadTrackingFilter filter;
    try {
      filter = (StandardRoadTrackingFilter) super.clone();
      return filter;
    } catch (final CloneNotSupportedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return null;
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

  public double getObservationErrorAbsRadius() {
    final double varDistance = 1.98d * Math.sqrt(this
        .getObsVariance().normFrobenius());  
    return varDistance;
  }
  
  public KalmanFilter getGroundFilter() {
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

    final Matrix Q = Og.times(projBelief.getCovariance()).times(
        Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final MultivariateGaussian res = new MultivariateGaussian(
        Og.times(projBelief.getMean()), Q);
    return res;
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

  public KalmanFilter getRoadFilter() {
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
  public double logLikelihood(Vector obs,
    MultivariateGaussian belief, PathEdge edge) {
    final MultivariateGaussian projBelief = belief.clone();
    if (projBelief.getInputDimensionality() == 2) {
      convertToGroundBelief(projBelief, edge);
    }

    final Matrix Q = Og.times(projBelief.getCovariance()).times(
        Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final MultivariateGaussian.PDF pdf = new MultivariateGaussian.PDF(
        Og.times(projBelief.getMean()), Q);
    final double result = pdf.logEvaluate(obs);
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
      final PathEdge startEdge = path.getEdgeForDistance(belief.getMean().getElement(0), true);
      convertToGroundBelief(updatedBelief, startEdge);

      /*
       * Perform Kalman update
       */
      // Vector a = projBelief.getMean();
      // Matrix R = projBelief.getCovariance();
      // final Matrix Q = Og.times(R).times(Og.transpose())
      // .plus(groundFilter.getMeasurementCovariance());
      // final Matrix A =
      // Q.transpose().solve(Og.times(R.transpose())).transpose();
      // final Vector e = observation.minus(Og.times(a));
      //
      // final Matrix C = R.minus(A.times(Q.transpose()).times(A.transpose()));
      // final Vector m = a.plus(A.times(e));
      this.groundFilter.measure(updatedBelief, observation);

      /*
       * Convert back to road-coordinates
       */
      convertToRoadBelief(updatedBelief, path);
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
  public void predict(MultivariateGaussian belief, PathEdge edge,
    PathEdge prevEdge) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2
        || belief.getInputDimensionality() == 4);
    Preconditions.checkNotNull(edge);

    if (edge.isEmptyEdge()) {
      if (belief.getInputDimensionality() == 4) {
        /*-
         * Predict free-movement
         */
        groundFilter.predict(belief);
      } else {
        /*-
         * Going off-road
         */
        Preconditions.checkNotNull(prevEdge);
        convertToGroundBelief(belief, prevEdge);
        groundFilter.predict(belief);
      }
    } else {
      if (belief.getInputDimensionality() == 4) {
        /*-
         * Predict movement onto a path/edge.
         * Currently, this just consists of projecting onto
         * the edge
         * 
         */
        convertToRoadBelief(belief, InferredPath.getInferredPath(edge));
        StandardRoadTrackingFilter.normalizeBelief(belief.getMean(), edge);
        roadFilter.predict(belief);
      } else {
        StandardRoadTrackingFilter.normalizeBelief(belief.getMean(), edge);
        roadFilter.predict(belief);
      }
    }

  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    if (currentTimeDiff != prevTimeDiff) {
      groundFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, Qg, false));
      roadFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, Qr, true));

      groundModel.setA(createStateTransitionMatrix(
          currentTimeDiff, false));
      roadModel.setA(createStateTransitionMatrix(
          currentTimeDiff, true));
    }
    this.prevTimeDiff = this.currentTimeDiff;
    this.currentTimeDiff = currentTimeDiff;
  }
  
  /**
   * Use this sampling method to beat the inherent degeneracy of our state
   * covariance.
   * 
   * @param vehicleState
   * @return
   */
  public Vector sampleStateBelief(Vector mean, Random rng) {
    final boolean isRoad = mean.getDimensionality() == 2;
    final Matrix Q = isRoad ? this.getQr() : this.getQg();

    final Matrix covSqrt = CholeskyDecompositionMTJ.create(
        DenseMatrixFactoryMTJ.INSTANCE.copyMatrix(Q)).getR();
    Vector underlyingSample = MultivariateGaussian.sample(VectorFactory
        .getDefault().createVector(2), covSqrt, rng);
    final Matrix Gamma = this.getCovarianceFactor(isRoad);
    Vector thisStateSample = Gamma.times(underlyingSample).plus(mean);
    return thisStateSample;
  }
  
  @Override
  public String toString() {
    return "StandardRoadTrackingFilter ["
        + "groundFilterCov=" + groundFilter.getModelCovariance()
        + ", roadFilterCov="
        + roadFilter.getModelCovariance() + ", onRoadStateVariance="
        + onRoadStateVariance + ", offRoadStateVariance="
        + offRoadStateVariance + ", obsVariance=" + obsVariance
        + ", currentTimeDiff=" + currentTimeDiff + "]";
  }

  public static boolean checkPosDef(DenseMatrix covar) {
    final EigenDecompositionRightMTJ decomp = EigenDecompositionRightMTJ
        .create(covar);
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
    MultivariateGaussian belief, PathEdge edge, boolean allowExtensions) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2
        || belief.getInputDimensionality() == 4);

    if (belief.getInputDimensionality() == 4)
      return;

    Preconditions.checkArgument(!edge.isEmptyEdge());

    final Vector positiveMean;
    if (belief.getMean().getElement(0) < 0d) {
      /*
       * We're going all positive here, since we should've been using
       * the reversed geometry if negative.
       */
      final Vector posMeanTmp = belief.getMean().clone();
      
      final double adjNegLocation = belief.getMean().getElement(0) - edge.getDistToStartOfEdge();
      final double posLocation = edge.getInferredEdge().getLength() + adjNegLocation 
          + Math.abs(edge.getDistToStartOfEdge());
      posMeanTmp.setElement(0, posLocation);
      positiveMean = posMeanTmp;
    } else {
      positiveMean = belief.getMean().clone();
    }
    
    final Entry<LineSegment, Double> segmentDist = getSegmentAndDistanceToStart(
        edge, positiveMean.getElement(0));
    final double absTotalPathDistanceToStartOfSegment = Math.abs(segmentDist.getValue());
    final double absTotalPathDistanceToEndOfSegment = absTotalPathDistanceToStartOfSegment
        + segmentDist.getKey().getLength();
    final Entry<Matrix, Vector> projPair = StandardRoadTrackingFilter
        .posVelProjectionPair(
            segmentDist.getKey(), absTotalPathDistanceToStartOfSegment);
    
    if (!allowExtensions) {
  //    assert (Math.abs(belief.getMean().getElement(0)) <= absTotalPathDistanceToEndOfSegment 
  //        && Math.abs(belief.getMean().getElement(0)) >= absTotalPathDistanceToStartOfSegment);
      
      /*
       * Truncate, to keep it on the edge.
       */
      if (positiveMean.getElement(0) > absTotalPathDistanceToEndOfSegment) {
        positiveMean.setElement(0, absTotalPathDistanceToEndOfSegment);
      } else if (positiveMean.getElement(0) < absTotalPathDistanceToStartOfSegment) {
        positiveMean.setElement(0, absTotalPathDistanceToStartOfSegment);
      }
    }

    final Matrix C = belief.getCovariance();
    final Vector projMean = projPair.getKey().times(positiveMean)
        .plus(projPair.getValue());
    final Matrix projCov = projPair.getKey().times(C)
        .times(projPair.getKey().transpose());

    belief.setMean(projMean);
    belief.setCovariance(projCov);
  }

  public static void convertToRoadBelief(MultivariateGaussian belief,
    InferredPath path) {
    
    // TODO FIXME XXX make sure this works with the direction of motion.
    Preconditions.checkArgument(belief.getInputDimensionality() == 2
        || belief.getInputDimensionality() == 4);

    if (belief.getInputDimensionality() == 2)
      return;

    Preconditions.checkArgument(!path.isEmptyPath());

    final Vector m = belief.getMean().clone();
    final Matrix C = belief.getCovariance().clone();

    /*
     * We snap to the line and find the segment of interest.
     */
    final Coordinate currentPos = GeoUtils.makeCoordinate(Og.times(m));
    final LocationIndexedLine locIndex = new LocationIndexedLine(path.getGeometry());
    final LinearLocation lineLocation = locIndex.project(currentPos);
    
    // TODO necessary?
    final Coordinate pointOnLine = lineLocation.getCoordinate(path.getGeometry());
    m.setElement(0, pointOnLine.x);
    m.setElement(2, pointOnLine.y);
    
    /*
     * Get the segment we're projected onto, and the distance offset
     * of the path.
     */
    final LineSegment lineSegment = lineLocation.getSegment(path.getGeometry());
    final LengthIndexedLine lengthIndex = new LengthIndexedLine(path.getGeometry());
    final double distanceToStartOfSegmentOnGeometry = lengthIndex.indexOf(lineSegment.p0);
    
    final Entry<Matrix, Vector> projPair = StandardRoadTrackingFilter
        .posVelProjectionPair(
            lineSegment, distanceToStartOfSegmentOnGeometry);

    final Vector projMean = projPair.getKey().transpose()
        .times(m.minus(projPair.getValue()));
    
    if (path.isBackward())
      projMean.setElement(0, -1d * projMean.getElement(0));
    
    normalizeBelief(projMean, path.getEdgeForDistance(projMean.getElement(0), true));
    
    final Matrix projCov = projPair.getKey().transpose().times(C)
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
  private static Matrix createStateCovarianceMatrix(double timeDiff,
    Matrix Q, boolean isRoad) {

    final Matrix A_half = getCovarianceFactor(timeDiff, isRoad);
    final Matrix A = A_half.times(Q).times(A_half.transpose());

    return A;
  }

  private static Matrix createStateTransitionMatrix(double timeDiff,
    boolean isRoad) {

    final int dim;
    if (isRoad) {
      dim = 2;
    } else {
      dim = 4;
    }
    final Matrix Gct = MatrixFactory.getDefault().createIdentity(
        dim, dim);
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
    final Matrix A_half = MatrixFactory.getDefault().createMatrix(
        dim * 2, 2);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }

    return A_half;
  }

  /**
   * Returns the lineSegment in the geometry of the edge and the
   * distance-to-start of the segment on the entire path.
   * The line segment is in the direction of the edge's geometry,
   * and the distance-to-start has the same sign as the
   * direction of movement.
   * 
   * @param edge
   * @param distanceAlong
   * @return
   */
  public static Entry<LineSegment, Double> getSegmentAndDistanceToStart(
    PathEdge edge, double distanceAlong) {
    final Geometry geometry = edge.getInferredEdge().getGeometry();
    final LengthIndexedLine lengthIdxLine = edge.getInferredEdge()
        .getLengthIndexedLine();

    final double direction = distanceAlong >= 0d ? 1d : -1d;
    final double distAlongGeometry = distanceAlong
        - edge.getDistToStartOfEdge();
    final LinearLocation lineLocation = LengthLocationMap
        .getLocation(
            geometry,
            distAlongGeometry);
    final LineSegment lineSegment = lineLocation.getSegment(geometry);
    final Coordinate startOfSegmentCoord = direction < 0d ? lineSegment.p1 : lineSegment.p0;  
    final double positiveDistToStartOfSegmentOnGeometry = lengthIdxLine
            .indexOf(startOfSegmentCoord);
            
    double distanceToStartOfSegmentOnPath;
    if (direction < 0d) {
      distanceToStartOfSegmentOnPath = geometry.getLength() 
          - positiveDistToStartOfSegmentOnGeometry;
    } else {
      distanceToStartOfSegmentOnPath = positiveDistToStartOfSegmentOnGeometry;
    }
    distanceToStartOfSegmentOnPath = distanceToStartOfSegmentOnPath 
        + edge.getDistToStartOfEdge();

    return Maps.immutableEntry(
        lineSegment, distanceToStartOfSegmentOnPath);
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

  private static Matrix getRotatedCovarianceMatrix(double aVariance,
    double a0Variance, double angle) {

    final Matrix rotationMatrix = MatrixFactory.getDefault()
        .createIdentity(2, 2);
    rotationMatrix.setElement(0, 0, Math.cos(angle));
    rotationMatrix.setElement(0, 1, -Math.sin(angle));
    rotationMatrix.setElement(1, 0, Math.sin(angle));
    rotationMatrix.setElement(1, 1, Math.cos(angle));

    final Matrix temp = MatrixFactory.getDefault().createDiagonal(
        VectorFactory.getDefault().copyArray(
            new double[] { a0Variance, aVariance }));
    return rotationMatrix.times(temp).times(
        rotationMatrix.transpose());
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
      convertToGroundBelief(dist, path.getEdgeForDistance(dist.getMean().getElement(0), true));
    } else {
      /*
       * Convert to road-coordinates
       */
      convertToRoadBelief(dist, path);

    }
  }

  /**
   * TODO FIXME associate these values with segments?
   * Returns the matrix and offset vector for projection onto the given edge.
   * distEnd is the distance from the start of the path to the end of the given
   * edge. NOTE: These results are only in the positive direction. Convert on
   * your end.
   */
  static private Entry<Matrix, Vector> posVelProjectionPair(
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
   * Note: it's very important that the position be "normalized" relative
   * to the edge w.r.t. the velocity.  That way, when we predict the next
   * location, the start position isn't relative to the wrong end of the
   * edge, biasing our measurements by the length of the edge in the wrong
   * direction.  E.g. {35, -1} -> {-5, -1} for 30sec time diff., then 
   * relative to the origin edge we will mistakenly evaluate a loop-around.
   * Later, when evaluating likelihoods on paths/path-edges, we'll need to
   * re-adjust locally when path directions don't line up.
   */
  public static void normalizeBelief(Vector mean, PathEdge edge) {

    final double normalizedEdgeLoc = mean.getElement(0)
            - edge.getDistToStartOfEdge();
    
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
    
    if (Math.signum(normalizedEdgeLoc) == desiredDirection) {
      mean.setElement(
          0, normalizedEdgeLoc);
    } else {
      mean.setElement(
          0, desiredDirection * edge.getInferredEdge().getLength() 
            + normalizedEdgeLoc);
    }

    assert Double.compare(Math.abs(mean.getElement(0)), edge.getInferredEdge().getLength() + 1) <= 0; 
    
  }

}
