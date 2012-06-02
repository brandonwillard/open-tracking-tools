package org.openplans.tools.tracking.impl;

import java.util.List;
import java.util.Map.Entry;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.GeodeticCalculator;
import org.opengis.referencing.operation.TransformException;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LinearLocation;

import gov.sandia.cognition.math.ComplexNumber;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.bayesian.KalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.CloneableSerializable;

public class StandardRoadTrackingFilter implements CloneableSerializable {

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

  private Vector onRoadStateVariance;
  private Vector offRoadStateVariance;
  private Vector obsVariance;

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
   * a0Variance.
   * aVariance doubles as both the x and y variances for free-motion.
   * 
   * @param gVariance
   * @param aVariance
   * @param a0Variance
   * @param angle
   */
  public StandardRoadTrackingFilter(Vector obsVariance,
    Vector offRoadStateVariance, Vector onRoadStateVariance) {
    
    this.obsVariance = obsVariance;
    this.offRoadStateVariance = offRoadStateVariance;
    this.onRoadStateVariance = onRoadStateVariance;
    
    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel = new LinearDynamicalSystem(0, 2);
    final Matrix roadG = createStateTransitionMatrix(currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setB(MatrixFactory.getDefault().createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;
    
    this.Qr = MatrixFactory.getDefault().createDiagonal(onRoadStateVariance);
    this.roadFilter = new KalmanFilter(roadModel,
        createStateCovarianceMatrix(1d, Qr, true),
        MatrixFactory.getDefault().createDiagonal(obsVariance));
    
    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel = new LinearDynamicalSystem(0, 4);

    final Matrix groundGct = createStateTransitionMatrix(currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setB(MatrixFactory.getDefault().createIdentity(4, 4));
    groundModel.setC(Og);

    this.groundModel = groundModel;
    
    this.Qg = MatrixFactory.getDefault().createDiagonal(offRoadStateVariance);
    this.groundFilter = new KalmanFilter(groundModel,
        createStateCovarianceMatrix(1d, Qg, false),
        MatrixFactory.getDefault().createDiagonal(obsVariance));


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

  @Override
  public StandardRoadTrackingFilter clone() {
    StandardRoadTrackingFilter filter;
    try {
      filter = (StandardRoadTrackingFilter) super
          .clone();
      return filter;
    } catch (CloneNotSupportedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    return null;
  }

  public MultivariateGaussian createInitialLearnedObject() {
    return new MultivariateGaussian(groundFilter.getModel().getState(),
        groundFilter.getModelCovariance());
  }

  public double getCurrentTimeDiff() {
    return currentTimeDiff;
  }
  
  /**
   * Updates the road-coordinates prior predictive belief to the posterior 
   * for the given observation, edge and path distance to the start of the edge.
   * @param belief
   * @param observation
   * @param edge
   */
  public void measure(MultivariateGaussian belief, Vector observation, PathEdge edge) {
    
    if (belief.getInputDimensionality() == 2) {
      /*
       * Convert road-coordinates prior predictive to ground-coordinates
       */
      MultivariateGaussian projBelief = belief.clone();
      invertProjection(projBelief, edge);
      Vector a = projBelief.getMean();
      Matrix R = projBelief.getCovariance();
      
      final Matrix Q = Og.times(R).times(Og.transpose())
          .plus(groundFilter.getMeasurementCovariance());
      final Matrix A = Q.transpose().solve(Og.times(R.transpose())).transpose();
          //R.times(Og.transpose()).times(Q.inverse());
      final Vector e = observation.minus(Og.times(a));
      
      final Matrix C = R.minus(A.times(Q.transpose()).times(A.transpose()));
      final Vector m = a.plus(A.times(e));
      
      MultivariateGaussian roadPost = projBelief.clone();
      invertProjection(roadPost, edge);
      Preconditions.checkArgument(roadPost.getMean().getElement(0) >= 0);
      belief.setMean(roadPost.getMean());
      belief.setCovariance(roadPost.getCovariance());
    } else {
      Preconditions.checkArgument(belief.getInputDimensionality() == 4);
      this.groundFilter.measure(belief, observation);
    }

  }

  public static void invertProjection(MultivariateGaussian dist, PathEdge edge) {
    Preconditions.checkNotNull(edge);
    Preconditions.checkArgument(edge != PathEdge.getEmptyPathEdge());
    Preconditions.checkArgument(dist.getInputDimensionality() == 2 ||
        dist.getInputDimensionality() == 4);
    
    if (dist.getInputDimensionality() == 2) {
      /*
       * Convert to ground-coordinates 
       */
      convertToGroundBelief(dist, edge);
    } else {
      /*
       * Convert to road-coordinates 
       */
      convertToRoadBelief(dist, edge);
      
    }
  }
  
  /**
   * Pass it a road-coordinates prior predictive belief distribution, edge and 
   * path starting distance, and it will update the prior predictive distribution 
   * for that edge and path.  Otherwise, project free-movement onto an edge or 
   * predict free movement.
   * @param belief
   * @param edge
   * @param startOfEdgeDist 
   */
  public void predict(MultivariateGaussian belief, PathEdge edge, PathEdge prevEdge) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2 ||
        belief.getInputDimensionality() == 4);

    if (edge == PathEdge.getEmptyPathEdge()) {
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
        invertProjection(belief, edge);
        roadFilter.predict(belief);
      } else {
        if (edge == null) {
          /*-
           * Not looking for a prediction on an edge, instead
           * the projection along the distance.
           */
          roadFilter.predict(belief);
          
        } else {
          /*-
           * Predict movement along an edge 
           * TODO really, this should just be the truncated/conditional
           * mean and covariance for the given interval/edge
           */
          final double S = Or.times(belief.getCovariance()).times(Or.transpose()).getElement(0, 0) 
              + Math.pow(edge.getInferredEdge().getLength()/Math.sqrt(12), 2);
          final Matrix W = belief.getCovariance().times(Or.transpose()).scale(1/S);
          final Matrix R = belief.getCovariance().minus(W.times(W.transpose()).scale(S));
          final double mean = (edge.getDistToStartOfEdge() + edge.getInferredEdge().getLength())/2d;
          final double e = mean - Or.times(belief.getMean()).getElement(0);
          final Vector a = belief.getMean().plus(W.getColumn(0).scale(e));
          
          belief.setMean(a);
          belief.setCovariance(R);
        }
      }
    }
    
  }
  
  /**
   * @param obs
   * @param belief
   * @return
   */
  public double logLikelihood(Vector obs, MultivariateGaussian belief, PathEdge edge) {
    MultivariateGaussian projBelief = belief.clone();
    if (projBelief.getInputDimensionality() == 2) {
      invertProjection(projBelief, edge);
    } 
    
    Matrix Q = Og.times(projBelief.getCovariance()).times( Og.transpose() );
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());
    
    MultivariateGaussian.PDF pdf = new MultivariateGaussian.PDF(
        Og.times(projBelief.getMean()), Q);
    final double result = pdf.logEvaluate(obs);
    return result;
  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    if (currentTimeDiff != prevTimeDiff) {
      groundFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, Qg, false));
      roadFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, Qr, true));
 
      groundModel.setA(createStateTransitionMatrix(currentTimeDiff, false));
      roadModel.setA(createStateTransitionMatrix(currentTimeDiff, true));
    }
    this.prevTimeDiff = this.currentTimeDiff;
    this.currentTimeDiff = currentTimeDiff;
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

  public Matrix getCovarianceFactor(boolean isRoad) {
    return getCovarianceFactor(this.currentTimeDiff, isRoad);
  }
  
  public static Matrix getCovarianceFactor(double timeDiff, boolean isRoad) {
    
    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half = MatrixFactory.getDefault().createMatrix(dim*2, 2);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }
    
    return A_half;
  }
  
  /**
   * Creates either a diagonal matrix with diag = xa0Variance, yaVariance,
   * or the aforementioned matrix rotated by the x-axis angle angle, with
   * xa0Variance, yaVariance the perpendicular, parallel variances.
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

  private static Matrix createStateTransitionMatrix(double timeDiff, boolean isRoad) {

    final int dim;
    if (isRoad) {
      dim = 2;
    } else {
      dim = 4;
    }
    final Matrix Gct = MatrixFactory.getDefault().createIdentity(dim, dim);
    Gct.setElement(0, 1, timeDiff);
    if (dim > 2)
      Gct.setElement(2, 3, timeDiff);

    return Gct;
  }

  public static Matrix getGroundObservationMatrix() {
    return Og;
  }
  
  public static Matrix getRoadObservationMatrix() {
    return Or;
  }

  private static Matrix getRotatedCovarianceMatrix(double aVariance,
    double a0Variance, double angle) {

    final Matrix rotationMatrix = MatrixFactory.getDefault().createIdentity(2,
        2);
    rotationMatrix.setElement(0, 0, Math.cos(angle));
    rotationMatrix.setElement(0, 1, -Math.sin(angle));
    rotationMatrix.setElement(1, 0, Math.sin(angle));
    rotationMatrix.setElement(1, 1, Math.cos(angle));

    final Matrix temp = MatrixFactory.getDefault().createDiagonal(
        VectorFactory.getDefault().copyArray(
            new double[] { a0Variance, aVariance }));
    return rotationMatrix.times(temp).times(rotationMatrix.transpose());
  }

  private final static Vector zeros2D = VectorFactory.getDefault().copyValues(0,0);
  
  /**
   * Returns the matrix and offset vector for projection onto the given edge.
   * distEnd is the distance from the start of the path to the end of the given edge.
   * NOTE: These results are only in the positive direction.  Convert on your end.
   */
  static private Entry<Matrix, Vector> posVelProjectionPair(LineSegment lineSegment, double distToStartOfLine) {
    
    final Vector start = GeoUtils.getEuclideanVector(
        GeoUtils.reverseCoordinates(lineSegment.p0));
    final Vector end = GeoUtils.getEuclideanVector(
        GeoUtils.reverseCoordinates(lineSegment.p1));
    
    final double length = GeoUtils.getAngleDegreesInMeters(lineSegment.getLength());
    
    final double distToStart = Math.abs(distToStartOfLine);
    
    final Vector P1 = end.minus(start).scale(1/length);
    final Vector s1 = start.minus(P1.scale(distToStart));
    
    final Matrix P = MatrixFactory.getDefault().createMatrix(4, 2);
    P.setColumn(0, P1.stack(zeros2D));
    P.setColumn(1, zeros2D.stack(P1));
    
    final Vector a = s1.stack(zeros2D);
    
    return Maps.immutableEntry(U.times(P), U.times(a));
  }

  public static Vector getTruncatedEdgeLocation(Vector mean, PathEdge edge) {
    Preconditions.checkArgument(mean.getDimensionality() == 2);
    
    Vector truncatedMean = mean.clone();
    final double totalPathDistance = Math.abs(edge.getDistToStartOfEdge())
        + edge.getInferredEdge().getLength();
    
    final double sign = Math.signum(mean.getElement(0));
    final double distance = Math.abs(mean.getElement(0));
    
    if (distance > totalPathDistance) {
      truncatedMean.setElement(0, sign*totalPathDistance);
    } else if (distance < Math.abs(edge.getDistToStartOfEdge())) {
      truncatedMean.setElement(0, sign*edge.getDistToStartOfEdge());
    } 
    return truncatedMean;
  }
  
  public static void convertToGroundBelief(MultivariateGaussian belief, PathEdge edge) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2 ||
        belief.getInputDimensionality() == 4);
    
    if (belief.getInputDimensionality() == 4)
      return;
    
    Preconditions.checkArgument(edge != PathEdge.getEmptyPathEdge());
    
    final double distanceAlong = belief.getMean().getElement(0);
    final boolean isNegative = distanceAlong < 0d;
    LinearLocation lineLocation = edge.getInferredEdge().getLengthLocationMap().getLocation(
        GeoUtils.getMetersInAngleDegrees(distanceAlong));
    LineSegment lineSegment = lineLocation.getSegment(edge.getInferredEdge().getGeometry());
    final double distanceToStartOfSegment = GeoUtils.getAngleDegreesInMeters(
        edge.getInferredEdge().getLengthIndexedLine().indexOf(lineSegment.p0));
    if (isNegative)
      lineSegment.reverse();
    Entry<Matrix, Vector> projPair = StandardRoadTrackingFilter.posVelProjectionPair(lineSegment,
        distanceToStartOfSegment);
    /*
     * First, we convert to a positive distance traveled.
     */
    Vector truncatedMean = getTruncatedEdgeLocation(belief.getMean(), edge);
    
    if (isNegative) {
      truncatedMean.setElement(0, Math.abs(truncatedMean.getElement(0)));
    }
    
    final Matrix C = belief.getCovariance();
    final Vector projMean = projPair.getKey().times(truncatedMean).plus(projPair.getValue());
    final Matrix projCov = projPair.getKey().times(C).times(projPair.getKey().transpose());
    
    belief.setMean(projMean);
    belief.setCovariance(projCov);
  }
  
  public static void convertToRoadBelief(MultivariateGaussian belief, PathEdge edge) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2 ||
        belief.getInputDimensionality() == 4);
    
    if (belief.getInputDimensionality() == 2)
      return;
    
    Preconditions.checkArgument(edge != PathEdge.getEmptyPathEdge());
    
    final Vector m = belief.getMean().clone();
    final Matrix C = belief.getCovariance().clone();
    
    /*
     * We snap to the line and find the segment of interest.
     */
    final Coordinate latlonCurrentPos = GeoUtils.convertToLatLon(Og.times(m));
    LinearLocation lineLocation = edge.getInferredEdge().getLocationIndexedLine().project(
        GeoUtils.reverseCoordinates(latlonCurrentPos));
    LineSegment lineSegment = lineLocation.getSegment(edge.getInferredEdge().getGeometry());
    final double distanceToStartOfSegment = GeoUtils.getAngleDegreesInMeters(
        edge.getInferredEdge().getLengthIndexedLine().indexOf(lineSegment.p0));
    Entry<Matrix, Vector> projPair = StandardRoadTrackingFilter.posVelProjectionPair(lineSegment, distanceToStartOfSegment);
    
    final Vector projMean = projPair.getKey().transpose().times(m.minus(projPair.getValue()));
    final Matrix projCov = projPair.getKey().transpose().times(C).times(projPair.getKey());
  
    belief.setMean(projMean);
    belief.setCovariance(projCov);
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  public LinearDynamicalSystem getGroundModel() {
    return groundModel;
  }

  public KalmanFilter getGroundFilter() {
    return groundFilter;
  }

  public LinearDynamicalSystem getRoadModel() {
    return roadModel;
  }

  public KalmanFilter getRoadFilter() {
    return roadFilter;
  }

  public static Matrix getOg() {
    return Og;
  }

  public static Matrix getOr() {
    return Or;
  }

  public static Matrix getU() {
    return U;
  }

  public double getPrevTimeDiff() {
    return prevTimeDiff;
  }

  public static Vector getZeros2d() {
    return zeros2D;
  }

  public MultivariateGaussian getObservationBelief(final MultivariateGaussian belief, PathEdge edge) {
    MultivariateGaussian projBelief = belief.clone();
    if (projBelief.getInputDimensionality() == 2) {
      invertProjection(projBelief, edge);
    } 
    
    Matrix Q = Og.times(projBelief.getCovariance()).times( Og.transpose() );
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());
    
    MultivariateGaussian res = new MultivariateGaussian(Og.times(projBelief.getMean()), Q);
    return res;
  }

  public Matrix getQr() {
    return Qr;
  }

  public Matrix getQg() {
    return Qg;
  }

  public Vector getOnRoadStateVariance() {
    return onRoadStateVariance;
  }

  public Vector getOffRoadStateVariance() {
    return offRoadStateVariance;
  }

  public Vector getObsVariance() {
    return obsVariance;
  }

  @Override
  public String toString() {
    return "StandardRoadTrackingFilter [groundModel=" + groundModel
        + ", groundFilter=" + groundFilter.getModelCovariance() 
        + ", roadModel=" + roadModel
        + ", roadFilter=" + roadFilter.getModelCovariance()
        + ", onRoadStateVariance=" + onRoadStateVariance 
        + ", offRoadStateVariance=" + offRoadStateVariance 
        + ", obsVariance=" + obsVariance
        + ", currentTimeDiff=" + currentTimeDiff + "]";
  }

}
















