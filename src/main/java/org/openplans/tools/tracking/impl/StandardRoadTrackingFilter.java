package org.openplans.tools.tracking.impl;

import java.util.List;
import java.util.Map.Entry;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Maps;

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

  private final double gVariance;
  private final double aVariance;
  private final double a0Variance;

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
  public StandardRoadTrackingFilter(double gVariance, double aVariance,
    double a0Variance) {

    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel = new LinearDynamicalSystem(0, 2);
    final Matrix roadG = createStateTransitionMatrix(currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setB(MatrixFactory.getDefault().createIdentity(2, 2));
    roadModel.setC(Or);
    this.roadModel = roadModel;
    this.roadFilter = new KalmanFilter(roadModel,
        createStateCovarianceMatrix(1d, aVariance, a0Variance, true),
        MatrixFactory.getDefault().createIdentity(2, 2).scale(gVariance));
    
    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel = new LinearDynamicalSystem(0, 4);

    final Matrix groundGct = createStateTransitionMatrix(currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setB(MatrixFactory.getDefault().createIdentity(4, 4));
    groundModel.setC(Og);

    this.groundModel = groundModel;
    
    this.groundFilter = new KalmanFilter(groundModel,
        createStateCovarianceMatrix(1d, aVariance, aVariance, false),
        MatrixFactory.getDefault().createIdentity(2, 2).scale(gVariance));

    this.aVariance = aVariance;
    this.gVariance = gVariance;
    this.a0Variance = a0Variance;

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

  public double getgVariance() {
    return gVariance;
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
      Entry<Matrix, Vector> projPair = posVelProjectionPair(edge);
      /*
       * Convert road-coordinates prior predictive to ground-coordinates
       */
  //    Vector a = projPair.getKey().times(belief.getMean()).plus(projPair.getValue());
      Matrix R = projPair.getKey().times(belief.getCovariance()).times(projPair.getKey().transpose());
      
      final Matrix Q = Og.times(R).times(Og.transpose())
          .plus(groundFilter.getMeasurementCovariance());
      // FIXME TODO use solve
      final Matrix A = R.times(Og.transpose()).times(Q.inverse());
      final Vector e = observation.minus(Og.times(belief.getMean()));
      
      final Matrix C = belief.getCovariance().minus(A.times(Q.transpose()).times(A.transpose()));
      final Vector m = belief.getMean().plus(A.times(e));
      
      belief.setMean(projPair.getKey().transpose().times(m.minus(projPair.getValue())));
      belief.setCovariance(projPair.getKey().transpose().times(C).times(projPair.getKey()));
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
    
    Entry<Matrix, Vector> projPair = posVelProjectionPair(edge);
    if (dist.getInputDimensionality() == 2) {
      /*
       * Convert to ground-coordinates 
       */
      Vector a = projPair.getKey().times(dist.getMean()).plus(projPair.getValue());
      Matrix R = projPair.getKey().times(dist.getCovariance()).times(projPair.getKey().transpose());
      dist.setCovariance(R);
      dist.setMean(a);
    } else {
      /*
       * Convert to road-coordinates 
       */
      Vector m = projPair.getKey().transpose().times(dist.getMean().minus(projPair.getValue()));
      Matrix C = projPair.getKey().transpose().times(dist.getCovariance()).times(projPair.getKey());
      dist.setCovariance(C);
      dist.setMean(m);
      
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
  public void predict(MultivariateGaussian belief, PathEdge edge) {
    Preconditions.checkArgument(belief.getInputDimensionality() == 2 ||
        belief.getInputDimensionality() == 4);

    if (edge == PathEdge.getEmptyPathEdge()) {
      Preconditions.checkArgument(belief.getInputDimensionality() == 4); 
      /*
       * Predict free-movement
       */
      groundFilter.predict(belief);
    } else {
      if (belief.getInputDimensionality() == 4) {
        /*-
         * Predict movement onto a path/edge.
         * Currently, this just consists of projecting onto
         * the edge
         * 
         * TODO FIXME this is a temporary hack.  check this.
         */
        invertProjection(belief, edge);
        
      } else {
        if (edge == null) {
          /*-
           * Not looking for a prediction on an edge, instead
           * the projection along the distance.
           */
          roadFilter.predict(belief);
          
        } else {
          /*
           * Predict movement along a path
           */
          final double S = Or.times(belief.getCovariance()).times(Or.transpose()).getElement(0, 0) 
              + Math.pow(edge.getInferredEdge().getLength()/Math.sqrt(12), 2);
          final Matrix W = belief.getCovariance().times(Or.transpose()).scale(1/S);
          final Matrix R = belief.getCovariance().minus(W.times(W.transpose()).scale(S));
          final double e = edge.getDistToStartOfEdge() + edge.getInferredEdge().getLength()/2d 
              - Or.times(belief.getMean()).getElement(0);
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
    Vector a;
    Matrix R;
    Matrix O;
    Matrix V;
    if (belief.getInputDimensionality() == 2) {
      Entry<Matrix, Vector> projPair = posVelProjectionPair(edge);
      a = projPair.getKey().times(belief.getMean()).plus(projPair.getValue());
      R = projPair.getKey().times(belief.getCovariance()).times(projPair.getKey().transpose());
      O = this.roadModel.getC();
      V = this.roadFilter.getMeasurementCovariance();
    } else {
      a = belief.getMean();
      R = belief.getCovariance();
      O = this.groundModel.getC();
      V = this.groundFilter.getMeasurementCovariance();
    }
    
    Matrix Q = O.times(R).times( O.transpose() );
    Q.plusEquals(V);
    
    MultivariateGaussian.PDF pdf = new MultivariateGaussian.PDF(
        O.times(a), Q);
    return pdf.logEvaluate(obs);
  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    if (currentTimeDiff != prevTimeDiff) {
      groundFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, aVariance, aVariance, false));
      roadFilter.setModelCovariance(createStateCovarianceMatrix(
          currentTimeDiff, aVariance, a0Variance, true));
 
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
    double yaVariance, double xa0Variance, boolean isRoad) {
    final int dim;
    final Matrix Q;
    if (!isRoad) {
      dim = 2;
      Q = MatrixFactory.getDefault().createDiagonal(
        VectorFactory.getDefault().copyArray(
            new double[] { xa0Variance, yaVariance }));
    } else {
//      Q = getRotatedCovarianceMatrix(yaVariance, xa0Variance, angle);
      Q = MatrixFactory.getDefault().createDiagonal(
        VectorFactory.getDefault().copyArray(
            new double[] { xa0Variance, yaVariance }));
      dim = 1;
    }
    final Matrix A_half = MatrixFactory.getDefault().createMatrix(dim*2, 2);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }
    
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
   * Note: there is no distance from start of edge, so you'll need to adjust the 
   * returned vector.
   * @param edge
   * @param distEnd
   * @return
   */
  static public Entry<Matrix, Vector> posVelProjectionPair(PathEdge edge) {
    final Vector start = edge.getInferredEdge().getStartPoint();
    final Vector end = edge.getInferredEdge().getEndPoint();
    
    final double length = edge.getInferredEdge().getLength();
    
    final Vector P1 = start.minus(end).scale(1/length);
    final Vector s1 = start.minus(P1.scale(edge.getDistToStartOfEdge()));
    
    final Matrix P = MatrixFactory.getDefault().createMatrix(4, 2);
    P.setColumn(0, P1.stack(zeros2D));
    P.setColumn(2, zeros2D.stack(P1));
    
    final Vector a = s1.stack(zeros2D);
    
    return Maps.immutableEntry(U.times(P), U.times(a));
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

  public double getaVariance() {
    return aVariance;
  }

  public double getA0Variance() {
    return a0Variance;
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
}
















