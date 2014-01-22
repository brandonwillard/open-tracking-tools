package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.AbstractBatchAndIncrementalLearner;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;
import gov.sandia.cognition.math.matrix.mtj.decomposition.SingularValueDecompositionMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.RecursiveBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.statslibextensions.math.matrix.SvdMatrix;
import com.statslibextensions.math.matrix.decomposition.SimpleSingularValueDecomposition;
import com.statslibextensions.statistics.distribution.SvdMultivariateGaussian;

/**
 * This is an improved (computationally) filter based on the Sandia KalmanFilter
 * class.
 * 
 * @author bwillard
 * 
 */
public class TruncatedRoadKalmanFilter
    extends
    AbstractBatchAndIncrementalLearner<Vector, SvdMultivariateGaussian>
    implements
    RecursiveBayesianEstimator<Vector, Vector, SvdMultivariateGaussian> {

  /**
   * Default autonomous dimension, {@value} .
   */
  public static final int DEFAULT_DIMENSION = 1;

  private static final long serialVersionUID = 8046227346384488242L;

  protected SvdMatrix measurementCovariance;

  /**
   * Motion model of the underlying system.
   */
  protected LinearDynamicalSystem model;
  protected SvdMatrix modelCovariance;
  protected double timeDiff;

  /**
   * Creates a new instance of LinearUpdater
   * 
   * @param model
   *          Motion model of the underlying system.
   * @param modelCovariance
   *          Covariance associated with the system's model.
   * @param measurementCovariance
   *          Covariance associated with the measurements.
   */
  public TruncatedRoadKalmanFilter(LinearDynamicalSystem model,
    SvdMatrix modelCovariance, SvdMatrix measurementCovariance,
    double timeDiff) {
    this.timeDiff = timeDiff;
    this.measurementCovariance = measurementCovariance;
    this.modelCovariance = modelCovariance;
    this.setModel(model);
  }

  @Override
  public TruncatedRoadKalmanFilter clone() {
    final TruncatedRoadKalmanFilter clone =
        (TruncatedRoadKalmanFilter) super.clone();
    clone.model = this.model.clone();
    clone.measurementCovariance =
        ObjectUtil.cloneSmart(this.measurementCovariance);
    clone.modelCovariance =
        ObjectUtil.cloneSmart(this.modelCovariance);
    return clone;
  }

  @Override
  public SvdMultivariateGaussian createInitialLearnedObject() {
    final Matrix subM =
        MatrixFactory.getDefault().copyArray(
            new double[][] {
                { 1d, 1d / this.timeDiff },
                { 1d / this.timeDiff,
                    2 / (this.timeDiff * this.timeDiff) } });
    final Matrix Wtmp =
        MatrixFactory.getDefault().createMatrix(
            this.modelCovariance.getNumRows(),
            this.modelCovariance.getNumColumns());
    Wtmp.setSubMatrix(0, 0,
        subM.scale(this.measurementCovariance.getElement(0, 0)));
    if (Wtmp.getNumColumns() == 4) {
      Wtmp.setSubMatrix(0, 2,
          subM.scale(this.measurementCovariance.getElement(0, 1)));
      Wtmp.setSubMatrix(2, 0,
          subM.scale(this.measurementCovariance.getElement(0, 1)));
      Wtmp.setSubMatrix(2, 2,
          subM.scale(this.measurementCovariance.getElement(1, 1)));
    }
    final AbstractSingularValueDecomposition svdW =
        SingularValueDecompositionMTJ.create(Wtmp);
    final SvdMatrix initialW =
        new SvdMatrix(new SimpleSingularValueDecomposition(
            svdW.getU(), svdW.getS(), svdW.getU().transpose()));
    return new TruncatedRoadGaussian(this.model.getState().clone(),
        initialW);
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final TruncatedRoadKalmanFilter other =
        (TruncatedRoadKalmanFilter) obj;
    if (this.model == null) {
      if (other.model != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(
        this.model.convertToVector(), other.model.convertToVector())) {
      return false;
    }
    return true;
  }

  public SvdMatrix getMeasurementCovariance() {
    return this.measurementCovariance;
  }

  /**
   * Getter for model
   * 
   * @return Motion model of the underlying system.
   */
  public LinearDynamicalSystem getModel() {
    return this.model;
  }

  public SvdMatrix getModelCovariance() {
    return this.modelCovariance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((this.model == null) ? 0 : StatisticsUtil
                .hashCodeVector(this.model.convertToVector()));
    return result;
  }

  public void
      measure(MultivariateGaussian belief, Vector observation) {

    Preconditions.checkArgument(StatisticsUtil
        .isPosSemiDefinite(belief.getCovariance()));
    final Matrix F = this.model.getC();

    AbstractSingularValueDecomposition svdR;
    if (belief instanceof SvdMultivariateGaussian) {
      svdR =
          ((SvdMultivariateGaussian) belief).getCovariance().getSvd();
    } else {
      svdR =
          SingularValueDecompositionMTJ
              .create(belief.getCovariance());
    }

    final Matrix NvInv =
        StatisticsUtil.diagonalInverse(
            StatisticsUtil.getDiagonalSqrt(this.measurementCovariance
                .getSvd().getS(), 1e-7), 1e-7).times(
            this.measurementCovariance.getSvd().getU().transpose());
    final Matrix NvFU = NvInv.times(F).times(svdR.getU());
    final Matrix SRinv =
        StatisticsUtil.diagonalInverse(
            StatisticsUtil.getDiagonalSqrt(svdR.getS(), 1e-7), 1e-7);
    final int nN2 = NvFU.getNumRows() + SRinv.getNumRows();
    final int nM2 = SRinv.getNumColumns();
    final Matrix M2 =
        MatrixFactory.getDefault().createMatrix(nN2, nM2);
    M2.setSubMatrix(0, 0, NvFU);
    M2.setSubMatrix(NvFU.getNumRows(), 0, SRinv);

    final AbstractSingularValueDecomposition svdM2 =
        SingularValueDecompositionMTJ.create(M2);
    final Matrix S =
        MatrixFactory.getDefault().createMatrix(
            svdM2.getS().getNumColumns(),
            svdM2.getS().getNumColumns());
    for (int i = 0; i < Math.min(svdM2.getS().getNumColumns(), svdM2
        .getS().getNumRows()); i++) {
      final double sVal = svdM2.getS().getElement(i, i);
      final double sValInvSq = 1d / (sVal * sVal);
      if (sValInvSq > 1e-7) {
        S.setElement(i, i, sValInvSq);
      }
    }
    final Matrix UcNew =
        svdR.getU().times(svdM2.getVtranspose().transpose());
    final AbstractSingularValueDecomposition svdCnew =
        new SimpleSingularValueDecomposition(UcNew, S,
            UcNew.transpose());

    Preconditions.checkArgument(StatisticsUtil
        .isPosSemiDefinite(UcNew.times(S).times(UcNew.transpose())));

    final SvdMatrix Q =
        StatisticsUtil.symmetricSvdAdd(
            (SvdMatrix) belief.getCovariance(),
            this.measurementCovariance, F);
    final Matrix Qinv =
        Q.getSvd()
            .getU()
            .times(
                StatisticsUtil.diagonalInverse(Q.getSvd().getS(),
                    1e-7)).times(Q.getSvd().getU().transpose());
    Preconditions.checkArgument(StatisticsUtil
        .isPosSemiDefinite(Qinv));
    final Vector e = observation.minus(F.times(belief.getMean()));

    final Matrix A =
        belief.getCovariance().times(F.transpose()).times(Qinv);
//    /*
//     * Note: for exact location/velocity correlation, the velocity
//     * term(s) in A should be <= 1/30. 
//     * DEBUG REMOVE
//     */
//    if (A.getNumColumns() == 1 && A.getElement(1, 0) > 1d / 30d) {
//      System.out.println("amplifying velocity noise!");
//    }

    final Vector postMean = belief.getMean().plus(A.times(e));

    //      final Vector a = belief.getMean();
    //      final Matrix R = belief.getCovariance();
    //      final Matrix Q =
    //          F.times(R).times(F.transpose())
    //              .plus(this.getMeasurementCovariance());
    //      /*
    //       * This is the source of one major improvement:
    //       * uses the solve routine for a positive definite matrix
    //       */
    //      final UpperSPDDenseMatrix Qspd =
    //          new UpperSPDDenseMatrix(
    //              ((AbstractMTJMatrix) Q).getInternalMatrix(), false);
    //      final no.uib.cipr.matrix.Matrix CRt =
    //          ((AbstractMTJMatrix) F.times(R.transpose()))
    //              .getInternalMatrix();
    //  
    //      final DenseMatrix Amtj =
    //          new DenseMatrix(Qspd.numRows(), CRt.numColumns());
    //      Qspd.transSolve(CRt, Amtj);
    //  
    //      final DenseMatrix AtQt =
    //          new DenseMatrix(Amtj.numColumns(), Qspd.numRows());
    //      Amtj.transABmult(Qspd, AtQt);
    //  
    //      final DenseMatrix AtQtAMtj =
    //          new DenseMatrix(AtQt.numRows(), Amtj.numColumns());
    //      AtQt.mult(Amtj, AtQtAMtj);
    //  
    //      final Matrix AtQtA =
    //          ((DenseMatrixFactoryMTJ) MatrixFactory.getDenseDefault())
    //              .createWrapper(AtQtAMtj);
    //  
    //      final DenseVector e2 =
    //          new DenseVector(
    //              ((gov.sandia.cognition.math.matrix.mtj.DenseVector) observation
    //                  .minus(F.times(a))).getArray(), false);
    //  
    //      final DenseVector AteMtj = new DenseVector(Amtj.numColumns());
    //      Amtj.transMult(e2, AteMtj);
    //      final Vector Ate =
    //          ((DenseVectorFactoryMTJ) VectorFactory.getDenseDefault())
    //              .createWrapper(AteMtj);
    //  
    //      final Matrix CC = R.minus(AtQtA);
    //      final Vector m = a.plus(Ate);
    //  
    //      assert StatisticsUtil
    //          .isPosSemiDefinite((gov.sandia.cognition.math.matrix.mtj.DenseMatrix) CC);
    //  
    //      belief.setCovariance(CC);
    //      belief.setMean(m);

    belief.setMean(postMean);
    if (belief instanceof SvdMultivariateGaussian) {
      ((SvdMultivariateGaussian) belief).getCovariance().setSvd(
          svdCnew);
    } else {
      belief.setCovariance(svdCnew.getU().times(svdCnew.getS())
          .times(svdCnew.getVtranspose()));
    }
  }

  public void predict(MultivariateGaussian belief) {

    final Matrix G = this.model.getA();
    AbstractSingularValueDecomposition svdC;
    if (belief instanceof SvdMultivariateGaussian) {
      svdC =
          ((SvdMultivariateGaussian) belief).getCovariance().getSvd();
    } else {
      svdC =
          SingularValueDecompositionMTJ
              .create(belief.getCovariance());
    }
    final Matrix SUG =
        StatisticsUtil.getDiagonalSqrt(svdC.getS(), 1e-7)
            .times(svdC.getU().transpose()).times(G.transpose());
    final Matrix Nw =
        StatisticsUtil.getDiagonalSqrt(
            this.modelCovariance.getSvd().getS(), 1e-7).times(
            this.modelCovariance.getSvd().getU().transpose());
    final int nN = SUG.getNumRows() + Nw.getNumRows();
    final int nM = SUG.getNumColumns();
    final Matrix M1 = MatrixFactory.getDefault().createMatrix(nN, nM);
    M1.setSubMatrix(0, 0, SUG);
    M1.setSubMatrix(SUG.getNumRows(), 0, Nw);

    final AbstractSingularValueDecomposition svdM =
        SingularValueDecompositionMTJ.create(M1);
    final Matrix S = StatisticsUtil.diagonalSquare(svdM.getS(), 1e-7);

    final AbstractSingularValueDecomposition svdR =
        new SimpleSingularValueDecomposition(svdM.getVtranspose()
            .transpose(), S, svdM.getVtranspose());

    final Matrix R;
    if (belief instanceof SvdMultivariateGaussian) {
      R = new SvdMatrix(svdR);
    } else {
      R = svdR.getU().times(svdR.getS()).times(svdR.getVtranspose());
    }
    /*
     * Check that we maintain numerical accuracy for our given model
     * design (in which the state covariances are always degenerate).
     */
    //    Preconditions.checkState((belief.getInputDimensionality() != 2 || svdR.rank() == 1)
    //        && (belief.getInputDimensionality() != 4 || svdR.rank() == 2));
    //    Preconditions.checkState(svdR.getU().getNumRows() == 2
    //        || svdR.getU().getNumRows() == 4);

    belief.setMean(G.times(belief.getMean()));
    belief.setCovariance(R);

    Preconditions.checkState(belief.getCovariance().isSquare()
        && belief.getCovariance().isSymmetric());
  }

  public void
      setMeasurementCovariance(SvdMatrix measurementCovariance) {
    this.measurementCovariance = measurementCovariance;
  }

  /**
   * Setter for model
   * 
   * @param model
   *          Motion model of the underlying system.
   */
  public void setModel(LinearDynamicalSystem model) {
    this.model = model;
  }

  public void setModelCovariance(SvdMatrix modelCovariance) {
    this.modelCovariance = modelCovariance;
  }

  @Override
  public void update(SvdMultivariateGaussian target, Vector data) {
    this.measure(target, data);
  }

}
