package org.opentrackingtools.estimators;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.DenseVectorFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.SingularValueDecompositionMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.KalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import no.uib.cipr.matrix.DenseMatrix;
import no.uib.cipr.matrix.UpperSymmBandMatrix;

import org.opentrackingtools.distributions.TruncatedRoadGaussian;

import com.google.common.base.Preconditions;
import com.statslibextensions.math.matrix.SvdMatrix;
import com.statslibextensions.math.matrix.decomposition.SimpleSingularValueDecomposition;
import com.statslibextensions.statistics.bayesian.DlmUtils;
import com.statslibextensions.statistics.distribution.SvdMultivariateGaussian;

/**
 * A Kalman filter specific to truncated edges (roads).
 * 
 * @author bwillard
 * 
 */
public class TruncatedRoadKalmanFilter extends KalmanFilter {

  /**
   * Default autonomous dimension, {@value} .
   */
  public static final int DEFAULT_DIMENSION = 1;

  private static final long serialVersionUID = 8046227346384488242L;

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
    super(model, modelCovariance, measurementCovariance);
    this.timeDiff = timeDiff;
  }

  @Override
  public TruncatedRoadKalmanFilter clone() {
    final TruncatedRoadKalmanFilter clone =
        (TruncatedRoadKalmanFilter) super.clone();
    clone.timeDiff = this.timeDiff;
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
  public void
      measure(MultivariateGaussian belief, Vector observation) {
//    DlmUtils.svdForwardFilter(observation, belief, this);
    DlmUtils.schurForwardFilter(observation, belief, this);
    // TODO FIXME: hack for a numerical issue
    if (!belief.getCovariance().isSymmetric()) {
      UpperSymmBandMatrix symmat = new UpperSymmBandMatrix(
          ((AbstractMTJMatrix) belief.getCovariance()).getInternalMatrix(), 
          belief.getInputDimensionality());
      belief.setCovariance(((DenseMatrixFactoryMTJ) MatrixFactory.getDenseDefault())
          .createWrapper(new DenseMatrix(symmat)));
    }
//    super.measure(belief, observation);
  }

  @Override
  public void predict(MultivariateGaussian belief) {
//    DlmUtils.svdPredict(belief, this);
    super.predict(belief);
  }

  @Override
  public void
      setMeasurementCovariance(Matrix measurementCovariance) {
    Preconditions.checkArgument(measurementCovariance instanceof SvdMatrix);
    super.setMeasurementCovariance(measurementCovariance);
  }

  @Override
  public void
      setModelCovariance(Matrix modelCovariance) {
    Preconditions.checkArgument(modelCovariance instanceof SvdMatrix);
    super.setModelCovariance(modelCovariance);
  }

}
