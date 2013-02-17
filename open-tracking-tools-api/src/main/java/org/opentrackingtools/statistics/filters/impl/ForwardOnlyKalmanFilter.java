package org.opentrackingtools.statistics.filters.impl;

import org.opentrackingtools.statistics.distributions.impl.TruncatedRoadGaussian;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

public class ForwardOnlyKalmanFilter extends AdjKalmanFilter {

  private static final long serialVersionUID = 3493849809266428560L;

  public ForwardOnlyKalmanFilter() {
  }

  public ForwardOnlyKalmanFilter(int dim) {
    super(dim);
  }

  public ForwardOnlyKalmanFilter(LinearDynamicalSystem model,
    Matrix modelCovariance, Matrix measurementCovariance) {
    super(model, modelCovariance, measurementCovariance);
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    return new TruncatedRoadGaussian(this.model.getState(),
        this.getModelCovariance(), 15, 0);
  }

  @Override
  public void
      measure(MultivariateGaussian belief, Vector observation) {
    super.measure(belief, observation);
  }

  @Override
  public void predict(MultivariateGaussian belief) {
    super.predict(belief);
  }

}
