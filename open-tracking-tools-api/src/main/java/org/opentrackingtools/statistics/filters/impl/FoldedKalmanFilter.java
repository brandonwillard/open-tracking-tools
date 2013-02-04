package org.opentrackingtools.statistics.filters.impl;

import org.opentrackingtools.statistics.distributions.impl.AdjMultivariateGaussian;
import org.opentrackingtools.statistics.distributions.impl.FoldedMultivariateGaussian;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

public class FoldedKalmanFilter extends AdjKalmanFilter {

  private static final long serialVersionUID = 3493849809266428560L;

  public FoldedKalmanFilter() {
  }

  public FoldedKalmanFilter(int dim) {
    super(dim);
  }

  public FoldedKalmanFilter(LinearDynamicalSystem model,
    Matrix modelCovariance, Matrix measurementCovariance) {
    super(model, modelCovariance, measurementCovariance);
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    return new FoldedMultivariateGaussian(this.model.getState(),
        this.getModelCovariance());
  }

  @Override
  public void
      measure(MultivariateGaussian belief, Vector observation) {
    super.measure(belief, observation);
    for (VectorEntry entry : belief.getMean()) {
      if (entry.getValue() < 0d)
        entry.setValue(0d);
    }
  }

  @Override
  public void predict(MultivariateGaussian belief) {
    super.predict(belief);
    for (VectorEntry entry : belief.getMean()) {
      if (entry.getValue() < 0d)
        entry.setValue(0d);
    }
  }

}
