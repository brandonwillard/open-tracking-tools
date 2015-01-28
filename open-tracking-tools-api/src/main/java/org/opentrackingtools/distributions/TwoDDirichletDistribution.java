package org.opentrackingtools.distributions;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Iterators;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;

public class TwoDDirichletDistribution extends DirichletDistribution {

  @Override
  public TwoDDirichletDistribution clone() {
    return (TwoDDirichletDistribution)super.clone();
  }

  public TwoDDirichletDistribution() {
    super();
    // TODO Auto-generated constructor stub
  }

  public TwoDDirichletDistribution(int dimensionality) {
    super(dimensionality);
    // TODO Auto-generated constructor stub
  }

  public TwoDDirichletDistribution(Vector parameters) {
    super(parameters);
    // TODO Auto-generated constructor stub
  }

  public TwoDDirichletDistribution(DirichletDistribution other) {
    super(other);
    // TODO Auto-generated constructor stub
  }

  @Override
  public Vector getMean() {

    /**
     * super disgusting hack for 2d degenerate dirichlet.
     * since one entry is inf, the mean will return a NaN (due to 
     * normalization), but we really want a 0 and 1.
     */
    for (int i = 0; i < this.parameters.getDimensionality(); i++) {
      if (Double.isInfinite(this.parameters.getElement(i))) {
        Vector result = VectorFactory.getDefault().createVector(this.parameters.getDimensionality());
        result.setElement(i, 1d);
        return result; 
      }
    }
    return super.getMean();
  }

}
