package org.opentrackingtools.util;

import com.google.common.base.Preconditions;

import org.apache.commons.lang3.builder.ToStringBuilder;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;

public class SimpleSingularValueDecomposition extends
    AbstractSingularValueDecomposition {
  public SimpleSingularValueDecomposition(Matrix U, Matrix S,
      Matrix Vtranspose) {
    super(U, S, Vtranspose);
    Preconditions.checkState(U.getNumColumns() == S.getNumRows()
        && S.getNumColumns() == Vtranspose.getNumRows());
  }

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("getS()", getS());
    builder.append("conditionNumber()", conditionNumber());
    builder.append("rank()", rank());
    return builder.toString();
  }
  
  
}