package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;

import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;

import com.google.common.base.Preconditions;

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
    final ToStringBuilder builder = new ToStringBuilder(this, ToStringStyle.SHORT_PREFIX_STYLE);
    builder.append("S", this.getS());
    builder.append("conditionNumber", this.conditionNumber());
    builder.append("rank", this.rank());
    return builder.toString();
  }

}