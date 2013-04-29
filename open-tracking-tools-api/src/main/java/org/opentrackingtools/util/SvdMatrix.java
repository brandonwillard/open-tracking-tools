package org.opentrackingtools.util;

import gov.sandia.cognition.math.ComplexNumber;
import gov.sandia.cognition.math.MathUtil;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixEntry;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJMatrix;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJVector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.math.matrix.mtj.DenseVectorFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.SingularValueDecompositionMTJ;
import gov.sandia.cognition.util.ObjectUtil;

import java.text.NumberFormat;
import java.util.Iterator;

import no.uib.cipr.matrix.DenseMatrix;

/**
 * This class is simply a matrix object that carries the SVD components. When
 * possible, operations will be performed with respect to those components.
 * 
 * TODO implement this for a symmetric matrix.
 * 
 * @author bwillard
 * 
 */
public class SvdMatrix extends AbstractMTJMatrix {

  private static final long serialVersionUID = -1191322577584088530L;

  protected AbstractSingularValueDecomposition svd;

  public SvdMatrix(AbstractSingularValueDecomposition svd) {
    super(new DenseMatrix(svd.getU().times(svd.getS())
        .times(svd.getVtranspose()).toArray()));
    this.svd = svd;
  }

  public SvdMatrix(Matrix other) {
    super(new DenseMatrix(other.toArray()));
    if (other instanceof SvdMatrix) {
      this.svd = ((SvdMatrix) other).svd;
    } else {
      this.svd = SingularValueDecompositionMTJ.create(other);
    }
  }

  protected SvdMatrix(Matrix other,
    AbstractSingularValueDecomposition svd) {
    super(new DenseMatrix(other.toArray()));
    this.svd = svd;
  }

  @Override
  public void assertSameDimensions(Matrix otherMatrix) {
    super.assertSameDimensions(otherMatrix);
  }

  @Override
  public boolean checkMultiplicationDimensions(
    Matrix postMultiplicationMatrix) {
    return super
        .checkMultiplicationDimensions(postMultiplicationMatrix);
  }

  @Override
  public boolean checkSameDimensions(Matrix otherMatrix) {
    return super.checkSameDimensions(otherMatrix);
  }

  @Override
  public SvdMatrix clone() {
    final SvdMatrix clone =
        new SvdMatrix(super.clone(), ObjectUtil.cloneSmart(this.svd));
    return clone;
  }

  @Override
  public void convertFromVector(Vector parameters) {
    super.convertFromVector(parameters);
    this.svd = null;
  }

  @Override
  public DenseVector convertToVector() {
    return super.convertToVector();
  }

  @Override
  public SvdMatrix dotTimes(Matrix other) {
    return new SvdMatrix(super.dotTimes(other));
  }

  @Override
  public void dotTimesEquals(Matrix other) {
    super.dotTimesEquals(other);
    this.svd = null;
  }

  @Override
  public boolean equals(Matrix other, double effectiveZero) {
    return super.equals(other, effectiveZero);
  }

  @Override
  public boolean equals(Object other) {
    return super.equals(other);
  }

  @Override
  public Vector getColumn(int columnIndex) {
    final int M = this.getNumRows();
    final DenseVector columnVector =
        DenseVectorFactoryMTJ.INSTANCE.createVector(M);
    this.getColumnInto(columnIndex, columnVector);
    return columnVector;
  }

  @Override
  public double getElement(int rowIndex, int columnIndex) {
    return super.getElement(rowIndex, columnIndex);
  }

  @Override
  public int getNumColumns() {
    return super.getNumColumns();
  }

  @Override
  public int getNumRows() {
    return super.getNumRows();
  }

  @Override
  public Vector getRow(int rowIndex) {
    final int M = this.getNumColumns();
    final DenseVector rowVector =
        DenseVectorFactoryMTJ.INSTANCE.createVector(M);
    this.getRowInto(rowIndex, rowVector);
    return rowVector;
  }

  @Override
  public SvdMatrix getSubMatrix(int minRow, int maxRow,
    int minColumn, int maxColumn) {
    final int numRows = maxRow - minRow + 1;
    if (numRows <= 0) {
      throw new IllegalArgumentException("minRow " + minRow
          + " >= maxRow " + maxRow);
    }
    final int numColumns = maxColumn - minColumn + 1;
    if (numColumns <= 0) {
      throw new IllegalArgumentException("minCol " + minColumn
          + " >= maxCol " + maxColumn);
    }
    final gov.sandia.cognition.math.matrix.mtj.DenseMatrix submatrix =
        DenseMatrixFactoryMTJ.INSTANCE.createMatrix(numRows,
            numColumns);
    super.getSubMatrixInto(minRow, maxRow, minColumn, maxColumn,
        submatrix);

    return new SvdMatrix(submatrix);
  }

  public AbstractSingularValueDecomposition getSvd() {
    if (this.svd == null) {
      this.svd = SingularValueDecompositionMTJ.create(this);
    }
    return this.svd;
  }

  @Override
  public void identity() {
    super.identity();
    this.svd = null;
  }

  @Override
  public SvdMatrix inverse() {
    return new SvdMatrix(new SimpleSingularValueDecomposition(this
        .getSvd().getVtranspose().transpose(),
        StatisticsUtil.diagonalInverse(this.getSvd().getS(), 1e-7),
        this.getSvd().getU().transpose()));
  }

  @Override
  public boolean isSparse() {
    return false;
  }

  @Override
  public boolean isSquare() {
    return super.isSquare();
  }

  @Override
  public boolean isSymmetric() {
    if (this.svd != null) {
      return this.svd.getU().equals(
          this.svd.getVtranspose().transpose());
    } else {
      return super.isSymmetric();
    }
  }

  @Override
  public boolean isSymmetric(double effectiveZero) {
    if (this.svd != null) {
      return this.svd.getU().equals(
          this.svd.getVtranspose().transpose(), effectiveZero);
    } else {
      return super.isSymmetric(effectiveZero);
    }
  }

  @Override
  public boolean isZero() {
    return super.isZero();
  }

  @Override
  public boolean isZero(double effectiveZero) {
    return super.isZero(effectiveZero);
  }

  @Override
  public Iterator<MatrixEntry> iterator() {
    return super.iterator();
  }

  @Override
  public ComplexNumber logDeterminant() {
    return super.logDeterminant();
  }

  @Override
  public SvdMatrix minus(Matrix other) {
    return new SvdMatrix(super.minus(other));
  }

  @Override
  public void minusEquals(Matrix other) {
    super.minusEquals(other);
    this.svd = null;
  }

  @Override
  public SvdMatrix negative() {
    return new SvdMatrix(super.negative());
  }

  @Override
  public void negativeEquals() {
    super.negativeEquals();
    this.svd = null;
  }

  @Override
  public double normFrobenius() {
    return super.normFrobenius();
  }

  @Override
  public SvdMatrix plus(Matrix other) {
    return new SvdMatrix(super.plus(other));
  }

  @Override
  public void plusEquals(Matrix other) {
    super.plusEquals(other);
    this.svd = null;
  }

  @Override
  public SvdMatrix pseudoInverse() {
    return this.inverse();
  }

  @Override
  public SvdMatrix pseudoInverse(double effectiveZero) {
    return new SvdMatrix(this.getSvd().pseudoInverse(effectiveZero));
  }

  @Override
  public int rank() {
    return this.getSvd().rank();
  }

  @Override
  public int rank(double effectiveZero) {
    return this.getSvd().effectiveRank(effectiveZero);
  }

  @Override
  public SvdMatrix scale(double scaleFactor) {
    return new SvdMatrix(super.scale(scaleFactor));
  }

  @Override
  public SvdMatrix scaledMinus(double scaleFactor, Matrix other) {
    return new SvdMatrix(super.scaledMinus(scaleFactor, other));
  }

  @Override
  public void scaledMinusEquals(double scaleFactor, Matrix other) {
    super.scaledMinusEquals(scaleFactor, other);
    this.svd = null;
  }

  @Override
  public SvdMatrix scaledPlus(double scaleFactor, Matrix other) {
    return new SvdMatrix(super.scaledPlus(scaleFactor, other));
  }

  @Override
  public void scaledPlusEquals(double scaleFactor, Matrix other) {
    super.scaledPlusEquals(scaleFactor, other);
    this.svd = null;
  }

  @Override
  public void scaleEquals(double scaleFactor) {
    super.scaleEquals(scaleFactor);
    this.svd = null;
  }

  @Override
  public void setColumn(int columnIndex, Vector columnVector) {
    super.setColumn(columnIndex, columnVector);
    this.svd = null;
  }

  @Override
  public void setElement(int rowIndex, int columnIndex, double value) {
    super.setElement(rowIndex, columnIndex, value);
    this.svd = null;
  }

  @Override
  public void setRow(int rowIndex, Vector rowVector) {
    super.setRow(rowIndex, rowVector);
    this.svd = null;
  }

  @Override
  public void
      setSubMatrix(int minRow, int minColumn, Matrix submatrix) {
    super.setSubMatrix(minRow, minColumn, submatrix);
    this.svd = null;
  }

  public void setSvd(AbstractSingularValueDecomposition svd) {
    this.svd = svd;
    this.setInternalMatrix(new DenseMatrix(svd.getU()
        .times(svd.getS()).times(svd.getVtranspose()).toArray()));
  }

  @Override
  public Matrix solve(Matrix B) {
    return super.solve(B);
  }

  @Override
  public Vector solve(Vector b) {
    return super.solve(b);
  }

  @Override
  public Vector sumOfColumns() {
    return super.sumOfColumns();
  }

  @Override
  public Vector sumOfRows() {
    return super.sumOfRows();
  }

  @Override
  public SvdMatrix times(AbstractMTJMatrix matrix) {
    final int returnRows = this.getNumRows();
    final int returnColumns = matrix.getNumColumns();
    final gov.sandia.cognition.math.matrix.mtj.DenseMatrix retval =
        DenseMatrixFactoryMTJ.INSTANCE.createMatrix(returnRows,
            returnColumns);
    super.timesInto(matrix, retval);
    return new SvdMatrix(retval);
  }

  @Override
  public AbstractMTJVector times(AbstractMTJVector vector) {
    final gov.sandia.cognition.math.matrix.mtj.DenseVector answer =
        DenseVectorFactoryMTJ.INSTANCE
            .createVector(this.getNumRows());
    this.timesInto(vector, answer);
    return answer;
  }

  @Override
  public double[][] toArray() {
    return super.toArray();
  }

  /**
   * From MTJ DenseMatrix
   */
  @Override
  public String toString() {
    final StringBuilder result =
        new StringBuilder(MathUtil.checkedMultiply(
            10,
            MathUtil.checkedMultiply(this.getNumRows(),
                this.getNumColumns())));

    for (int i = 0; i < this.getNumRows(); i++) {
      for (int j = 0; j < this.getNumColumns(); j++) {
        result.append(" ");
        result.append(this.getElement(i, j));
      }
      result.append("\n");
    }

    return result.toString();
  }

  @Override
  public String toString(NumberFormat format) {
    return this.toString();
  }

  @Override
  public double trace() {
    return super.trace();
  }

  @Override
  public SvdMatrix transpose() {
    SvdMatrix transMat;
    if (this.svd != null) {
      final AbstractSingularValueDecomposition svdTrans =
          new SimpleSingularValueDecomposition(this.getSvd()
              .getVtranspose().transpose(), this.getSvd().getS()
              .transpose(), this.getSvd().getU().transpose());
      final AbstractMTJMatrix tMatrix =
          DenseMatrixFactoryMTJ.INSTANCE.createMatrix(
              this.getNumColumns(), this.getNumRows());
      super.transposeInto(tMatrix);
      transMat = new SvdMatrix(tMatrix, svdTrans);
    } else {
      final AbstractMTJMatrix tMatrix =
          DenseMatrixFactoryMTJ.INSTANCE.createMatrix(
              this.getNumColumns(), this.getNumRows());
      super.transposeInto(tMatrix);
      transMat = new SvdMatrix(tMatrix);
    }
    return transMat;
  }

  @Override
  public void zero() {
    super.zero();
    this.svd = null;
  }

}
