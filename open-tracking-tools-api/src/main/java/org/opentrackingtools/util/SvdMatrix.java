package org.opentrackingtools.util;

import no.uib.cipr.matrix.DenseMatrix;

import java.text.NumberFormat;
import java.util.Iterator;

import gov.sandia.cognition.math.ComplexNumber;
import gov.sandia.cognition.math.MathUtil;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixEntry;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.decomposition.AbstractSingularValueDecomposition;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJMatrix;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJVector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.math.matrix.mtj.DenseVectorFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.SingularValueDecompositionMTJ;
import gov.sandia.cognition.util.ObjectUtil;

/**
 * This class is simply a this for a matrix object with
 * the addition of the this's SVD object.
 * @author bwillard
 *
 */
public class SvdMatrix extends AbstractMTJMatrix {
  
  private static final long serialVersionUID = -1191322577584088530L;
  
  protected AbstractSingularValueDecomposition svd;

  public SvdMatrix(AbstractSingularValueDecomposition svd) {
    super(new DenseMatrix(svd.getU().times(svd.getS()).times(svd.getVtranspose()).toArray()));
    this.svd = svd;
  }

  protected SvdMatrix(Matrix other, AbstractSingularValueDecomposition svd) {
    super(new DenseMatrix(other.toArray()));
    this.svd = svd;
  }

  public SvdMatrix(Matrix other) {
    super(new DenseMatrix(other.toArray()));
    if (other instanceof SvdMatrix) {
      this.svd = ((SvdMatrix)other).svd;
    } else {
      this.svd = SingularValueDecompositionMTJ.create(other);
    }
  }

  public Iterator<MatrixEntry> iterator() {
    return super.iterator();
  }

  public SvdMatrix clone() {
    SvdMatrix clone = new SvdMatrix(super.clone(), ObjectUtil.cloneSmart(svd));
    return clone;
  }

  public int getNumRows() {
    return super.getNumRows();
  }

  public int getNumColumns() {
    return super.getNumColumns();
  }

  public double getElement(int rowIndex, int columnIndex) {
    return super.getElement(rowIndex, columnIndex);
  }

  public boolean equals(Object other) {
    return super.equals(other);
  }

  public boolean equals(Matrix other, double effectiveZero) {
    return super.equals(other, effectiveZero);
  }

  public void setElement(int rowIndex, int columnIndex, double value) {
    super.setElement(rowIndex, columnIndex, value);
    this.svd = null;
  }

  public SvdMatrix plus(Matrix other) {
    return new SvdMatrix(super.plus(other));
  }

  public SvdMatrix getSubMatrix(int minRow, int maxRow, int minColumn,
      int maxColumn) {
    int numRows = maxRow - minRow + 1;
    if (numRows <= 0)
    {
        throw new IllegalArgumentException( "minRow " + minRow +
            " >= maxRow " + maxRow );
    }
    int numColumns = maxColumn - minColumn + 1;
    if (numColumns <= 0)
    {
        throw new IllegalArgumentException( "minCol " + minColumn +
            " >= maxCol " + maxColumn );
    }
    gov.sandia.cognition.math.matrix.mtj.DenseMatrix submatrix = DenseMatrixFactoryMTJ.INSTANCE.createMatrix( numRows, numColumns );
    super.getSubMatrixInto(
        minRow, maxRow, minColumn, maxColumn, submatrix );

    return new SvdMatrix(submatrix);
  }

  public void plusEquals(Matrix other) {
    super.plusEquals(other);
    this.svd = null;
  }

  public SvdMatrix minus(Matrix other) {
    return new SvdMatrix(super.minus(other));
  }

  public void minusEquals(Matrix other) {
    super.minusEquals(other);
    this.svd = null;
  }

  public void setSubMatrix(int minRow, int minColumn, Matrix submatrix) {
    super.setSubMatrix(minRow, minColumn, submatrix);
    this.svd = null;
  }

  public SvdMatrix dotTimes(Matrix other) {
    return new SvdMatrix(super.dotTimes(other));
  }

  public void dotTimesEquals(Matrix other) {
    super.dotTimesEquals(other);
    this.svd = null;
  }

  public boolean isSymmetric() {
    if (this.svd != null)
      return this.svd.getU().equals(this.svd.getVtranspose().transpose());
    else
      return super.isSymmetric();
  }

  public boolean isSymmetric(double effectiveZero) {
    if (this.svd != null)
      return this.svd.getU().equals(this.svd.getVtranspose().transpose(), effectiveZero);
    else
      return super.isSymmetric(effectiveZero);
  }

  public SvdMatrix scale(double scaleFactor) {
    return new SvdMatrix(super.scale(scaleFactor));
  }

  public boolean checkSameDimensions(Matrix otherMatrix) {
    return super.checkSameDimensions(otherMatrix);
  }

  public void scaleEquals(double scaleFactor) {
    super.scaleEquals(scaleFactor);
    this.svd = null;
  }

  public SvdMatrix scaledPlus(double scaleFactor, Matrix other) {
    return new SvdMatrix(super.scaledPlus(scaleFactor, other));
  }

  public void assertSameDimensions(Matrix otherMatrix) {
    super.assertSameDimensions(otherMatrix);
  }

  public boolean checkMultiplicationDimensions(Matrix postMultiplicationMatrix) {
    return super.checkMultiplicationDimensions(postMultiplicationMatrix);
  }

  public void scaledPlusEquals(double scaleFactor, Matrix other) {
    super.scaledPlusEquals(scaleFactor, other);
    this.svd = null;
  }

  public SvdMatrix transpose() {
    SvdMatrix transMat; 
    if (this.svd != null) {
      AbstractSingularValueDecomposition svdTrans = new SimpleSingularValueDecomposition(
          this.getSvd().getVtranspose().transpose(), this.getSvd().getS().transpose(), this.getSvd().getU().transpose());
      final AbstractMTJMatrix tMatrix = DenseMatrixFactoryMTJ.INSTANCE.createMatrix(this.getNumColumns(), this.getNumRows());
      super.transposeInto(tMatrix);
      transMat = new SvdMatrix(tMatrix, svdTrans);
    } else {
      final AbstractMTJMatrix tMatrix = DenseMatrixFactoryMTJ.INSTANCE.createMatrix(this.getNumColumns(), this.getNumRows());
      super.transposeInto(tMatrix);
      transMat = new SvdMatrix(tMatrix);
    }
    return transMat;
  }

  public SvdMatrix scaledMinus(double scaleFactor, Matrix other) {
    return new SvdMatrix(super.scaledMinus(scaleFactor, other));
  }

  public SvdMatrix inverse() {
    return new SvdMatrix(
        new SimpleSingularValueDecomposition(this.getSvd().getVtranspose().transpose(),
            StatisticsUtil.diagonalInverse(this.getSvd().getS(), 1e-7), 
            this.getSvd().getU().transpose()));
  }

  public SvdMatrix pseudoInverse() {
    return this.inverse();
  }

  public SvdMatrix pseudoInverse(double effectiveZero) {
    return new SvdMatrix(this.getSvd().pseudoInverse(effectiveZero));
  }

  public void scaledMinusEquals(double scaleFactor, Matrix other) {
    super.scaledMinusEquals(scaleFactor, other);
    this.svd = null;
  }

  public ComplexNumber logDeterminant() {
    return super.logDeterminant();
  }

  public double trace() {
    return super.trace();
  }

  public SvdMatrix negative() {
    return new SvdMatrix(super.negative());
  }

  public int rank() {
    return this.getSvd().rank();
  }

  public void negativeEquals() {
    super.negativeEquals();
    this.svd = null;
  }

  public void zero() {
    super.zero();
    this.svd = null;
  }

  public int rank(double effectiveZero) {
    return this.getSvd().effectiveRank(effectiveZero);
  }

  public boolean isZero() {
    return super.isZero();
  }

  public boolean isZero(double effectiveZero) {
    return super.isZero(effectiveZero);
  }

  public double normFrobenius() {
    return super.normFrobenius();
  }

  public boolean isSquare() {
    return super.isSquare();
  }

  public Matrix solve(Matrix B) {
    return super.solve(B);
  }

  public Vector solve(Vector b) {
    return super.solve(b);
  }

  public void identity() {
    super.identity();
    this.svd = null;
  }

  public Vector getColumn(int columnIndex) {
    int M = this.getNumRows();
    DenseVector columnVector = DenseVectorFactoryMTJ.INSTANCE.createVector( M );
    this.getColumnInto( columnIndex, columnVector );
    return columnVector;
  }

  public Vector getRow(int rowIndex) {
    int M = this.getNumColumns();
    DenseVector rowVector = DenseVectorFactoryMTJ.INSTANCE.createVector( M );
    this.getRowInto( rowIndex, rowVector );
    return rowVector;
  }

  public void setColumn(int columnIndex, Vector columnVector) {
    super.setColumn(columnIndex, columnVector);
    this.svd = null;
  }

  public void setRow(int rowIndex, Vector rowVector) {
    super.setRow(rowIndex, rowVector);
    this.svd = null;
  }

  public Vector sumOfRows() {
    return super.sumOfRows();
  }

  public Vector sumOfColumns() {
    return super.sumOfColumns();
  }

  public void convertFromVector(Vector parameters) {
    super.convertFromVector(parameters);
    this.svd = null;
  }

  public DenseVector convertToVector() {
    return super.convertToVector();
  }

  public double[][] toArray() {
    return super.toArray();
  }

  @Override
  public String toString(NumberFormat format) {
    return this.toString();
  }

  public AbstractSingularValueDecomposition getSvd() {
    if (svd == null)
      this.svd = SingularValueDecompositionMTJ.create(this);
    return svd;
  }

  public void setSvd(AbstractSingularValueDecomposition svd) {
    this.svd = svd;
    this.setInternalMatrix(
        new DenseMatrix(svd.getU().times(svd.getS()).times(svd.getVtranspose()).toArray()));
  }

  @Override
  public SvdMatrix times(AbstractMTJMatrix matrix) {
    int returnRows = this.getNumRows();
    int returnColumns = matrix.getNumColumns();
    gov.sandia.cognition.math.matrix.mtj.DenseMatrix retval = DenseMatrixFactoryMTJ.INSTANCE.createMatrix(returnRows, returnColumns);
    super.timesInto( matrix, retval );
    return new SvdMatrix(retval);
  }

  @Override
  public AbstractMTJVector times(AbstractMTJVector vector) {
    gov.sandia.cognition.math.matrix.mtj.DenseVector answer = DenseVectorFactoryMTJ.INSTANCE.createVector(
        this.getNumRows());
    this.timesInto( vector, answer );
    return answer;
  }

  /**
   * From MTJ DenseMatrix
   */
  @Override
  public String toString() {
    final StringBuilder result =
        new StringBuilder(MathUtil.checkedMultiply(10, 
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

  
}
