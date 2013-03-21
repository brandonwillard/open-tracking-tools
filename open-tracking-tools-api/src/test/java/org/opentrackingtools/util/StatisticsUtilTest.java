package org.opentrackingtools.util;

import org.testng.annotations.Test;
import org.testng.annotations.BeforeMethod;
import org.testng.AssertJUnit;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.decomposition.CholeskyDecompositionMTJ;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;


public class StatisticsUtilTest {

  @BeforeMethod
  public void setUp() throws Exception {

  }

  @Test
  public void testCovSqrt() {
    final Matrix mat =
        MatrixFactory.getDefault().copyArray(
            new double[][] { { 0.0001d, 0.00009d },
                { 0.00009d, 0.0001d } });

    final Matrix chol =
        CholeskyDecompositionMTJ.create((DenseMatrix) mat)
            .getR();
    final Matrix covSqrt =
        StatisticsUtil.rootOfSemiDefinite(mat);
    final Matrix localChol = StatisticsUtil.getCholR(mat);

    AssertJUnit.assertTrue(chol.equals(localChol, 1e-5));

    final Random rng = new Random(1234567);

    MultivariateGaussian.sample(VectorFactory.getDefault()
        .copyArray(new double[] { 0, 0 }), covSqrt, rng);

    rng.setSeed(1234567);

    MultivariateGaussian.sample(VectorFactory.getDefault()
        .copyArray(new double[] { 0, 0 }), chol, rng);

    final MultivariateGaussian.SufficientStatistic ss1 =
        new MultivariateGaussian.SufficientStatistic();

    final MultivariateGaussian.SufficientStatistic ss2 =
        new MultivariateGaussian.SufficientStatistic();

    for (int i = 0; i < 500000; i++) {
      final Vector localSmpl1 =
          MultivariateGaussian.sample(VectorFactory
              .getDefault()
              .copyArray(new double[] { 0, 0 }), covSqrt,
              rng);
      final Vector localSmpl2 =
          MultivariateGaussian.sample(VectorFactory
              .getDefault()
              .copyArray(new double[] { 0, 0 }), chol, rng);
      ss1.update(localSmpl1);
      ss2.update(localSmpl2);
    }

    final Matrix cov1 = ss1.getCovariance();
    ss2.getCovariance();

    AssertJUnit.assertTrue(ss1.getMean().isZero(1e-5));
    AssertJUnit.assertTrue(mat.equals(cov1, 1e-5));
  }

  @Test
  public void testInvWishartSampling() {
    final Random rng = //new Random();
        new Random(123456789);

    final Matrix mean =
        MatrixFactory.getDefault().copyArray(
            new double[][] { { 100d, 0d }, { 0d, 100d } });
    final InverseWishartDistribution invWish =
        new InverseWishartDistribution(MatrixFactory
            .getDefault().copyArray(
                new double[][] { { 1700d, 0d },
                    { 0d, 1700d } }), 20);

    MultivariateGaussian.SufficientStatistic ss =
        new MultivariateGaussian.SufficientStatistic();
    for (int j = 0; j < 10; j++) {
      ss = new MultivariateGaussian.SufficientStatistic();
      for (int i = 0; i < 10000; i++) {
        final Matrix smpl2 =
            StatisticsUtil.sampleInvWishart(invWish, rng);
        ss.update(mean.minus(smpl2).convertToVector());
      }
      System.out.println(ss.getMean());
    }

    AssertJUnit.assertTrue(Math.abs(ss.getMean().sum()) < 1d);

  }

}
