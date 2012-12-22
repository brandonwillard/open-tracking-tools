package org.openplans.tools.tracking.impl.statistics;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.decomposition.CholeskyDecompositionMTJ;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.junit.Before;
import org.junit.Test;
import org.openplans.tools.tracking.impl.TrackingTestUtils;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.filters.particle_learning.VehicleTrackingPLFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class StatisticsUtilTest {


  @Before
  public void setUp() throws Exception {
    
  }

  @Test
  public void testInvWishartSampling() {
    Random rng = //new Random();
        new Random(123456789);
    
    final Matrix mean = MatrixFactory.getDefault().copyArray(
      new double[][] { 
          {100d, 0d},
          {0d, 100d}});
    InverseWishartDistribution invWish = new
        InverseWishartDistribution(
            MatrixFactory.getDefault().copyArray(
            new double[][] { 
                {1700d, 0d},
                {0d, 1700d}}), 20);
    
    MultivariateGaussian.SufficientStatistic ss = 
        new MultivariateGaussian.SufficientStatistic();
    for (int j = 0; j < 10; j++) {
      ss = new MultivariateGaussian.SufficientStatistic();
      for (int i = 0; i < 10000; i++) {
        final Matrix smpl2 = StatisticsUtil.sampleInvWishart(
            invWish, rng);
        ss.update(mean.minus(smpl2).convertToVector());
      } 
      System.out.println(ss.getMean());
    }
    
    assertTrue(Math.abs(ss.getMean().sum()) < 1d);
    
  }
  
  @Test
  public void testCovSqrt() {
    final Matrix mat = MatrixFactory.getDefault().copyArray(
      new double[][] { 
          {0.0001d, 0.00009d},
          {0.00009d, 0.0001d}});
    
    
    final Matrix chol = CholeskyDecompositionMTJ.create(
        (DenseMatrix) mat).getR();
    final Matrix covSqrt = StatisticsUtil.rootOfSemiDefinite(mat);
    final Matrix localChol = StatisticsUtil.getCholR(mat);
    
    assertTrue(chol.equals(localChol, 1e-5));
    
    final Random rng = new Random(1234567);
    
    final Vector smpl1 = MultivariateGaussian.sample(
        VectorFactory.getDefault().copyArray(
            new double[] {0, 0}), covSqrt, rng);
    
    rng.setSeed(1234567);
    
    final Vector smpl2 = MultivariateGaussian.sample(
        VectorFactory.getDefault().copyArray(
            new double[] {0, 0}), chol, rng);
    
    MultivariateGaussian.SufficientStatistic ss1 = 
        new MultivariateGaussian.SufficientStatistic();
    
    MultivariateGaussian.SufficientStatistic ss2 = 
        new MultivariateGaussian.SufficientStatistic();
    
    for (int i = 0; i < 500000; i++) {
      final Vector localSmpl1 = MultivariateGaussian.sample(
          VectorFactory.getDefault().copyArray(
              new double[] {0, 0}), covSqrt, rng);
      final Vector localSmpl2 = MultivariateGaussian.sample(
          VectorFactory.getDefault().copyArray(
              new double[] {0, 0}), chol, rng);
      ss1.update(localSmpl1);
      ss2.update(localSmpl2);
    }
    
    final Matrix cov1 = ss1.getCovariance();
    final Matrix cov2 = ss2.getCovariance();
    
    assertTrue(ss1.getMean().isZero(1e-5));
    assertTrue(mat.equals(cov1, 1e-5));
  }

}
