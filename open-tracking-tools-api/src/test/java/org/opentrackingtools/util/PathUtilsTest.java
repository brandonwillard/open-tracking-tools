package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.mockito.Mockito;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.internal.junit.ArrayAsserts;

import com.google.common.collect.Iterables;
import com.statslibextensions.math.matrix.SvdMatrix;
import com.statslibextensions.statistics.distribution.SvdMultivariateGaussian;
import com.vividsolutions.jts.geom.Coordinate;

public class PathUtilsTest {

  @DataProvider
  private static final Object[][] stateData() {

    return new Object[][] {
        {
            VectorFactory.getDefault().copyArray(
                new double[] { -9.999992857195395E7, 0.0,
                    -13.537576549229717, 0.0 }),
            1e8,
            VectorFactory.getDefault().copyArray(
                new double[] { -9.999992857195395E7 - 1e8, 0.0 }),
            VectorFactory.getDefault().copyArray(
                new double[] { -9.999992857195395E7, 0d, 0d, 0.0 }) },
        {
            VectorFactory.getDefault().copyArray(
                new double[] { -9.999992857195395E1, 0.0,
                    -13.537576549229717, 0.0 }),
            1e2,
            VectorFactory.getDefault().copyArray(
                new double[] { -9.999992857195395E1 - 1e2, 0.0 }),
            VectorFactory.getDefault().copyArray(
                new double[] { -9.999992857195395E1, 0d, 0d, 0.0 }) } };
  }

  @Test(dataProvider = "stateData")
  public void testProjection(Vector from, double length,
    Vector roadTo, Vector groundTo) {

    final InferenceGraph graph = Mockito.mock(InferenceGraph.class);
    final Path path =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(-length, 0d), new Coordinate(0d, 0d),
            new Coordinate(length, 0d));

    final MultivariateGaussian belief =
        new SvdMultivariateGaussian(from, new SvdMatrix(MatrixFactory
            .getDefault().copyArray(
                new double[][] {
                    { 91.64766085510277, 0.0, -10.790534809853966,
                        0.0 },
                    { 0.0, 0.0, 0.0, 0.0 },
                    { -10.790534809853973, 0.0, 110.08645314343424,
                        0.0 }, { 0.0, 0.0, 0.0, 0.0 } })));

    final PathEdge pathEdge = Iterables.getLast(path.getPathEdges());
    final MultivariateGaussian projBelief =
        PathUtils.getRoadBeliefFromGround(belief, path.getGeometry(),
            path.isBackward(), pathEdge.getLine(),
            pathEdge.getDistToStartOfEdge(), true, null, null);

    ArrayAsserts.assertArrayEquals("convert to road",
        roadTo.toArray(), projBelief.getMean().toArray(), 1e-1);

    PathUtils.convertToGroundBelief(projBelief, pathEdge, false,
        false, true);

    ArrayAsserts.assertArrayEquals("convert back to ground",
        groundTo.toArray(), projBelief.getMean().toArray(), 1e-1);
  }

}
