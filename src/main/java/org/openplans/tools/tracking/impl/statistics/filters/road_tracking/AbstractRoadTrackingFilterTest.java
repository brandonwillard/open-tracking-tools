package org.openplans.tools.tracking.impl.statistics.filters.road_tracking;

import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import org.junit.Before;
import org.junit.Test;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.vividsolutions.jts.geom.LineString;

public class AbstractRoadTrackingFilterTest {

  @Before
  public void setUp() throws Exception {
  }

  @Test
  public void test() {

    final OtpGraph graph = mock(OtpGraph.class);

    final Vector mean =
        VectorFactory.getDenseDefault().copyArray(
            new double[] { -86.01050120108039, -51.44579396037449 });
    final Vector projMean =
        VectorFactory.getDenseDefault().copyArray(
            new double[] { 324480.0240871321, -17.961092165568616,
                4306914.231357716, -48.208597619441846 });
    final LineString edgeGeom =
        GeometryUtils.makeLineString(324470.0865109131,
            4306887.558335339, 324480.0240871321, 4306914.231357716,
            324487.9070333349, 4306923.394792204, 324497.3591208192,
            4306927.015912709, 324514.4626894819, 4306930.933664588);
    final StreetVertex v1 = mock(StreetVertex.class);
    final PlainStreetEdge edge =
        new PlainStreetEdge(v1, v1, edgeGeom, "test", 0d, null, false);
    final InferredEdge infEdge =
        new InferredEdge(edge, 680402, graph);
    final PathEdge pathEdge = PathEdge.getEdge(infEdge, -46d, true);

    assertTrue(AbstractRoadTrackingFilter.isIsoMapping(mean,
        projMean, pathEdge));
  }

}
