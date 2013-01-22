package org.opentrackingtools.util;

/*
 * Copy of JTS's LineBuilder.
 */

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.vividsolutions.jts.algorithm.PointLocator;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geomgraph.DirectedEdge;
import com.vividsolutions.jts.geomgraph.DirectedEdgeStar;
import com.vividsolutions.jts.geomgraph.Edge;
import com.vividsolutions.jts.geomgraph.Label;
import com.vividsolutions.jts.geomgraph.Node;
import com.vividsolutions.jts.operation.overlay.OverlayOp;
import com.vividsolutions.jts.util.Assert;

/**
 * Forms JTS LineStrings out of a the graph of {@link DirectedEdge}s created by
 * an {@link OverlayOp}.
 * 
 * @version 1.7
 */
public class EdgeLineBuilder {
  private final EdgeOverlayOp op;
  private final GeometryFactory geometryFactory;
  private final PointLocator ptLocator;

  private final List lineEdgesList = new ArrayList();
  private final List resultLineList = new ArrayList();

  public EdgeLineBuilder(EdgeOverlayOp op,
    GeometryFactory geometryFactory, PointLocator ptLocator) {
    this.op = op;
    this.geometryFactory = geometryFactory;
    this.ptLocator = ptLocator;
  }

  /**
   * @return a list of the LineStrings in the result of the specified overlay
   *         operation
   */
  public List build(int opCode) {
    findCoveredLineEdges();
    collectLines(opCode);
    //labelIsolatedLines(lineEdgesList);
    buildLines(opCode);
    return resultLineList;
  }

  private void buildLines(int opCode) {
    for (final Iterator it = lineEdgesList.iterator(); it
        .hasNext();) {
      final Edge e = (Edge) it.next();
      e.getLabel();
      final LineString line =
          geometryFactory.createLineString(e
              .getCoordinates());
      resultLineList.add(line);
      e.setInResult(true);
    }
  }

  /**
   * Collect edges from Area inputs which should be in the result but which have
   * not been included in a result area. This happens ONLY:
   * <ul>
   * <li>during an intersection when the boundaries of two areas touch in a line
   * segment
   * <li>OR as a result of a dimensional collapse.
   * </ul>
   */
  private void collectBoundaryTouchEdge(DirectedEdge de,
    int opCode, List edges) {
    final Label label = de.getLabel();
    if (de.isLineEdge())
      return; // only interested in area edges
    if (de.isVisited())
      return; // already processed
    if (de.isInteriorAreaEdge())
      return; // added to handle dimensional collapses
    if (de.getEdge().isInResult())
      return; // if the edge linework is already included, don't include it again

    // sanity check for labelling of result edgerings
    Assert.isTrue(!(de.isInResult() || de.getSym()
        .isInResult()) || !de.getEdge().isInResult());

    // include the linework if it's in the result of the operation
    if (EdgeOverlayOp.isResultOfOp(label, opCode)
        && opCode == EdgeOverlayOp.INTERSECTION) {
      edges.add(de.getEdge());
      de.setVisitedEdge(true);
    }
  }

  /**
   * Collect line edges which are in the result. Line edges are in the result if
   * they are not part of an area boundary, if they are in the result of the
   * overlay operation, and if they are not covered by a result area.
   * 
   * @param de
   *          the directed edge to test
   * @param opCode
   *          the overlap operation
   * @param edges
   *          the list of included line edges
   */
  private void collectLineEdge(DirectedEdge de, int opCode,
    List edges) {
    final Label label = de.getLabel();
    final Edge e = de.getEdge();
    // include L edges which are in the result
    if (de.isLineEdge()) {
      if (!de.isVisited()
          && EdgeOverlayOp.isResultOfOp(label, opCode)
          && !e.isCovered()) {
        //Debug.println("de: " + de.getLabel());
        //Debug.println("edge: " + e.getLabel());

        edges.add(e);
        de.setVisitedEdge(true);
      }
    }
  }

  private void collectLines(int opCode) {
    for (final Iterator it =
        op.getGraph().getEdgeEnds().iterator(); it
        .hasNext();) {
      final DirectedEdge de = (DirectedEdge) it.next();
      collectLineEdge(de, opCode, lineEdgesList);
      collectBoundaryTouchEdge(de, opCode, lineEdgesList);
    }
  }

  /**
   * Find and mark L edges which are "covered" by the result area (if any). L
   * edges at nodes which also have A edges can be checked by checking their
   * depth at that node. L edges at nodes which do not have A edges can be
   * checked by doing a point-in-polygon test with the previously computed
   * result areas.
   */
  private void findCoveredLineEdges() {
    // first set covered for all L edges at nodes which have A edges too
    for (final Iterator nodeit =
        op.getGraph().getNodes().iterator(); nodeit
        .hasNext();) {
      final Node node = (Node) nodeit.next();
      //node.print(System.out);
      ((DirectedEdgeStar) node.getEdges())
          .findCoveredLineEdges();
    }

    /**
     * For all L edges which weren't handled by the above, use a point-in-poly
     * test to determine whether they are covered
     */
    for (final Iterator it =
        op.getGraph().getEdgeEnds().iterator(); it
        .hasNext();) {
      final DirectedEdge de = (DirectedEdge) it.next();
      final Edge e = de.getEdge();
      if (de.isLineEdge() && !e.isCoveredSet()) {
        final boolean isCovered =
            op.isCoveredByA(de.getCoordinate());
        e.setCovered(isCovered);
      }
    }
  }

  /**
   * Label an isolated node with its relationship to the target geometry.
   */
  private void labelIsolatedLine(Edge e, int targetIndex) {
    final int loc =
        ptLocator.locate(e.getCoordinate(),
            op.getArgGeometry(targetIndex));
    e.getLabel().setLocation(targetIndex, loc);
  }

  private void labelIsolatedLines(List edgesList) {
    for (final Iterator it = edgesList.iterator(); it
        .hasNext();) {
      final Edge e = (Edge) it.next();
      final Label label = e.getLabel();
      //n.print(System.out);
      if (e.isIsolated()) {
        if (label.isNull(0))
          labelIsolatedLine(e, 0);
        else
          labelIsolatedLine(e, 1);
      }
    }
  }

}
